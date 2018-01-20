//
// Created by lucas on 11/29/17.
//

#ifndef ORB_SLAM2_OCCUPANCYGRID_H
#define ORB_SLAM2_OCCUPANCYGRID_H

#include "FrameSelector.h"
#include "PointModelDrawer.h"
#include "Octree.h"
#include "SE3.h"
#include <queue>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap/math/Pose6D.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree.h>
#include <tsdf.cuh>

namespace ORB_SLAM2 {
    class FrameSelector;

    class OccupancyGrid {
    public:
        OccupancyGrid(FrameSelector *selector, PointModelDrawer* pPointModelDrawer, const string &strSettingsFile);

        virtual ~OccupancyGrid();

        void Run();

        void savePointCloud(string filename);

        void saveOccupancyGrid(string filename);

        void RequestFinish();

    protected:
        void publishTSDF(cv::Mat &imRGB, cv::Mat &imD, rmd::SE3<float> &T_world_ref);

        void addPointCloud(pcl::PointCloud<pcl::PointXYZRGB>* cloud);

        void deletePointCloud(pcl::PointCloud<pcl::PointXYZRGB>* cloud);

    private:
        FrameSelector *mpFrameSelector;
        PointModelDrawer *mpPointModelDrawer;
        Octree mOCtree;
        octomap::ColorOcTree *mpOctomap;
        OpenARK::GpuTsdfGenerator *mpGpuTsdfGenerator;

        std::mutex mFinishMutex;
        bool mFinishRequest;

        std::mutex mQueueMutex;
        std::mutex mDeleteMutex;

        std::queue<pcl::PointCloud<pcl::PointXYZRGB> *> clouds;
        std::queue<pcl::PointCloud<pcl::PointXYZRGB> *> deleteClouds;

        // camera params
        float fx_, fy_, cx_, cy_;
        float maxdepth_;
        int width_, height_;
        float depthfactor_;
    };
}


#endif //ORB_SLAM2_OCCUPANCYGRID_H
