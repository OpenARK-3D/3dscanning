//
// Created by yang on 16-12-3.
//

#ifndef POINTMODELDRAWER_H
#define POINTMODELDRAWER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <mutex>

namespace ORB_SLAM2 {
    class PointModelDrawer {
    public:
        PointModelDrawer();

        void update(pcl::PointCloud<pcl::PointXYZRGB>::Ptr points);

        void drawModel();

    protected:
        std::mutex mCloudMutex;
        pcl::PointCloud<pcl::PointXYZRGB> mCloud;
    };
}


#endif //POINTMODELDRAWER_H
