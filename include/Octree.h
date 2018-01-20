//
// Created by lucas on 11/30/17.
//

#ifndef ORB_SLAM2_OCTREE_H
#define ORB_SLAM2_OCTREE_H
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree.h>
#include <mutex>

class Octree {
public:
    Octree();
    void addPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,int count);
    void deletePoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,int count);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPoints();
private:
    void removeOldAge(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);
    void radiusFilter(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);
    int findPoint(pcl::PointXYZRGB& point,
                  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBA> octree;
    pcl::PointCloud<pcl::PointXYZRGBA>* mPoints;

    std::mutex pointMutex;
};

#endif //ORB_SLAM2_OCTREE_H
