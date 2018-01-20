//
// Created by lucas on 11/30/17.
//

//
// Created by yang on 16-12-2.
//

// TODO refactor the code with self define Point type

#include "Octree.h"
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

using namespace pcl;
using namespace std;

static const float eps = 1e-5;
static int age_limit = 3;

static int depthNumber=0;

static pcl::RadiusOutlierRemoval<pcl::PointXYZRGBA> outrem;
static std::vector<int> pointIdxVec(20);

Octree::Octree() : octree(0.01f) {
    mPoints = new PointCloud<PointXYZRGBA>();
}

static float pointSquaredDist (PointXYZRGBA & point_a, PointXYZRGBA & point_b)
{
    return (point_a.getVector3fMap () - point_b.getVector3fMap ()).squaredNorm();
}

int Octree::findPoint(PointXYZRGB& p,PointCloud<PointXYZRGBA>::Ptr points){
    int result_idx;
    if(points->size() == 0){
        return -1;
    }
    PointXYZRGBA point;
    copyPoint(p,point);

    double smallest_squared_dist = std::numeric_limits<double>::max ();
    pointIdxVec.resize(0);

    float dist;
    if (octree.voxelSearch (point, pointIdxVec))
    {
        for (size_t i = 0; i < pointIdxVec.size (); ++i){
            dist = pointSquaredDist(point,points->points[pointIdxVec[i]]);
            if(dist < smallest_squared_dist){
                smallest_squared_dist = dist;
                result_idx = pointIdxVec[i];
            }
        }
    }

//    octree.approxNearestSearch(point,result_idx,result_diff);

    if(smallest_squared_dist < eps){
        return result_idx;
    }

    return -1;
}

void Octree::radiusFilter(PointCloud<PointXYZRGBA>::Ptr points){
    PointCloud<PointXYZRGBA>::Ptr cloud_filtered(new PointCloud<PointXYZRGBA>);
    outrem.setInputCloud(points);
    outrem.setRadiusSearch(0.01);
    outrem.setMinNeighborsInRadius (50);
    // apply filter
    outrem.filter (*cloud_filtered);

    copyPointCloud(*cloud_filtered,*points);
}

void Octree::removeOldAge(PointCloud<PointXYZRGBA>::Ptr cloud){
    // creata a new point cloud
    PointCloud<PointXYZRGBA>::Ptr newpoints(new PointCloud<PointXYZRGBA>(cloud->size(), 1));

    int count = 0;
//    int maxAge = 0;
//    for(int i=0;i<points->size();i++){
//        if(maxAge < age[i]){
//            maxAge = age[i];
//        }
//    }

    for(int i=0;i<cloud->size();i++){
        if(cloud->points[i].a < age_limit){
            newpoints->points[count++] = cloud->points[i];
        }
    }

    printf("cloud size %d remove point size %d\n",cloud->size(),cloud->size() - count);

    newpoints->resize(count);

    copyPointCloud(*newpoints,*cloud);
    age_limit += 1;

    if(age_limit > 10){
        age_limit = 10;
    }
    // rebuilt the octree
//    octree->deleteTree();
//    octree->setInputCloud(points);
//    octree->addPointsFromInputCloud();
}

static std::vector<bool> visited;
std::vector<pcl::PointXYZRGB> addVector;

void Octree::addPoints(PointCloud<PointXYZRGB>::Ptr cloud,int count){
    depthNumber++;
    printf("now depth number %d\n",depthNumber);
//    *points += *cloud;
    PointCloud<PointXYZRGBA>::Ptr points(new PointCloud<PointXYZRGBA>);

    {
        unique_lock<mutex> lock(pointMutex);
        copyPointCloud(*mPoints,*points);
    }

    // create a octree
    octree.deleteTree();
    octree.setInputCloud(points);
    octree.addPointsFromInputCloud();

    int size = points->size();
    visited.resize(points->size());
    addVector.resize(0);

    for(int i=0;i<count;i++){
        int idx = findPoint(cloud->points[i],points);
        // have visit the point
        if(idx != -1){
            points->points[idx].a = 1;
            visited[idx] = true;

        }else{
            addVector.push_back(cloud->points[i]);
        }
    }

    printf("Add points %d\n",addVector.size());

    // now add the age
    for(int i=0;i<points->size();i++){
        if(!visited[i]){
            points->points[i].a++;
        }
    }

    // now add the point
    size = points->size();
    points->points.resize(points->size() + addVector.size());
    for(int i=0;i<addVector.size();i++){
        copyPoint(addVector[i],points->points[size + i]);
        points->points[size + i].a = 1;
    }

    printf("now point size %d\n",points->size());

    {
        unique_lock<mutex> lock(pointMutex);
        copyPointCloud(*points,*mPoints);
    }
}

void Octree::deletePoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int count) {
    PointCloud<PointXYZRGBA>::Ptr points(new PointCloud<PointXYZRGBA>);

    {
        unique_lock<mutex> lock(pointMutex);
        copyPointCloud(*mPoints,*points);
    }

    for(int i=0;i<count;i++){
        PointXYZRGBA point;
        copyPoint(cloud->points[i],point);
        octree.deleteVoxelAtPoint(point);
    }

    {
        unique_lock<mutex> lock(pointMutex);
        copyPointCloud(*points,*mPoints);
    }
}

PointCloud<PointXYZRGB>::Ptr Octree::getPoints(){
    PointCloud<PointXYZRGB>::Ptr ans(new PointCloud<PointXYZRGB>);

    {
        unique_lock<mutex> lock(pointMutex);
        copyPointCloud(*mPoints, *ans);
    }

    printf("paint size %d\n",ans->size());
    return ans;
}