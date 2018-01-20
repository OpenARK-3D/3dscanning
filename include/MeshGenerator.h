#pragma once

#include <iostream>
#include <fstream>
#include <ctime>
#include <string>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

using namespace std;
enum fileFormat{ PCD, PLY };

class MeshGenerator {
private:
	void generateMesh(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,
					  double smoothRadius,
					  string outputFileName);

public:
	MeshGenerator() {}
	~MeshGenerator() {}
	void run(double smoothRadius,
			 fileFormat format,
			 string inputFileName,
			 string outputFileName);
	void run(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcloud,
			 double smoothRadius,
			 string outputFileName);
};