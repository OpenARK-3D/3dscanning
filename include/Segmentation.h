//
// Created by lucas on 12/2/17.
//

#ifndef ORB_SLAM2_SEGMENTATION_H
#define ORB_SLAM2_SEGMENTATION_H

#include <iostream>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "Viz.h"

class Segmentaion
{
    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> mvecpCluster;
public:
    void readPcd(std::string filepath, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud) {
        if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(filepath, *cloud) == -1) {
            PCL_ERROR("File loading error");
        }
    };

    /**
 * Removes the plane from the objects from a point cloud.
 * \in cloud
 * \out cloud_plane The cloud representing plane.
 * \out cloud_objs The cloud representing objects (the rest of points).
 */
    void removePlane(
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_plane,
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_objs
    ) {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.07);
        seg.setMaxIterations(100);

        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            PCL_ERROR("Could not estimate a planar model for the given dataset.");
            return;
        }

        pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
        // Extract plane inliers.
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_plane);
        // Extract outliers as objects cloud (no plane).
        extract.setNegative(true);
        extract.filter(*cloud_objs);
    };

    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> computeClusters(
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud
    ) {
        pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
        kdtree->setInputCloud(cloud);

        // Cluster point clouds using Euclidean.
        std::vector<pcl::PointIndices> cluster_indices; // Vector of multiple indices collections (clusters)
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> clustering;
        clustering.setClusterTolerance(0.04); // m
        clustering.setMinClusterSize(200);
        clustering.setMaxClusterSize(1000000);
        clustering.setSearchMethod(kdtree);
        clustering.setInputCloud(cloud);
        clustering.extract(cluster_indices);

        std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> clusters;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGBA>);
            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
                cloud_cluster->points.push_back(cloud->points[*pit]);
            }
            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
            clusters.push_back(cloud_cluster);
        }
        return clusters;
    };

    void vizSelect(std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> clusters) {
        Viz* viz = new Viz();
        viz->setClusters(&clusters);
        pcl::visualization::PCLVisualizer viewer = *viz->getPCLVisualizer();

        int id(0);
        viewer.createViewPort(0.0, 0.0, 1.0, 1.0, id);
        viewer.setBackgroundColor(0.0, 0.2, 0.2, id);
        for (int i = 0; i < (int)clusters.size(); i++) {
            std::stringstream ss;
            ss << "cluster" << i;
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> cloud_color(clusters[i]);
            viewer.addPointCloud<pcl::PointXYZRGBA>(clusters[i], cloud_color, ss.str(), id);
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, ss.str());
        }

        while (!viewer.wasStopped()) {
            viewer.spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }
    };

    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> getCluster()
    {
        return mvecpCluster;
    }

    void segment(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud) {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_objs(new pcl::PointCloud<pcl::PointXYZRGBA>);

        // Down sample the dataset.
        pcl::VoxelGrid<pcl::PointXYZRGBA> vox;
        vox.setInputCloud(cloud);
        vox.setLeafSize(0.005f, 0.005f, 0.005f); // m
        vox.filter(*cloud_filtered);

        // Statistical outlier removal
        // TODO: should be handled in a sort of preprocessing stage
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
        sor.setInputCloud(cloud_filtered);
        sor.setMeanK(50);
        sor.setStddevMulThresh(1.0);
        sor.filter(*cloud_filtered);

        removePlane(cloud_filtered, cloud_plane, cloud_objs);
        mvecpCluster = computeClusters(cloud_objs);

//        vizSelect(clusters);
    };
};

#endif //ORB_SLAM2_SEGMENTATION_H
