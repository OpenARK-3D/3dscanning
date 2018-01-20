#pragma once

#include <cstdlib>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>


class Viz {
public:
	std::string pcdpath;

	Viz() {
		init();
		text_id = 0;
	};

	pcl::visualization::PCLVisualizer* getPCLVisualizer() {
		return viewer;
	};

	/** Set the cloud including all clusters' points. For click raycasting for cluster selection */
	void setClusters(std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>>> * in) {
		clusters = in;
	};

private:
	/** PCL Visualizer pointer */
	pcl::visualization::PCLVisualizer* viewer;

	/** Current ID. For usage of debugging. Objects (text, clouds,...) placed into visualizer
	 * has a unique ID. Use incrementing ints here. */
	unsigned int text_id;

	/** Vector of cluster point cloud ptrs. */
	std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>>>* clusters;

	/** Current selected cluster index inside of vector `clusters`. -1 if none selected. */
	int selected_cluster_index;

	/** Initialize Viz, creating a PCL visualizer among other things. */
	void init()
	{
		viewer = new pcl::visualization::PCLVisualizer("OpenARK Cluster Selection");
			
		viewer->setBackgroundColor(0, 0, 0);
		viewer->addCoordinateSystem(1.0);
		viewer->initCameraParameters();
		viewer->setCameraPosition(0, 0, -3, 0, 1, 0);
		int x_res = 800;
		int y_res = 600;
		viewer->setSize(x_res, y_res);

		viewer->registerMouseCallback(&Viz::mouseEventOccurred, *this);

		selected_cluster_index = -1;
	};

	void mouseEventOccurred(
		const pcl::visualization::MouseEvent &event,
		void* viewer_void
	) {
		if (event.getButton() == pcl::visualization::MouseEvent::RightButton &&
			event.getType() == pcl::visualization::MouseEvent::MouseButtonRelease) {

			// Reset selection
			selected_cluster_index = -1;

			// Get camera from PCL Visualizer. This is done at every mouse event because the
			// camera and/or viewport might be changed/updated.
			std::vector<pcl::visualization::Camera> cameras;
			viewer->getCameras(cameras);
			pcl::visualization::Camera* cam = &cameras[0];

			double x_res = cam->window_size[0];
			double y_res = cam->window_size[1];
			double aspect_ratio = x_res / y_res;
			double yfov = cam->fovy;
			double xfov = 2.0 * std::atan(aspect_ratio * std::tan(yfov / 2.0));

			// Ray cast left from mouse click.
		    // This translates the x,y position on image space into a ray, whose
			// origin is the pinhole camera's position and direction is from the origin
			// through the image space pixel x,y.
			// More about this approach can be found in any ray tracing literature.
			// Note: I use Eigen library for vector maths because it already comes with PCL.
			Eigen::Vector3f up(cam->view[0], cam->view[1], cam->view[2]);
			Eigen::Vector3f focal(cam->focal[0], cam->focal[1], cam->focal[2]);
			// Vectors of the camera matrix: a b c d. Brings cam space to world space.
			// a,b,c represents the x,y,z axes of the camera space. d is camera origin.
			Eigen::Vector3f d(cam->pos[0], cam->pos[1], cam->pos[2]);
			Eigen::Vector3f c = (d - focal);
			c.normalize();
			Eigen::Vector3f a = up.cross(c);
			a.normalize();
			Eigen::Vector3f b = c.cross(a);
			
			// Account for FOV of camera.
			double scaleX = 2 * std::tan(xfov / 2);
			double scaleY = 2 * std::tan(yfov / 2);

			// Make selected image pixel in range of [-0.5,0.5]. Added 0.5 to get center of the
			// pixel.
			double rangeX = (event.getX() + 0.5) / x_res - 0.5;
			double rangeY = (event.getY() + 0.5) / y_res - 0.5;

			// Create ray!
			Eigen::Vector3f dir = rangeX * scaleX * a + rangeY * scaleY * b - c;
			dir.normalize();
			Eigen::Vector3f pos = d;

			// Draw debugging line 
			viewer->removeShape("line");
			Eigen::Vector3f far_pt = pos + 25.0 * dir;
			viewer->addLine<pcl::PointXYZ, pcl::PointXYZ>(
				pcl::PointXYZ(cam->pos[0], cam->pos[1], cam->pos[2]),
				//pcl::PointXYZ(0.0,0.0,1.0),
				pcl::PointXYZ(far_pt(0), far_pt(1), far_pt(2)),
				1.0, 0.0, 0.0,
				"line"
				);
			viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1.0, "line");

			// Find point (and therfore cluster) that is closest to the camera.
			// Because the ray probably will not directly intersect any points, use
			// epsilon radius around ray; any points within epsilon distance from ray will
			// be considered.
			Eigen::ParametrizedLine<float, 3> ray(pos, dir);
			double epsilon = 0.02; // Hard-coded to be voxelized distance.
			double closest_dist_cam = std::numeric_limits<double>::max();
			for (int i = 0; i < clusters->size(); i++) {
				auto clust = (*clusters)[i];
				for (int j = 0; j < clust->points.size(); j++) {
					auto pcl_pt = clust->points[j];
					Eigen::Vector3f pt(pcl_pt.x, pcl_pt.y, pcl_pt.z);
					double dist_ray = ray.distance(pt);
					if (dist_ray < epsilon) {
						double dist_cam = (pos - pt).squaredNorm();
						if (dist_cam < closest_dist_cam) {
							selected_cluster_index = i;
							closest_dist_cam = dist_cam;
						}
					}
				}
			}
			std::cout << "Selected cluster: " << selected_cluster_index << std::endl;

			// TODO Export selected cluster. Can be file write, etc.
			if (selected_cluster_index != -1) {
				auto clust = (*clusters)[selected_cluster_index];
				std::stringstream ss;
				ss << "ransac" << selected_cluster_index;
				viewer->removePointCloud(ss.str());
				pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZRGBA> cloud_color(clust);
				viewer->addPointCloud<pcl::PointXYZRGBA>(clust, cloud_color, ss.str(), 0);
				viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, ss.str());

				//Save selected cluster
				//std::ostringstream filename;
				//boost::filesystem::path p(pcdpath);
				//filename << "pcd_out\\" << p.stem().string() << "_" << selected_cluster_index << "_" << clust->points.size() << ".pcd";
				//Writer::savePCD(*clust, filename.str());

				// Reset
				selected_cluster_index = -1;
			}
		}
	};
};