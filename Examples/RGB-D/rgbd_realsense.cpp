//
// Created by yang on 16-12-9.
//

/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <librealsense/rs.hpp>
#include<opencv2/core/core.hpp>

#include<System.h>
#include<BridgeRSR200.h>
#include<MeshGenerator.h>
#include<Segmentation.h>

using namespace cv;
using namespace std;
typedef cv::Vec<uchar, 3> Vec3b;

int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cerr << endl << "Usage: ./rgbd_realsense path_to_vocabulary path_to_settings" << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD, true);

    // Start the R200 rgbd camera
    BridgeRSR200 bridgeRSR200;
    bridgeRSR200.Start();

    // Main loop
    int tframe = 1;
    while(!SLAM.IsStopRequested())
    {
        cv::Mat imRGB, imD;

        bridgeRSR200.GrabRGBDPair(imRGB, imD);

        // Pass the image to the SLAM system
        SLAM.TrackRGBD(imRGB, imD, tframe);

        // check map changed
        if(SLAM.MapChanged()){
            std::cout << "map changed" << std::endl;
        }

        tframe++;
    }
    SLAM.SavePointCloud("tmp.pcd");
//    SLAM.SaveOccupancyGrid("map.ot");
    SLAM.Shutdown();

    Segmentaion segmentation;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    segmentation.readPcd("tmp.pcd", cloud);
    segmentation.segment(cloud);
    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> clusters = segmentation.getCluster();

    for(int i=0;i<clusters.size();++i) {
        cout << "Start Post-processing" << endl;
        cout << "Mesh Generation" << endl;
        string filepcd;
        stringstream ss;
        ss<<"pc_"<<i<<".pcd";
        ss>>filepcd;
        pcl::io::savePCDFileBinary(filepcd, *clusters[i]);
    }
}