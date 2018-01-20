//
// Created by lucas on 11/30/17.
//

#ifndef ORB_SLAM2_BRIDGERSR200_H
#define ORB_SLAM2_BRIDGERSR200_H

#include <librealsense/rs.hpp>
#include <mutex>
#include <opencv2/opencv.hpp>

class BridgeRSR200{
    rs::context mCtx;
    rs::device * mpDev;
    rs::intrinsics mDepth_intrin;
    rs::extrinsics mDepth_to_color;
    rs::intrinsics mColor_intrin;
    float mDepthScale;
public:
    BridgeRSR200();
    int Start();
    void GrabRGBDPair(cv::Mat& imRGB, cv::Mat& imD);
    void Stop();
};

#endif //ORB_SLAM2_BRIDGERSR200_H
