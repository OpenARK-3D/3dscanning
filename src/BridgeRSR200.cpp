//
// Created by lucas on 11/30/17.
//
#include "BridgeRSR200.h"

BridgeRSR200::BridgeRSR200() {
    // Turn on logging. We can separately enable logging to console or to file, and use different severity filters for each.
    rs::log_to_console(rs::log_severity::warn);
    //rs::log_to_file(rs::log_severity::debug, "librealsense.log");
}

int BridgeRSR200::Start() {
    // Create a context object. This object owns the handles to all connected realsense devices.
    printf("There are %d connected RealSense devices.\n", mCtx.get_device_count());
    if(mCtx.get_device_count() == 0) return EXIT_FAILURE;

    // This tutorial will access only a single device, but it is trivial to extend to multiple devices
    mpDev = mCtx.get_device(0);
    printf("\nUsing device 0, an %s\n", mpDev->get_name());
    printf("    Serial number: %s\n", mpDev->get_serial());
    printf("    Firmware version: %s\n", mpDev->get_firmware_version());

    // Configure depth and color to run with the device's preferred settings
    mpDev->enable_stream(rs::stream::color, rs::preset::best_quality);
    mpDev->enable_stream(rs::stream::depth, rs::preset::largest_image);
    mpDev->start();

    mDepth_intrin = mpDev->get_stream_intrinsics(rs::stream::depth);
    mDepth_to_color = mpDev->get_extrinsics(rs::stream::depth, rs::stream::color);
    mColor_intrin = mpDev->get_stream_intrinsics(rs::stream::color);
    mDepthScale = mpDev->get_depth_scale();
    return 0;
}

void BridgeRSR200::GrabRGBDPair(cv::Mat &imRGB, cv::Mat &imD) {
    mpDev->wait_for_frames();

    uint8_t * color_image = (uint8_t *)mpDev->get_frame_data(rs::stream::rectified_color);
    uint16_t * depth_image = (uint16_t *)mpDev->get_frame_data(rs::stream::depth_aligned_to_rectified_color);

    imRGB = cv::Mat(mColor_intrin.height, mColor_intrin.width, CV_8UC3);
    imD =  cv::Mat(mDepth_intrin.height, mDepth_intrin.width, CV_16SC1);

    std::memcpy(imRGB.datastart, color_image, mColor_intrin.height*mColor_intrin.width*3*sizeof(unsigned char));
    std::memcpy(imD.datastart, depth_image, mDepth_intrin.height*mDepth_intrin.width*sizeof(short));
//    cv::imshow("Depth", imD);
//    cv::imshow("RGB", imRGB);
}

void BridgeRSR200::Stop() {
    mpDev->stop();
}