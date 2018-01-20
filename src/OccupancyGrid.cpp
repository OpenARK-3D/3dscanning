//
// Created by lucas on 11/29/17.
//

#include "OccupancyGrid.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/median_filter.h>

typedef pcl::PointXYZRGB PointType;
static int framecount;

static rmd::SE3<float> convertPoseToRmd(cv::Mat& tcw){
    float rot[6];
    float tra[3];

    rot[0] = tcw.at<float>(0, 0);
    rot[1] = tcw.at<float>(0, 1);
    rot[2] = tcw.at<float>(0, 2);
    rot[3] = tcw.at<float>(1, 0);
    rot[4] = tcw.at<float>(1, 1);
    rot[5] = tcw.at<float>(1, 2);
    rot[6] = tcw.at<float>(2, 0);
    rot[7] = tcw.at<float>(2, 1);
    rot[8] = tcw.at<float>(2, 2);

    tra[0] = tcw.at<float>(0, 3);
    tra[1] = tcw.at<float>(1, 3);
    tra[2] = tcw.at<float>(2, 3);
    rmd::SE3<float> T_world_curr(rot, tra);

    return T_world_curr;
}

static float3 operator*(const rmd::SE3<float> &se3, const float3 &p)
{
    return se3.translate(se3.rotate(p));
}

namespace ORB_SLAM2 {
    int framedix = 0;

    OccupancyGrid::OccupancyGrid(FrameSelector *pFrameSelector, PointModelDrawer* pPointModelDrawer, const string &strSettingsFile):mpFrameSelector(pFrameSelector), mpPointModelDrawer(pPointModelDrawer) {
        cv::FileStorage fSettings(strSettingsFile, cv::FileStorage::READ);

        fx_ = fSettings["Camera.fx"];
        fy_ = fSettings["Camera.fy"];
        cx_ = fSettings["Camera.cx"];
        cy_ = fSettings["Camera.cy"];
        width_ = fSettings["Camera.width"];
        height_ = fSettings["Camera.height"];
        depthfactor_ = fSettings["DepthMapFactor"];
        maxdepth_ = fSettings["MaxDepth"];

        float v_g_o_x = fSettings["Voxel.Origin.x"];
        float v_g_o_y = fSettings["Voxel.Origin.y"];
        float v_g_o_z = fSettings["Voxel.Origin.z"];

        float v_size = fSettings["Voxel.Size"];

        float v_trunc_margin = fSettings["Voxel.TruncMargin"];

        int v_g_d_x = fSettings["Voxel.Dim.x"];
        int v_g_d_y = fSettings["Voxel.Dim.y"];
        int v_g_d_z = fSettings["Voxel.Dim.z"];

        mpOctomap = new octomap::ColorOcTree(fSettings["OctomapVoxel"]);
        mpGpuTsdfGenerator = new OpenARK::GpuTsdfGenerator(width_,height_,fx_,fy_,cx_,cy_,
                                                           v_g_o_x,v_g_o_y,v_g_o_z,v_size,
                                                           v_trunc_margin,v_g_d_x,v_g_d_y,v_g_d_z);
        mFinishRequest = false;
    }

    void OccupancyGrid::Run() {
        bool requestFinish = false;
        int prevFrameId = -1;
        while (!requestFinish) {
            FrameData *data;
            data = mpFrameSelector->getKeyFrame(prevFrameId);
            if (data == NULL) {
                continue;
            }

            prevFrameId = data->frameId;
            cv::Mat Twc = data->mTcw.inv();
            rmd::SE3<float> T_world_curr = convertPoseToRmd(Twc);

            publishTSDF(data->imColor, data->imDepth, T_world_curr);

            mpPointModelDrawer->update(mOCtree.getPoints());

            {
                std::unique_lock<std::mutex> lock(mFinishMutex);
                requestFinish = mFinishRequest;
            }
            delete data;
        }

    }

    void OccupancyGrid::savePointCloud(string filename) {

        pcl::io::savePCDFileBinary(filename, *mOCtree.getPoints());
//        mpGpuTsdfGenerator->SaveTSDF("tsdf.bin");
        mpGpuTsdfGenerator->SavePLY("tsdf.ply");
    }

    void OccupancyGrid::saveOccupancyGrid(string filename){
        mpOctomap->updateInnerOccupancy();
        mpOctomap->write(filename);
    }

    void OccupancyGrid::RequestFinish() {
        std::unique_lock<std::mutex> lock(mFinishMutex);
        mFinishRequest = true;
    }

    void OccupancyGrid::addPointCloud(pcl::PointCloud<pcl::PointXYZRGB> *cloud) {
        unique_lock<mutex> lock(mQueueMutex);
        clouds.push(cloud);
    }

    void OccupancyGrid::deletePointCloud(pcl::PointCloud<pcl::PointXYZRGB> *cloud) {
        unique_lock<mutex> lock(mDeleteMutex);
        deleteClouds.push(cloud);
    }

    void OccupancyGrid::publishTSDF(cv::Mat &imRGB, cv::Mat &imD, rmd::SE3<float> &T_world_ref) {

        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
        for (int y = 0; y < imD.rows; ++y) {
            for (int x = 0; x < imD.cols; ++x) {
                const float3 f = make_float3(x, y, imD.at<float>(y, x));
                if (imD.at<float>(y, x) > 0.0f && imD.at<float>(y,x) <maxdepth_) {
                    PointType p;
                    p.x = f.x;
                    p.y = f.y;
                    p.z = f.z;
                    const cv::Vec3b color = imRGB.at<cv::Vec3b>(y, x);
                    p.r = color[0];
                    p.g = color[1];
                    p.b = color[2];
                    cloud->push_back(p);
                } else {
                    PointType p;
                    p.x = NAN;
                    p.y = NAN;
                    p.z = NAN;
                    p.r = 0;
                    p.g = 0;
                    p.b = 0;
                    cloud->push_back(p);
                }
            }
        }

        cloud->width = 640;
        cloud->height = 480;

        if (!cloud->empty()) {
            pcl::PointCloud<PointType>::Ptr cloud_bilateral(new pcl::PointCloud<PointType>);

            pcl::FastBilateralFilter<PointType> filter;
            filter.setInputCloud(cloud);
            filter.setSigmaS(10);
            filter.setSigmaR(0.05);
            filter.applyFilter(*cloud_bilateral);

            pcl::PointCloud<PointType>::Ptr cloud_sor(new pcl::PointCloud<PointType>);
            pcl::StatisticalOutlierRemoval<PointType> sor;
            sor.setInputCloud (cloud_bilateral);
            sor.setMeanK (50);
            sor.setStddevMulThresh (0.2);
            sor.filter (*cloud_sor);

            pcl::PointCloud<PointType>::Ptr cloud_for_disp(new pcl::PointCloud<PointType>);
            for(auto p: cloud_sor->points)
            {
                if(!isnan(p.x)) {
                    const float3 f = make_float3((p.x - cx_) / fx_, (p.y - cy_) / fy_, 1.0f);
                    const float3 xyz = T_world_ref * (f * p.z);
                    PointType p_disp;
                    p_disp.x = xyz.x;
                    p_disp.y = xyz.y;
                    p_disp.z = xyz.z;
                    p_disp.r = p.r;
                    p_disp.g = p.g;
                    p_disp.b = p.b;
                    cloud_for_disp->push_back(p_disp);
                }
            }

            mOCtree.addPoints(cloud_for_disp, cloud_for_disp->size());

            cv::Mat imD_filtered(height_, width_, CV_32FC1);
            memset(imD_filtered.datastart,0.0f,sizeof(float)*height_*width_);

            for(auto p:cloud_sor->points)
                if(!isnan(p.x))
                    imD_filtered.at<float>(p.y,p.x) = p.z;

            float cam2base[16];
            for(int r=0;r<3;++r)
                for(int c=0;c<4;++c)
                    cam2base[r*4+c] = T_world_ref(r,c);
            cam2base[12] = 0.0f;
            cam2base[13] = 0.0f;
            cam2base[14] = 0.0f;
            cam2base[15] = 1.0f;

            mpGpuTsdfGenerator->processFrame((float *)imD_filtered.datastart, cam2base);
            cout << "TSDF processed" <<endl;
        }
    }

    OccupancyGrid::~OccupancyGrid() {
        delete mpOctomap;
    }
}