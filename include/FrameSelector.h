//
// Created by lucas on 11/29/17.
//

#ifndef ORB_SLAM2_FRAMESELECTOR_H
#define ORB_SLAM2_FRAMESELECTOR_H

#include "Tracking.h"
#include <mutex>

namespace ORB_SLAM2 {
    class Tracking;
    struct FrameData {
        cv::Mat mTcw;
        cv::Mat imColor;
        cv::Mat imDepth;
        int frameId;
    };

    class FrameSelector {
    public:
        FrameSelector(Map* map);

        void Update(Tracking *pTracker);

        FrameData* getFrame(int frameId);
        FrameData* getKeyFrame(int frameId);
    protected:

        vector<FrameData *> mDatas;
        FrameData* mCurrentFrameData;
        FrameData* mKeyFrameData;
        Map* mpMap;

        int mState;
        bool mbBadKeyFrame;
        std::mutex mFrameMutex;
    };
}


#endif //ORB_SLAM2_FRAMESELECTOR_H