//
// Created by lucas on 11/29/17.
//

#include "FrameSelector.h"

namespace ORB_SLAM2 {
    FrameSelector::FrameSelector(Map* pMap) : mpMap(pMap) {
        mState = Tracking::SYSTEM_NOT_READY;
        mbBadKeyFrame = false;
    }
    void convert(Frame &frame, FrameData *frameData) {
        frame.mTcw.copyTo(frameData->mTcw);
        frame.mImColor.copyTo(frameData->imColor);
        frame.mImDepth.copyTo(frameData->imDepth);
        frameData->frameId = frame.mnId;
    }

    void convert(KeyFrame *keyFrame, FrameData *frameData) {
        frameData->mTcw = keyFrame->GetPose();
        frameData->imColor = keyFrame->mImColor;
        frameData->imDepth = keyFrame->mImDepth;
        frameData->frameId = keyFrame->mnFrameId;
    }

    FrameData* FrameSelector::getKeyFrame(int frameId){
        std::unique_lock<std::mutex> lock(mFrameMutex);
        FrameData* frameData = NULL;
        if(mState == Tracking::OK && mKeyFrameData != NULL && !mbBadKeyFrame && mKeyFrameData->frameId != frameId){
            frameData = new FrameData();

            mKeyFrameData->mTcw.copyTo(frameData->mTcw);
            mKeyFrameData->imColor.copyTo(frameData->imColor);
            mKeyFrameData->imDepth.copyTo(frameData->imDepth);

            frameData->frameId = mKeyFrameData->frameId;

            delete mKeyFrameData;
            mKeyFrameData = NULL;
        }

        return frameData;
    }

    FrameData* FrameSelector::getFrame(int frameId){
        std::unique_lock<std::mutex> lock(mFrameMutex);
        FrameData* frameData = NULL;
        if(mState == Tracking::OK && mCurrentFrameData != NULL && mCurrentFrameData->frameId!=frameId){
            frameData = new FrameData();

            mCurrentFrameData->mTcw.copyTo(frameData->mTcw);
            mCurrentFrameData->imColor.copyTo(frameData->imColor);
            mCurrentFrameData->imDepth.copyTo(frameData->imDepth);

            frameData->frameId = mCurrentFrameData->frameId;

            mCurrentFrameData = NULL;
        }

        return frameData;
    }

    void FrameSelector::Update(Tracking *pTracker) {
        if(pTracker->mLastProcessedState != Tracking::OK){
            return;
        }

        Frame frame = Frame(pTracker->mCurrentFrame);

        KeyFrame *keyFrame = frame.mpReferenceKF;

        bool bad = false;

        if(!keyFrame->mbBA){
            bad = true;
            std::cout << "not local mapping optimatize, drop" << std::endl;
        }

        if(keyFrame->isBad()){
            std::cout << "is bad " << std::endl;
            bad = true;
        }

        FrameData *currentFrameData = new FrameData();
        convert(frame,currentFrameData);

        FrameData* keyFrameData = new FrameData();
        convert(keyFrame,keyFrameData);

        // lock
        {
            std::unique_lock<std::mutex> lock(mFrameMutex);
            mState = static_cast<int>(pTracker->mLastProcessedState);

            mCurrentFrameData = currentFrameData;
            mKeyFrameData = keyFrameData;
            mbBadKeyFrame = bad;
        }
    }
}
