//
// Created by shida on 06/12/16.
//

#ifndef __MODELDRAWER_H
#define __MODELDRAWER_H

#include"Map.h"
#include"MapPoint.h"
#include"KeyFrame.h"
#include<pangolin/pangolin.h>

#include<mutex>

#include "Frame.h"
#include "Tracking.h"
#include <map>
#include <list>

namespace ORB_SLAM2
{

    class KeyFrame;

    class ModelDrawer
    {
    public:
        ModelDrawer();

        void DrawModel();

        void AddTexture(KeyFrame* pKF);

        void AddFrame(const long unsigned int &frameID, const cv::Mat &im);


    private:

        //queue for the keyframes used to texture the model, keyframe mnFrameId
        std::list<long unsigned int> mlTextureQueue;
        size_t mnMaxTextureQueueSize;
        std::mutex mMutexTexture;

        //queue for the frames recieved, pair<mnID,image>
        std::map<long unsigned int, cv::Mat> mmFrameQueue;
        size_t mnMaxFrameQueueSize;
        std::mutex mMutexFrame;

    };

} //namespace ORB_SLAM

#endif //__MODELDRAWER_H
