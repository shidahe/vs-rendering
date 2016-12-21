//
// Created by shida on 06/12/16.
//

#include "Modeler/ModelDrawer.h"
#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>

namespace ORB_SLAM2
{


    ModelDrawer::ModelDrawer():mnMaxTextureQueueSize(4), mnMaxFrameQueueSize(100)
    {
    }

    void ModelDrawer::DrawModel()
    {
//        glEnable(GL_TEXTURE_2D);
//        static unsigned int frameTex[4] = {0, 0, 0, 0};
//        if(!frameTex[0])
//            glGenTextures(4, frameTex);
//
//        for(int i = 0; i < 4; i++){
//            glBindTexture(GL_TEXTURE_2D, frameTex[i]);
//            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
//            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
//            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
//            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
//            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
//                         ir.x, ir.y, 0,
//                         GL_RGB,
//                         GL_UNSIGNED_BYTE,
//                         imFrame[i].data());
//        }

    }

    void ModelDrawer::AddTexture(KeyFrame* pKF)
    {
        unique_lock<mutex> lock(mMutexTexture);
        const long unsigned int frameID = pKF->mnFrameId;
        if (mlTextureQueue.size() >= mnMaxTextureQueueSize) {
            mlTextureQueue.pop_front();
        }
        mlTextureQueue.push_back(frameID);

    }


    void ModelDrawer::AddFrame(const long unsigned int &frameID, const cv::Mat &im)
    {
        unique_lock<mutex> lock(mMutexFrame);
        if (mmFrameQueue.size() >= mnMaxFrameQueueSize) {
            mmFrameQueue.erase(mmFrameQueue.begin());
        }
        if (mmFrameQueue.count(frameID) > 0){
            std::cerr << "ERROR: trying to add an existing frame" << std::endl;
            return;
        }
        mmFrameQueue.insert(make_pair(frameID,im.clone()));
    }


} //namespace ORB_SLAM
