//
// Created by theia on 1/10/17.
//

#ifndef ORB_SLAM2_MODELFRAME_H
#define ORB_SLAM2_MODELFRAME_H

#include "KeyFrame.h"
#include "MapPoint.h"
#include "Frame.h"

namespace ORB_SLAM2 {

    class KeyFrame;
    class MapPoint;
    class Frame;

    class TextureFrame {
    public:
        TextureFrame(KeyFrame *pKF);

        TextureFrame(Frame *pF);

        vector<float> GetTexCoordinate(float x, float y, float z, cv::Size s);

        vector<float> GetTexCoordinate(float x, float y, float z);

        long unsigned int mFrameID;
        cv::Mat mRcw;
        cv::Mat mtcw;

        float mfx, mfy, mcx, mcy;
        float mnMinX;
        float mnMaxX;
        float mnMinY;
        float mnMaxY;
    };


    class ModelFrame {
    public:
        ModelFrame(Frame *pF, vector<MapPoint *> vMPs);

        long unsigned int mFrameID;
        cv::Mat mTcw;
        vector<MapPoint *> mvMPs;
    };

}

#endif //ORB_SLAM2_MODELFRAME_H
