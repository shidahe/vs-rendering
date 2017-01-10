//
// Created by theia on 1/10/17.
//

#ifndef ORB_SLAM2_MODELFRAME_H
#define ORB_SLAM2_MODELFRAME_H

#include "KeyFrame.h"
#include "MapPoint.h"
#include "Frame.h"

namespace ORB_SLAM2 {

    class TextureFrame {
    public:
        TextureFrame(KeyFrame *pKF) {
            mFrameID = pKF->mnFrameId;
            // GetPose instead GetPoseInverse, seems camera position need to be inversed
            mRcw = pKF->GetPose().rowRange(0, 3).colRange(0, 3);
            mtcw = pKF->GetPose().rowRange(0, 3).col(3);
            mfx = pKF->fx;
            mfy = pKF->fy;
            mcx = pKF->cx;
            mcy = pKF->cy;
        }

        TextureFrame(Frame *pF) {
            mFrameID = pF->mnId;
            // GetPose instead GetPoseInverse, seems camera position need to be inversed
            mRcw = pF->mTcw.rowRange(0, 3).colRange(0, 3);
            mtcw = pF->mTcw.rowRange(0, 3).col(3);
            mfx = pF->fx;
            mfy = pF->fy;
            mcx = pF->cx;
            mcy = pF->cy;
        }

        vector<float> GetTexCoordinate(float x, float y, float z, cv::Size s) {
            const cv::Mat P = (cv::Mat_<float>(3, 1) << x, y, z);
            // 3D in camera coordinates
            const cv::Mat Pc = mRcw * P + mtcw;
            const float &PcX = Pc.at<float>(0);
            const float &PcY = Pc.at<float>(1);
            const float &PcZ = Pc.at<float>(2);

            // Project in image and check it is not outside
            const float invz = 1.0f / PcZ;
            const float u = mfx * PcX * invz + mcx;
            const float v = mfy * PcY * invz + mcy;

            float uTex = u / s.width;
            float vTex = v / s.height;
            std::vector<float> uv;
            uv.push_back(uTex);
            uv.push_back(vTex);
            return uv;
        }

        long unsigned int mFrameID;
        cv::Mat mRcw;
        cv::Mat mtcw;
        float mfx, mfy, mcx, mcy;
    };

    class ModelFrame {
    public:
        ModelFrame(Frame *pF, vector<MapPoint *> vMPs) {
            mFrameID = pF->mnId;
            mTcw = pF->mTcw.clone();
            mvMPs = vMPs;
        }

        long unsigned int mFrameID;
        cv::Mat mTcw;
        vector<MapPoint *> mvMPs;
    };

}

#endif //ORB_SLAM2_MODELFRAME_H
