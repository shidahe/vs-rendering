
#ifndef __MODELER_H
#define __MODELER_H

#include "LoopClosing.h"
#include "LocalMapping.h"
#include "Tracking.h"
#include "KeyFrame.h"

#include <mutex>

#include "Modeler/SFMTranscriptInterface_ORBSLAM.h"
#include "Modeler/SFMTranscriptInterface_Delaunay.h"
#include "Modeler/ModelDrawer.h"

namespace ORB_SLAM2 {

    class Tracking;
    class LoopClosing;
    class LocalMapping;
    class KeyFrame;
    class ModelDrawer;

    class TextureFrame {
    public:
        TextureFrame(KeyFrame* pKF){
            frameID = pKF->mnFrameId;
            Rcw = pKF->GetPoseInverse().rowRange(0,3).colRange(0,3);
            tcw = pKF->GetPoseInverse().rowRange(0,3).col(3);
            fx = pKF->fx;
            fy = pKF->fy;
            cx = pKF->cx;
            cy = pKF->cy;
            mnMinX = pKF->mnMinX;
            mnMaxX = pKF->mnMaxX;
            mnMinY = pKF->mnMinY;
            mnMaxX = pKF->mnMaxY;
        }

        vector<float> GetTexCoordinate(float x, float y, float z){
            const cv::Mat P = (cv::Mat_<float>(3,1) << x, y, z);
            // 3D in camera coordinates
            const cv::Mat Pc = Rcw*P+tcw;
            const float &PcX = Pc.at<float>(0);
            const float &PcY= Pc.at<float>(1);
            const float &PcZ = Pc.at<float>(2);

            // Project in image and check it is not outside
            const float invz = 1.0f/PcZ;
            const float u=fx*PcX*invz+cx;
            const float v=fy*PcY*invz+cy;

            float uTex = u / (mnMaxX - mnMinX);
            float vTex = v / (mnMaxY - mnMinY);
            std::vector<float> uv;
            uv.push_back(uTex);
            uv.push_back(vTex);
            return uv;
        }

        long unsigned int frameID;
        cv::Mat Rcw;
        cv::Mat tcw;
        float fx, fy, cx, cy;
        float mnMinX, mnMaxX, mnMinY, mnMaxY;

    };

    class Modeler {
    public:
        Modeler(ModelDrawer* pModelDrawer);

        void SetLoopCloser(LoopClosing* pLoopCloser);

        void SetLocalMapper(LocalMapping* pLocalMapper);

        void SetTracker(Tracking* pTracker);

        // Main function
        void Run();

        void UpdateModelDrawer();
        bool CheckNewTranscriptEntry();
        void RunRemainder();

        // Thread Synch
        void RequestReset();
        void RequestFinish();
        bool isFinished();


        void AddTexture(KeyFrame* pKF);
        void AddFrame(const long unsigned int &frameID, const cv::Mat &im);

        // get last n keyframes for texturing
        vector<pair<cv::Mat,TextureFrame>> GetTextures(int n);

    public:
        void ResetIfRequested();
        bool mbResetRequested;
        std::mutex mMutexReset;

        bool CheckFinish();
        void SetFinish();
        bool mbFinishRequested;
        bool mbFinished;
        std::mutex mMutexFinish;

        Tracking* mpTracker;
        LocalMapping* mpLocalMapper;
        LoopClosing* mpLoopCloser;

        ModelDrawer* mpModelDrawer;

        // This avoid that two transcript entries are created simultaneously in separate threads
        std::mutex mMutexTranscript;

        std::list<KeyFrame*> mlpTranscriptKeyFrameQueue;

        void PushKeyFrame(KeyFrame*);
        void PopKeyFrameIntoTranscript();

        //CARV interface
        SFMTranscriptInterface_ORBSLAM mTranscriptInterface; // An interface to a transcript / log of the map's work.
        //CARV runner instance
        dlovi::FreespaceDelaunayAlgorithm mObjAlgorithm;
        SFMTranscriptInterface_Delaunay mAlgInterface; // An encapsulation of the interface between the transcript and the surface inferring algorithm.
        bool mbFirstKeyFrame;


        //queue for the keyframes used to texture the model, keyframe mnFrameId
        std::deque<TextureFrame> mdTextureQueue;
        size_t mnMaxTextureQueueSize;
        std::mutex mMutexTexture;

        //queue for the frames recieved, pair<mnID,image>
        std::map<long unsigned int, cv::Mat> mmFrameQueue;
        size_t mnMaxFrameQueueSize;
        std::mutex mMutexFrame;

    };
}


#endif //__MODELVIEWER_H
