
#ifndef __MODELER_H
#define __MODELER_H

#include "LoopClosing.h"
#include "LocalMapping.h"
#include "Tracking.h"
#include "KeyFrame.h"

#include <mutex>

#include "MapPoint.h"
#include "Frame.h"
#include "System.h"
#include "Modeler/SFMTranscriptInterface_ORBSLAM.h"
#include "Modeler/SFMTranscriptInterface_Delaunay.h"
#include "Modeler/ModelDrawer.h"

namespace ORB_SLAM2 {

    class Tracking;
    class LoopClosing;
    class LocalMapping;
    class KeyFrame;
    class Frame;
    class ModelDrawer;
    class MapPoint;

    class TextureFrame {
    public:
        TextureFrame(KeyFrame* pKF){
            mFrameID = pKF->mnFrameId;
            // GetPose instead GetPoseInverse, seems camera position need to be inversed
            mRcw = pKF->GetPose().rowRange(0,3).colRange(0,3);
            mtcw = pKF->GetPose().rowRange(0,3).col(3);
            mfx = pKF->fx;
            mfy = pKF->fy;
            mcx = pKF->cx;
            mcy = pKF->cy;
        }

        TextureFrame(Frame* pF){
            mFrameID = pF->mnId;
            // GetPose instead GetPoseInverse, seems camera position need to be inversed
            mRcw = pF->mTcw.rowRange(0,3).colRange(0,3);
            mtcw = pF->mTcw.rowRange(0,3).col(3);
            mfx = pF->fx;
            mfy = pF->fy;
            mcx = pF->cx;
            mcy = pF->cy;
        }

        vector<float> GetTexCoordinate(float x, float y, float z, cv::Size s){
            const cv::Mat P = (cv::Mat_<float>(3,1) << x, y, z);
            // 3D in camera coordinates
            const cv::Mat Pc = mRcw*P+mtcw;
            const float &PcX = Pc.at<float>(0);
            const float &PcY= Pc.at<float>(1);
            const float &PcZ = Pc.at<float>(2);

            // Project in image and check it is not outside
            const float invz = 1.0f/PcZ;
            const float u=mfx*PcX*invz+mcx;
            const float v=mfy*PcY*invz+mcy;

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
        ModelFrame(Frame* pF, vector<MapPoint*> vMPs){
            mframeID = pF->mnId;
            mTcw = pF->mTcw.clone();
            mvMPs = vMPs;
        }

        long unsigned int mFrameID;
        cv::Mat mTcw;
        vector<MapPoint*> mvMPs;
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
        void AddTexture(Frame* pF);
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
        std::list<ModelFrame*> mlpTranscriptFrameQueue;


        void PushKeyFrame(KeyFrame*);
        void PopKeyFrameIntoTranscript();
        void PushFrame(ModelFrame*);
        void PopFrameIntoTranscript();

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
