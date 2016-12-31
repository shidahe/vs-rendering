
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
        std::deque<long unsigned int> mdTextureQueue;
        size_t mnMaxTextureQueueSize;
        std::mutex mMutexTexture;

        //queue for the frames recieved, pair<mnID,image>
        std::map<long unsigned int, cv::Mat> mmFrameQueue;
        size_t mnMaxFrameQueueSize;
        std::mutex mMutexFrame;

    };
}


#endif //__MODELVIEWER_H
