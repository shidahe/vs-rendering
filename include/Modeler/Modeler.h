
#ifndef __MODELER_H
#define __MODELER_H

#include "LoopClosing.h"
#include "LocalMapping.h"
#include "Tracking.h"
#include "KeyFrame.h"

#include <mutex>

#include "Modeler/SFMTranscriptInterface_ORBSLAM.h"
#include "Modeler/SFMTranscriptInterface_Delaunay.h"

namespace ORB_SLAM2 {

    class Tracking;
    class LoopClosing;
    class LocalMapping;
    class KeyFrame;

    class Modeler {
    public:
        Modeler();

        void SetLoopCloser(LoopClosing* pLoopCloser);

        void SetLocalMapper(LocalMapping* pLocalMapper);

        void SetTracker(Tracking* pTracker);

        // Main function
        void Run();

        bool CheckNewTranscriptEntry();
        void RunRemainder();

        // Thread Synch
        void RequestReset();
        void RequestFinish();
        bool isFinished();

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

    };
}


#endif //__MODELVIEWER_H
