//
// Created by shida on 05/12/16.
//

#include "Modeler/Modeler.h"

namespace ORB_SLAM2 {

    Modeler::Modeler() {
        mAlgInterface.setAlgorithmRef(&mObjAlgorithm);
        mAlgInterface.setTranscriptRef(mTranscriptInterface.getTranscriptRef());
        mAlgInterface.rewind();
        mbFirstKeyFrame = true;
        mbEmptyTranscript = true;
    }

    void Modeler::SetTracker(Tracking *pTracker)
    {
        mpTracker=pTracker;
    }

    void Modeler::SetLocalMapper(LocalMapping *pLocalMapper)
    {
        mpLocalMapper=pLocalMapper;
    }

    void Modeler::SetLoopCloser(LoopClosing* pLoopCloser)
    {
        mpLoopCloser = pLoopCloser;
    }

    void Modeler::Run()
    {
        mbFinished =false;

        while(1)
        {
            // Check if there are keyframes in the queue
            if(CheckNewEntry())
            {
                RunRemainder();
            }

            ResetIfRequested();

            if(CheckFinish())
                break;

            usleep(3000);
        }

        SetFinish();
    }

    bool Modeler::CheckNewEntry()
    {
        unique_lock<mutex> lock(mMutexTranscript);
        return(!mbEmptyTranscript);
    }

    void Modeler::RunRemainder()
    {
        unique_lock<mutex> lock(mMutexTranscript);
        mAlgInterface.runRemainder();
        mbEmptyTranscript = true;
    }

    void Modeler::RequestReset()
    {
        {
            unique_lock<mutex> lock(mMutexReset);
            mbResetRequested = true;
        }

        while(1)
        {
            {
                unique_lock<mutex> lock2(mMutexReset);
                if(!mbResetRequested)
                    break;
            }
            usleep(3000);
        }
    }

    void Modeler::ResetIfRequested()
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbResetRequested)
        {
            mbResetRequested=false;
        }
    }

    void Modeler::RequestFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinishRequested = true;
    }

    bool Modeler::CheckFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinishRequested;
    }

    void Modeler::SetFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinished = true;
    }

    bool Modeler::isFinished()
    {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinished;
    }

}
