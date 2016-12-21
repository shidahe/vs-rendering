//
// Created by shida on 05/12/16.
//

#include "Modeler/Modeler.h"

namespace ORB_SLAM2 {

    Modeler::Modeler():
            mbResetRequested(false), mbFinishRequested(false), mbFinished(true),
            mbFirstKeyFrame(true)
    {
        mAlgInterface.setAlgorithmRef(&mObjAlgorithm);
        mAlgInterface.setTranscriptRef(mTranscriptInterface.getTranscriptRef());
        mAlgInterface.rewind();
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
            if(CheckNewTranscriptEntry())
            {
                PopKeyFrameIntoTranscript();

                RunRemainder();
            }

            ResetIfRequested();

            if(CheckFinish())
                break;

            usleep(1000);
        }

        SetFinish();
    }

    bool Modeler::CheckNewTranscriptEntry()
    {
        unique_lock<mutex> lock(mMutexTranscript);
        return(!mlpTranscriptKeyFrameQueue.empty());
    }

    void Modeler::RunRemainder()
    {
        mAlgInterface.runRemainder();
    }

    void Modeler::PushKeyFrame(KeyFrame* pKF)
    {
        unique_lock<mutex> lock(mMutexTranscript);
        if(pKF->mnId!=0)
            mlpTranscriptKeyFrameQueue.push_back(pKF);
    }

    void Modeler::PopKeyFrameIntoTranscript()
    {
        unique_lock<mutex> lock(mMutexTranscript);
        KeyFrame* pKF = mlpTranscriptKeyFrameQueue.front();
        mlpTranscriptKeyFrameQueue.pop_front();
        // Avoid that a keyframe can be erased while it is being process by this thread
        pKF->SetNotErase();

        if (mbFirstKeyFrame) {
            mTranscriptInterface.addFirstKeyFrameInsertionEntry(pKF);
            mbFirstKeyFrame = false;
        } else {
            mTranscriptInterface.addKeyFrameInsertionEntry(pKF);
        }

        pKF->SetErase();
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
            usleep(1000);
        }
    }

    void Modeler::ResetIfRequested()
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbResetRequested)
        {
            {
                unique_lock<mutex> lock2(mMutexTranscript);
                mlpTranscriptKeyFrameQueue.clear();
            }
//            mTranscriptInterface.addResetEntry();
//            RunRemainder();
            mAlgInterface.rewind();
            mbFirstKeyFrame = true;

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
        //CARV
        mTranscriptInterface.writeToFile("sfmtranscript_orbslam.txt");
    }

    bool Modeler::isFinished()
    {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinished;
    }

}
