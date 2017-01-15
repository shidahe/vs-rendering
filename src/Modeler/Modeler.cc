//
// Created by shida on 05/12/16.
//

#include "Modeler/Modeler.h"

namespace ORB_SLAM2 {

    Modeler::Modeler(ModelDrawer* pModelDrawer):
            mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpModelDrawer(pModelDrawer),
            mnLastNumLines(2), mbFirstKeyFrame(true), mnMaxTextureQueueSize(4), mnMaxFrameQueueSize(1000)
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

        while(1) {
//            // Check if there are keyframes in the queue
//            if (CheckNewKeyFrameTranscriptEntry()) {
//
//                PopKeyFrameIntoTranscript();
//            }

            if (CheckNewTranscriptEntry()) {

                RunRemainder();

                UpdateModelDrawer();
            }

            ResetIfRequested();

            if(CheckFinish())
                break;

            usleep(100);
        }

        SetFinish();

        //CARV
        unique_lock<mutex> lock2(mMutexTranscript);
        mTranscriptInterface.writeToFile("sfmtranscript_orbslam.txt");
    }

    void Modeler::UpdateModelDrawer(){
            if(mpModelDrawer->UpdateRequested() && ! mpModelDrawer->UpdateDone()){
                std::pair<std::vector<dlovi::Matrix>, std::list<dlovi::Matrix> > objModel = mAlgInterface.getCurrentModel();
                mpModelDrawer->SetUpdatedModel(objModel.first, objModel.second);
                mpModelDrawer->MarkUpdateDone();
            }
    }

    bool Modeler::CheckNewTranscriptEntry()
    {
        unique_lock<mutex> lock(mMutexTranscript);
        int numLines = mTranscriptInterface.getTranscriptRef()->numLines();
        return numLines > mnLastNumLines;
    }

    bool Modeler::CheckNewKeyFrameTranscriptEntry()
    {
        unique_lock<mutex> lock(mMutexTranscript);
        return(!mlpTranscriptKeyFrameQueue.empty());
    }

    bool Modeler::CheckNewFrameTranscriptEntry()
    {
        unique_lock<mutex> lock(mMutexTranscript);
        return(!mlpTranscriptFrameQueue.empty());
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

    bool Modeler::PopKeyFrameIntoTranscript()
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
        //AddTexture(pKF);

        pKF->SetErase();
        return true;
    }

    void Modeler::PushFrame(ModelFrame* pMF)
    {
        unique_lock<mutex> lock(mMutexTranscript);
        if(pMF->mFrameID!=0)
            mlpTranscriptFrameQueue.push_back(pMF);
    }

    bool Modeler::PopFrameIntoTranscript()
    {
        unique_lock<mutex> lock(mMutexTranscript);
        ModelFrame* pMF = mlpTranscriptFrameQueue.front();
        mlpTranscriptFrameQueue.pop_front();

        if (pMF->mTcw.empty())
            return false;

        if (mbFirstKeyFrame) {
            mTranscriptInterface.addFirstFrameInsertionEntry(pMF);
            mbFirstKeyFrame = false;
        } else {
            mTranscriptInterface.addFrameInsertionEntry(pMF);
        }

        return true;
    }

    void Modeler::AddKeyFrameEntry(KeyFrame* pKF){
        unique_lock<mutex> lock(mMutexTranscript);
        // Avoid that a keyframe can be erased while it is being process by this thread
        pKF->SetNotErase();

        if (mbFirstKeyFrame) {
            mTranscriptInterface.addFirstKeyFrameInsertionEntry(pKF);
            mbFirstKeyFrame = false;
        } else {
            mTranscriptInterface.addKeyFrameInsertionEntry(pKF);
        }
        //AddTexture(pKF);

        pKF->SetErase();
    }

    void Modeler::AddDeletePointEntry(MapPoint* pMP){
        unique_lock<mutex> lock(mMutexTranscript);
        mTranscriptInterface.addPointDeletionEntry(pMP);
    }

    void Modeler::AddDeleteObservationEntry(KeyFrame *pKF, MapPoint *pMP) {
        unique_lock<mutex> lock(mMutexTranscript);
        mTranscriptInterface.addVisibilityRayDeletionEntry(pKF, pMP);
    }

    void Modeler::AddAdjustmentEntry(std::set<KeyFrame*> & sAdjustSet, std::set<MapPoint*> & sMapPoints){
        unique_lock<mutex> lock(mMutexTranscript);

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
            usleep(100);
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
                mlpTranscriptFrameQueue.clear();
            }
            mTranscriptInterface.addResetEntry();
//            RunRemainder();
//            mAlgInterface.rewind();
            {
                unique_lock<mutex> lock2(mMutexTexture);
                mmFrameQueue.clear();
            }
            {
                unique_lock<mutex> lock(mMutexTexture);
                mdTextureQueue.clear();
            }
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
    }

    bool Modeler::isFinished()
    {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinished;
    }

    void Modeler::AddTexture(KeyFrame* pKF)
    {
        unique_lock<mutex> lock(mMutexTexture);

        TextureFrame texFrame(pKF);

        if (mdTextureQueue.size() >= mnMaxTextureQueueSize) {
            mdTextureQueue.pop_front();
        }

        mdTextureQueue.push_back(texFrame);

    }

    void Modeler::AddTexture(Frame* pF)
    {
        unique_lock<mutex> lock(mMutexTexture);

        TextureFrame texFrame(pF);

        if (mdTextureQueue.size() >= mnMaxTextureQueueSize) {
            mdTextureQueue.pop_front();
        }

        mdTextureQueue.push_back(texFrame);

    }

    void Modeler::AddFrame(const long unsigned int &frameID, const cv::Mat &im)
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


    // get last n keyframes for texturing
    vector<pair<cv::Mat,TextureFrame>> Modeler::GetTextures(int n)
    {
        unique_lock<mutex> lock(mMutexTexture);
        unique_lock<mutex> lock2(mMutexFrame);
        int nLastKF = mdTextureQueue.size() - 1;
        vector<pair<cv::Mat,TextureFrame>> imAndTexFrame;
        // n most recent KFs
        for (int i = 0; i < n && i <= nLastKF; i++){
            TextureFrame texFrame = mdTextureQueue[std::max(0,nLastKF-i)];
            imAndTexFrame.push_back(make_pair(mmFrameQueue[texFrame.mFrameID],texFrame));
        }

        return imAndTexFrame;
    }

}
