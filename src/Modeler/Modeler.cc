//
// Created by shida on 05/12/16.
//

#include "Modeler/Modeler.h"

#include <chrono>

// main header file for Line3D++
//#include "line3D.h"

// Header files needed by EDLines
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/// Function prototype for DetectEdgesByED exported by EDLinesLib.a
LS *DetectLinesByED(unsigned char *srcImg, int width, int height, int *pNoLines);

namespace ORB_SLAM2 {

    Modeler::Modeler(ModelDrawer* pModelDrawer):
            mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpModelDrawer(pModelDrawer),
            mnLastNumLines(2), mbFirstKeyFrame(true), mnMaxTextureQueueSize(10), mnMaxFrameQueueSize(10000),
            mnMaxToLinesQueueSize(100)
    {
        mAlgInterface.setAlgorithmRef(&mObjAlgorithm);
        mAlgInterface.setTranscriptRef(mTranscriptInterface.getTranscriptToProcessRef());
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

            if (CheckNewTranscriptEntry()) {

                RunRemainder();

                UpdateModelDrawer();
            }
            else {

                AddPointsOnLineSegments();
            }

            ResetIfRequested();

            if(CheckFinish())
                break;

            usleep(100);
        }

        SetFinish();

        //CARV
        unique_lock<mutex> lock(mMutexTranscript);
        mTranscriptInterface.writeToFile("sfmtranscript_orbslam.txt");
    }

    void Modeler::AddPointsOnLineSegments(){
        KeyFrame* pKF;
        {
            unique_lock<mutex> lock(mMutexToLines);
            if(mdToLinesQueue.size() > 0) {
                pKF = mdToLinesQueue.front();
                mdToLinesQueue.pop_front();
            } else {
                return;
            }
        }

        if(pKF->isBad())
            return;

        // Avoid that a keyframe can be erased while it is being process by this thread
        pKF->SetNotErase();

        std::vector<cv::Point3f> vPOnLine = GetPointsOnLineSegments(pKF);

        {
            unique_lock<mutex> lock(mMutexTranscript);
            KeyFrame* pKFcopy = new KeyFrame(pKF);
            mTranscriptInterface.addKeyFrameInsertionWithLinesEntry(pKF,pKFcopy,vPOnLine);
        }

        pKF->SetErase();

    }

    std::vector<cv::Point3f> Modeler::GetPointsOnLineSegments(KeyFrame* pKF){

        std::vector<cv::Point3f> vPOnLine;

        cv::Mat imGray;
        {
            unique_lock<mutex> lock(mMutexFrame);
            mmFrameQueue[pKF->mnFrameId].copyTo(imGray);
        }

        if(imGray.empty()){
            cout << "Empty image to draw line!" << endl;
            vPOnLine.clear();
            return vPOnLine;
        }

        if (imGray.channels() > 1) // this should be always true
            cv::cvtColor(imGray,imGray,CV_RGB2GRAY);

        std::vector<LineSegment> lines = DetectLineSegments(imGray);

        //       L3DPP::Line3D* Line3D = new L3DPP::Line3D(...);

        for(size_t indexLines = 0; indexLines < lines.size(); indexLines++){
            LineSegment& line = lines[indexLines];
            // set reference keyframe of the line segment
            line.mpRefKF = pKF;

            std::set<MapPoint*> vpMP = pKF->GetMapPoints();

            // calculate distance from point to line, if small enough, assign it to the supporting point list of the line
            cv::Point2f &start = line.mStart;
            cv::Point2f &end = line.mEnd;
            cv::Point2f diff = end - start;
            float l2 = std::pow(diff.x, 2.0f) + std::pow(diff.y, 2.0f);

            for (std::set<MapPoint*>::iterator it = vpMP.begin(); it != vpMP.end(); it++) {
                if ((*it)->isBad())
                    continue;
                if ((*it)->Observations() < 5)
                    continue;

                cv::Point2f xy = pKF->ProjectPointOnCamera(*it);

                float t = max(0.0f, min(1.0f, (xy-start).dot(end-start) / l2));
                cv::Point2f proj = start + t * (end - start);
                cv::Point2f minDiff = xy - proj;
                float distSqr = std::pow(minDiff.x, 2.0f) + std::pow(minDiff.y, 2.0f);
                if (distSqr < 2.0) {
                    line.mmpMPProj[*it] = t;
                }
            }



            if (line.mmpMPProj.size() >= 2) {
                //TODO: using the first and last at this time, probably change to svd
                cv::Point3f p1, p2;
                cv::Mat p1mat = line.mmpMPProj.begin()->first->GetWorldPos();
                cv::Mat p2mat = line.mmpMPProj.rbegin()->first->GetWorldPos();
                float t1 = line.mmpMPProj.begin()->second;
                float t2 = line.mmpMPProj.rbegin()->second;

                p1.x = p1mat.at<float>(0);
                p1.y = p1mat.at<float>(1);
                p1.z = p1mat.at<float>(2);
                p2.x = p2mat.at<float>(0);
                p2.y = p2mat.at<float>(1);
                p2.z = p2mat.at<float>(2);

//                cv::Point3f dt = (p2 - p1) * (1 / (t2 - t1));
//                cv::Point3f start3f = p1 - t1 * dt;
                cv::Point3f dt = p2 - p1;
                cv::Point3f start3f = p1;

                cout << "p1mat" << p1mat << endl;
                cout << "p2mat" << p2mat << endl;
                cout << "dt" << dt << endl;
                cout << "start3f" << start3f << endl;

                for (float i = 0.2; i <= 0.8; i += 0.1) {
                    cv::Point3f currP = start3f + i * dt;
                    cout << "currP" << currP << endl;
                    vPOnLine.push_back(currP);
                }
            }
        }

        cout << vPOnLine.size() << " points are generated from " << vPOnLine.size()/7 << " lines. " << lines.size()
             << " lines are detected from keyframe with " << pKF->TrackedMapPoints(3)  << " tracked points." << endl;


        //find points that are not bad from keyframes that are not bad
        //which are on the lines (supporting the 3d position calculation)
        //densify by add points from lines

        // maybe: remove line points from a keyframe if the keyframe is deleted
        // maybe: remove line points if too many supporting points are deleted
        // maybe: move line points if supporting points are moved

        {
            //update lines and image to draw
            unique_lock<mutex> lock(mMutexLines);
//            mvLines.clear();
//            std::vector<LineSegment>(mvLines).swap(mvLines);
            mvLines = lines;
            imGray.copyTo(mImLines);
        }

        return vPOnLine;
    }

    std::vector<cv::Point3f> Modeler::GetPointsOnLineSegmentsOffline(){

        std::vector<cv::Point3f> vPOnLine;

        std::vector<KeyFrame*> vpKF;

        // take keyframes out of queue
        while(1) {
            unique_lock<mutex> lock(mMutexToLines);
            if (mdToLinesQueue.size() > 0) {
                KeyFrame *pKF = mdToLinesQueue.front();
                // filter out bad keyframes
                if (pKF->isBad())
                    continue;
                vpKF.push_back(pKF);
                mdToLinesQueue.pop_front();
            } else {
                break;
            }
        }


        // for each keyframe, detect lines in image and save them in the map
        std::map<KeyFrame*,std::vector<LineSegment>> mvLSpKF;

        for (size_t indexKF = 0; indexKF < vpKF.size(); indexKF++) {

            KeyFrame* pKF = vpKF[indexKF];

            cv::Mat imGray;
            {
                unique_lock<mutex> lock(mMutexFrame);
                mmFrameQueue[pKF->mnFrameId].copyTo(imGray);
            }

            if (imGray.empty()) {
                cout << "Empty image to draw line!" << endl;
                vPOnLine.clear();
                continue;
            }

            if (imGray.channels() > 1) // this should be always true
                cv::cvtColor(imGray, imGray, CV_RGB2GRAY);

            std::vector<LineSegment> lines = DetectLineSegments(imGray);

            mvLSpKF[pKF] = lines;

        }

        // for each keyframe, get map points and match against every other keyframe
        for (size_t indexKF = 0; indexKF < vpKF.size(); indexKF++){
            KeyFrame* pKF = vpKF[indexKF];

            std::set<MapPoint*> vpMP = pKF->GetMapPoints();

            // get the keyframe to match
            for (size_t indKFMatch = indexKF; indKFMatch < vpKF.size(); indKFMatch++){
                KeyFrame* pKFMatch = vpKF[indKFMatch];
                std::set<MapPoint*> vpMPMatch = pKFMatch->GetMapPoints();

                // all matched points between these two keyframes
                std::vector<MapPoint*> vpMPAll;

                // find all matched points in two keyframes
                for (std::set<MapPoint*>::iterator it = vpMP.begin(); it != vpMP.end(); it++){

                    // filter out bad map points
                    if ((*it)->isBad())
                        continue;
                    // only keep confident points
                    if ((*it)->Observations() < 10)
                        continue;

                    std::set<MapPoint*>::iterator itMatch;
                    itMatch = vpMPMatch.find(*it);

                    if (itMatch == vpMPMatch.end())
                        continue;

                    // if a match found
                    vpMPAll.push_back(*it);
                }



            }
        }




        return vPOnLine;
    }

    std::vector<LineSegment> Modeler::DetectLineSegments(cv::Mat& im) {
        int width, height;
        unsigned char *srcImg;
        int noLines;

        width = im.size().width;
        height = im.size().height;

        // copy data to a new array
        int nSize = width*height;
        srcImg = new unsigned char[nSize];
        unsigned char *pImCopy = srcImg;
        unsigned char *pImData = im.data;
        for (int k = 0; k < nSize; k++) {
            *pImCopy++ = *pImData++;
        }

        LS *lines = DetectLinesByED(srcImg, width, height, &noLines);

        std::vector<LineSegment> vLines;
        for (int k = 0; k < noLines; k++) {
            LineSegment line(lines + k);
            vLines.push_back(line);
        }

        delete lines;
        delete[] srcImg;

        return vLines;
    }

    void Modeler::UpdateModelDrawer() {
        if(mpModelDrawer->UpdateRequested() && ! mpModelDrawer->UpdateDone()) {
            std::pair<std::vector<dlovi::Matrix>, std::list<dlovi::Matrix> > objModel = mAlgInterface.getCurrentModel();
            mpModelDrawer->SetUpdatedModel(objModel.first, objModel.second);
            mpModelDrawer->MarkUpdateDone();
        }
    }

    bool Modeler::CheckNewTranscriptEntry()
    {
        unique_lock<mutex> lock(mMutexTranscript);
        int numLines = mTranscriptInterface.getTranscriptRef()->numLines();
        if (numLines > mnLastNumLines) {
            mnLastNumLines = numLines;
            mTranscriptInterface.UpdateTranscriptToProcess();
            return true;
        } else {
            return false;
        }
    }

    void Modeler::RunRemainder()
    {
        mAlgInterface.runRemainder();
    }

    void Modeler::AddKeyFrameEntry(KeyFrame* pKF){
        if(pKF->isBad())
            return;

        // Avoid that a keyframe can be erased while it is being process by this thread
        pKF->SetNotErase();

        if (mbFirstKeyFrame) {
            unique_lock<mutex> lock(mMutexTranscript);
            mTranscriptInterface.addFirstKeyFrameInsertionEntry(pKF);
            mbFirstKeyFrame = false;
        } else {
            unique_lock<mutex> lock(mMutexTranscript);

//            auto t1 = std::chrono::high_resolution_clock::now();
//            std::vector<cv::Point3f> vPOnLine = GetPointsOnLineSegments(pKF);
//            auto t2 = std::chrono::high_resolution_clock::now();
//            std::cout << "GetPointsOnLineSegments() took "
//                      << std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count()
//                      << " milliseconds\n";
//
//            mTranscriptInterface.addKeyFrameInsertionWithLinesEntry(pKF,vPOnLine);

            mTranscriptInterface.addKeyFrameInsertionEntry(pKF);

        }

        AddTexture(pKF);

        DetectLineSegmentsLater(pKF);

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
        mTranscriptInterface.addBundleAdjustmentEntry(sAdjustSet, sMapPoints);
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
                mTranscriptInterface.writeToFile("sfmtranscript_orbslam.txt");
                mTranscriptInterface.addResetEntry();
                //TODO: fix crash when initialize again after reset
//            RunRemainder();
//            mAlgInterface.rewind();
            }
            {
                unique_lock<mutex> lock2(mMutexTexture);
//                {
//                    unique_lock<mutex> lock3(mMutexFrame);
//                    for (int i = 0; i < mnMaxTextureQueueSize; i++){
//                        TextureFrame texFrame = mdTextureQueue[i];
//                        std::string imname = "texKF" + std::to_string(i);
//                        if (texFrame.mpKF != NULL){
//                            if (texFrame.mpKF->isBad()){
//                                imname = imname + "bad";
//                            } else {
//                                imname = imname + "good";
//                            }
//                        }
//                        cv::imwrite(imname+".jpg", mmFrameQueue[texFrame.mFrameID]);
//                    }
//                }
                mdTextureQueue.clear();
            }
            {
                unique_lock<mutex> lock2(mMutexFrame);
                mmFrameQueue.clear();
            }
            {
                unique_lock<mutex> lock2(mMutexToLines);
                mdToLinesQueue.clear();
            }
            {
                unique_lock<mutex> lock2(mMutexLines);
                mvLines.clear();
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

    void Modeler::DetectLineSegmentsLater(KeyFrame* pKF)
    {
        unique_lock<mutex> lock(mMutexToLines);
        if (mdToLinesQueue.size() >= mnMaxToLinesQueueSize) {
            mdToLinesQueue.pop_front();
        }
        mdToLinesQueue.push_back(pKF);
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

    void Modeler::AddFrameImage(const long unsigned int &frameID, const cv::Mat &im)
    {
        unique_lock<mutex> lock(mMutexFrame);

        // make a copy of image and save as RGB
        cv::Mat imc;
        im.copyTo(imc);
        if(imc.channels() < 3)
            cvtColor(imc,imc,CV_GRAY2RGB);

        if (mmFrameQueue.size() >= mnMaxFrameQueueSize) {
            mmFrameQueue.erase(mmFrameQueue.begin());
        }
        if (mmFrameQueue.count(frameID) > 0){
            std::cerr << "ERROR: trying to add an existing frame" << std::endl;
            return;
        }
        mmFrameQueue.insert(make_pair(frameID,imc));
    }


    // get last n keyframes for texturing
    std::vector<pair<cv::Mat,TextureFrame>> Modeler::GetTextures(int n)
    {
        unique_lock<mutex> lock(mMutexTexture);
        unique_lock<mutex> lock2(mMutexFrame);
        int nLastKF = mdTextureQueue.size() - 1;
        std::vector<pair<cv::Mat,TextureFrame>> imAndTexFrame;
        // n most recent KFs
        for (int i = 0; i < n && i <= nLastKF; i++){
            TextureFrame texFrame = mdTextureQueue[std::max(0,nLastKF-i)];
            imAndTexFrame.push_back(make_pair(mmFrameQueue[texFrame.mFrameID],texFrame));
        }

        return imAndTexFrame;
    }

    cv::Mat Modeler::GetImageWithLines()
    {
        unique_lock<mutex> lock(mMutexLines);
        cv::Mat im;
        if(!mImLines.empty()) {
            mImLines.copyTo(im);
        } else {
            im = cv::Mat::zeros(200, 200, CV_8UC3);
        }

        if(im.channels() < 3) // this should always be true
            cvtColor(im,im,CV_GRAY2RGB);

        for(size_t i = 0; i < mvLines.size(); i++){
            LineSegment line = mvLines[i];
            cv::line(im, line.mStart, line.mEnd, cv::Scalar(0,255,0));

            // draw points on line segment
            std::map<MapPoint*,float> mpMP = line.mmpMPProj;
            for (std::map<MapPoint*, float>::iterator it = mpMP.begin(); it != mpMP.end(); it++){
                MapPoint * pMP = it->first;
                cv::Point2f pt = line.mpRefKF->ProjectPointOnCamera(pMP);
                const float r = 5;
                cv::Point2f pt1,pt2;
                pt1.x=pt.x-r;
                pt1.y=pt.y-r;
                pt2.x=pt.x+r;
                pt2.y=pt.y+r;
                cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
                cv::circle(im,pt,2,cv::Scalar(0,255,0),-1);
            }
        }

        return im;
    }


}
