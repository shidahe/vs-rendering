//
// Created by shida on 05/12/16.
//

#include "Modeler/Modeler.h"

// Header files needed by EDLines
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/// Function prototype for DetectEdgesByED exported by EDLinesLib.a
LS *DetectLinesByED(unsigned char *srcImg, int width, int height, int *pNoLines);

namespace ORB_SLAM2 {

    Modeler::Modeler(ModelDrawer* pModelDrawer):
            mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpModelDrawer(pModelDrawer),
            mnLastNumLines(2), mbFirstKeyFrame(true), mnMaxTextureQueueSize(4), mnMaxFrameQueueSize(1000),
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

        cv::Mat imGray;
        {
            unique_lock<mutex> lock(mMutexFrame);
            mmFrameQueue[pKF->mnFrameId].copyTo(imGray);
        }

        if(imGray.empty()){
            pKF->SetErase();
            cout << "Empty image to draw line!" << endl;
            return;
        }

        if (imGray.channels() > 1) // this should be always true
            cv::cvtColor(imGray,imGray,CV_RGB2GRAY);

        vector<LineSegment> lines = DetectLineSegments(imGray);

        for(size_t indexLines = 0; indexLines < lines.size(); indexLines++){
            LineSegment& line = lines[indexLines];
            // set reference keyframe of the line segment
            line.mpRefKF = pKF;

            std::cout << "Lines: " << indexLines+1 << ":" << lines.size() << "(" << line.mStart.x << ","
                      << line.mStart.y << ") (" << line.mEnd.x << "," << line.mEnd.y << ")" << std::endl;

        //calculate distance map of lines
//        for (size_t indexLines = 0; indexLines < lines.size(); indexLines++){
            //assign pixels on lines to 255
//            cv::Point2d start(lines[indexLines].sx, lines[indexLines].sy);
//            cv::Point2d end(lines[indexLines].ex, lines[indexLines].ey);
//            cv::Mat binImage;
//            binImage = cv::Mat::zeros(imGray.size(), CV_8UC1);
//            cv::line(binImage, lines[indexLines].mStart, lines[indexLines].mEnd, cv::Scalar(255));
//            cv::Mat dist;
//            cv::distanceTransform(binImage, dist, CV_DIST_L2, 3);

//            normalize(dist, dist, 0, 1., cv::NORM_MINMAX);
//            cv::imwrite("distance_transfrom.jpg",dist);

            //project points that were newly added when the keyframe entry was added, to the distance map
//            vector<MapPoint *> vpMP = mTranscriptInterface.GetNewPoints(pKF);
            set<MapPoint*> spMP = pKF->GetMapPoints();

            // calculate distance from point to line, if small enough, assign it to the supporting point list of the line
            cv::Point2f &start = line.mStart;
            cv::Point2f &end = line.mEnd;
            cv::Point2f diff = end - start;
            float l2 = std::pow(diff.x, 2.0f) + std::pow(diff.y, 2.0f);

            for (set<MapPoint*>::iterator it = spMP.begin(); it != spMP.end(); it++) {
                if ((*it)->isBad())
                    continue;

                cv::Point2f xy = pKF->ProjectPointOnCamera(*it);

                float t = max(0.0f, min(1.0f, (xy-start).dot(end-start) / l2));
                cv::Point2f proj = start + t * (end - start);
                cv::Point2f minDiff = xy - proj;
                float distSqr = std::pow(minDiff.x, 2.0f) + std::pow(minDiff.y, 2.0f);
                if (distSqr < 2.0)
                    line.mvpMP.push_back(*it);
            }
        }

        //find points that are not bad from keyframes that are not bad
        //which are on the lines (supporting the 3d position calculation)
        //densify by add points from lines

        // maybe: remove line points from a keyframe if the keyframe is deleted
        // maybe: remove line points if too many supporting points are deleted
        // maybe: move line points if supporting points are moved


        {
            //update lines and image to draw
            unique_lock<mutex> lock(mMutexLines);
            mvLines = lines;
            mImLines = imGray;
        }

        pKF->SetErase();

    }

    vector<LineSegment> Modeler::DetectLineSegments(cv::Mat im) {
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

        vector<LineSegment> vLines;
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
        unique_lock<mutex> lock(mMutexTranscript);

        if(pKF->isBad())
            return;

        // Avoid that a keyframe can be erased while it is being process by this thread
        pKF->SetNotErase();

        cv::Mat imGray;
        {
            unique_lock<mutex> lock(mMutexFrame);
            mmFrameQueue[pKF->mnFrameId].copyTo(imGray);
        }

        if(imGray.empty()){
            pKF->SetErase();
            cout << "Empty image to draw line!" << endl;
            return;
        }

        if (imGray.channels() > 1) // this should be always true
            cv::cvtColor(imGray,imGray,CV_RGB2GRAY);

        vector<LineSegment> lines = DetectLineSegments(imGray);

        vector<cv::Point3f> vPOnLine;

        for(size_t indexLines = 0; indexLines < lines.size(); indexLines++) {
            LineSegment &line = lines[indexLines];
            line.mpRefKF = pKF;

            std::cout << "Lines: " << indexLines+1 << ":" << lines.size() << "(" << line.mStart.x << ","
                      << line.mStart.y << ") (" << line.mEnd.x << "," << line.mEnd.y << ")" << std::endl;

            set<MapPoint*> spMP = pKF->GetMapPoints();

            // calculate distance from point to line, if small enough, assign it to the supporting point list of the line
            cv::Point2f &start = line.mStart;
            cv::Point2f &end = line.mEnd;
            cv::Point2f diff = end - start;
            float l2 = std::pow(diff.x, 2.0f) + std::pow(diff.y, 2.0f);

            for (set<MapPoint*>::iterator it = spMP.begin(); it != spMP.end(); it++) {
                if ((*it)->isBad())
                    continue;
                if ((*it)->Observations() < 3)
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
            if (line.mmpMPProj.size() >= 3){
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

                cv::Point3f dt = (p2 - p1) * (1 / (t2 - t1));
                cv::Point3f start3f = p1 - t1 * dt;

                for (float i = 0.0; i < 1.0; i+=0.1){
                    cv::Point3f currP = start3f + i * dt;
                    vPOnLine.push_back(currP);
                }
            }
        }

        if (mbFirstKeyFrame) {
            mTranscriptInterface.addFirstKeyFrameInsertionWithLinesEntry(pKF, vPOnLine);
            mbFirstKeyFrame = false;
        } else {
            mTranscriptInterface.addKeyFrameInsertionWithLinesEntry(pKF, vPOnLine);
        }

        AddTexture(pKF);

//        DetectLineSegmentsLater(pKF);

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
                mTranscriptInterface.addResetEntry();
                //TODO: fix crash when initialize again after reset
//            RunRemainder();
//            mAlgInterface.rewind();
            }
            {
                unique_lock<mutex> lock2(mMutexTexture);
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
            vector<MapPoint*> vpMP = line.mvpMP;
            for (size_t j = 0; j < vpMP.size(); j++){
                MapPoint * pMP = vpMP[j];
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
