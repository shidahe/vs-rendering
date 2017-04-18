//
// Created by shida on 05/12/16.
//

#include "Modeler/Modeler.h"

#include <ctime>

#include <chrono>

// main header file for Line3D++
//#include "line3D.h"

// Header files needed by EDLines
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <include/Modeler/Modeler.h>

/// Function prototype for DetectEdgesByED exported by EDLinesLib.a
LS *DetectLinesByED(unsigned char *srcImg, int width, int height, int *pNoLines);

namespace ORB_SLAM2 {

    Modeler::Modeler(ModelDrawer* pModelDrawer):
            mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpModelDrawer(pModelDrawer),
            mnLastNumLines(2), mbFirstKeyFrame(true), mnMaxTextureQueueSize(10), mnMaxFrameQueueSize(10000),
            mnMaxToLinesQueueSize(1000)
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
//            else {
//
//                AddPointsOnLineSegments();
//            }

            ResetIfRequested();

            if(CheckFinish())
                break;

            usleep(100);
        }

        std::cout << std::endl << "Getting line crossings ..." << std::endl;
        std::clock_t start;
        double duration;
        start = std::clock();
        std::vector<LinePoint> vPOnLine = GetPointsOnLineSegmentsOffline();
        duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
        std::cout << std::endl << "Computing line crossing took " << std::to_string(duration) << "s" << std::endl;

        std::cout << std::endl << "Saving line crossings to OBJ file ..." << std::endl;
        saveLinePointToFile(vPOnLine, "line_crossings.obj");
        std::cout << std::endl << "line crossings saved!" << std::endl;

        //CARV
        {
            unique_lock<mutex> lock(mMutexTranscript);
            std::cout << std::endl << "Saving transcript to file ..." << std::endl;
            mTranscriptInterface.writeToFile("sfmtranscript_orbslam.txt");
            std::cout << std::endl << "transcript saved!" << std::endl;
        }

        SetFinish();

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

                // need to test if xy > 0
                cv::Point2f xy = pKF->ProjectPointOnCamera((*it)->GetWorldPos());

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



    double Modeler::computeNG(double N, double Khat){
        if (Khat > 0.5)
            return 0;
        // compute solution for the quadratic equation
        double a = 1.0 / 6.0;
        double b = - (0.5 - N/3.0);
        double c = - N * (N-1) * (0.5-Khat);
        double underSqrt = b*b - 4*a*c;
        double root = std::sqrt(underSqrt) / (2*a);
        double NG = - b / (2*a);
        // only return solution between 0 and N
        if (NG+root >= 0 && NG+root <= N){
            return NG+root;
        } else if (NG-root >= 0 && NG-root <= N){
            return NG-root;
        } else {
            return 0;
        }
    }

    std::vector<LinePoint> Modeler::GetPointsOnLineSegmentsOffline(){

        std::vector<LinePoint> vLPOnLine;

        // common points on matched line segments
        std::vector<LinePoint> vLPOnLineMatched;


        std::vector<KeyFrame*> vpKF;

        // take keyframes out of queue
        while(1) {
            unique_lock<mutex> lock(mMutexToLines);
            if (mdToLinesQueue.size() > 0) {
                KeyFrame *pKF = mdToLinesQueue.front();
                mdToLinesQueue.pop_front();

                // filter out bad keyframes
                if (pKF->isBad())
                    continue;
                vpKF.push_back(pKF);

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
                continue;
            }

            if (imGray.channels() > 1) // this should be always true
                cv::cvtColor(imGray, imGray, CV_RGB2GRAY);

            std::vector<LineSegment> lines = DetectLineSegments(imGray);

            for(size_t indexLines = 0; indexLines < lines.size(); indexLines++) {
                LineSegment &line = lines[indexLines];
                // set reference keyframe of the line segment
                line.mpRefKF = pKF;
            }

            mvLSpKF.insert(std::map<KeyFrame*,std::vector<LineSegment>>::value_type(pKF, lines));
        }

        // all virtual line segments
        std::vector<VirtualLineSegment> vVLSAll;

        // every pair of keyframes
        std::vector<std::pair<KeyFrame*,KeyFrame*>> vpairPKF;

        // get all best pairs
        for (size_t indexKF = 0; indexKF < vpKF.size(); indexKF++){
            // search from end to begin
            KeyFrame* pKF = vpKF[vpKF.size()-1-indexKF];

            // only match against keyframes with best covisibility
            std::vector<KeyFrame*> vKFBestCov5 = pKF->GetBestCovisibilityKeyFrames(5);
            if (vKFBestCov5.size() < 5)
                continue;
            // only use keyframes that are best
            std::vector<KeyFrame*> vKFBestCov(vKFBestCov5.begin(), vKFBestCov5.begin()+1);

            // get the keyframe to match
            for (size_t indKFMatch = indexKF; indKFMatch < vpKF.size(); indKFMatch++){
                KeyFrame* pKFMatch = vpKF[vpKF.size()-1-indKFMatch];
                // if it is not a best covisible keyframe
                if (std::find(vKFBestCov.begin(), vKFBestCov.end(), pKFMatch) == vKFBestCov.end())
                    continue;
                vpairPKF.push_back(std::make_pair(pKF,pKFMatch));
            }
        }

        std::map<MapPoint*,int> mpMPScore;

        // for each keyframe, get map points and match against every other keyframe
        for (size_t indexKFPair = 0; indexKFPair < vpairPKF.size(); indexKFPair++){
            KeyFrame* pKF = vpairPKF[indexKFPair].first;
            std::set<MapPoint*> vpMP = pKF->GetMapPoints();

            // get the keyframe to match
            KeyFrame* pKFMatch = vpairPKF[indexKFPair].second;
            std::set<MapPoint*> vpMPMatch = pKFMatch->GetMapPoints();

            // all matched points between these two keyframes
            std::vector<MapPoint*> vpMPMatched;

            // find all matched points in two keyframes
            for (std::set<MapPoint*>::iterator it = vpMP.begin(); it != vpMP.end(); it++){
                // only keep a fixed number of matched points
                if (vpMPMatched.size() >= 100)
                    break;

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
                vpMPMatched.push_back(*it);

                // initialize score map for map points
                mpMPScore.insert(std::map<MapPoint*,int>::value_type(*it, 0));
            }

            // all combinations of pairs of matched points between two keyframes
            std::vector<VirtualLineSegment> vVLS;

            for (size_t indexMPStart = 0; indexMPStart < vpMPMatched.size(); indexMPStart++){
                for (size_t indexMPEnd = indexMPStart+1; indexMPEnd < vpMPMatched.size(); indexMPEnd++) {
                    VirtualLineSegment vls(vpMPMatched[indexMPStart], vpMPMatched[indexMPEnd]);
                    vVLS.push_back(vls);
                }
            }

            std::cout << "Keyframe " << std::to_string(pKF->mnId) << "<->" << std::to_string(pKFMatch->mnId)
                      << " #VLS:" << std::to_string(vVLS.size()) << " #Matched points: "
                      << std::to_string(vpMPMatched.size())<< std::endl;


            // map for finding line segment matches
            std::map<LineSegment*,std::vector<LinePoint>> mpLSvLP;

            // initialize the map
            for (size_t indexLS = 0; indexLS < mvLSpKF[pKF].size(); indexLS++) {
                LineSegment* pLS = &mvLSpKF[pKF][indexLS];
                std::vector<LinePoint> vLP;
                mpLSvLP.insert(std::map<LineSegment*,std::vector<LinePoint>>::value_type(pLS, vLP));
            }
            for (size_t indexLS = 0; indexLS < mvLSpKF[pKFMatch].size(); indexLS++) {
                LineSegment* pLS = &mvLSpKF[pKFMatch][indexLS];
                std::vector<LinePoint> vLP;
                mpLSvLP.insert(std::map<LineSegment*,std::vector<LinePoint>>::value_type(pLS, vLP));
            }

            const double MAX_VLS_LEN = 1000;
            const double MIM_VLS_LEN = 10;
            // vls extension ratio
            const double VLS_EXTEND = 1.0;
            // line points matching threshold
            const double TH_LP_MATCH = 0.5;
            // threshold of number of line points on line segment
            const unsigned long TH_LPONLS = 5;

            // for each virtual line segment, compute the intersection of its projection and the line segments
            for (size_t indexVLS = 0; indexVLS < vVLS.size(); indexVLS++){

                VirtualLineSegment& vls = vVLS[indexVLS];

                cv::Point2f startVLS2f = pKF->ProjectPointOnCamera(vls.mpMPStart->GetWorldPos());
                cv::Point2f endVLS2f = pKF->ProjectPointOnCamera(vls.mpMPEnd->GetWorldPos());
                // test if the virtual line segment is in image, this should be always be false
                if (startVLS2f.x < 0 || startVLS2f.y < 0 || endVLS2f.x < 0 || endVLS2f.y < 0)
                    continue;
                cv::Point2f seVLS2f = endVLS2f - startVLS2f;
                // filter out virtual line segments that the pair of points are too far away
                if (cv::norm(seVLS2f) > MAX_VLS_LEN || cv::norm(seVLS2f) < MIM_VLS_LEN)
                    continue;

                // vls in second keyframe
                cv::Point2f startVLS2fMatch = pKFMatch->ProjectPointOnCamera(vls.mpMPStart->GetWorldPos());
                cv::Point2f endVLS2fMatch = pKFMatch->ProjectPointOnCamera(vls.mpMPEnd->GetWorldPos());
                // test if the virtual line segment is in image, this should be always be false
                if (startVLS2fMatch.x < 0 || startVLS2fMatch.y < 0 || endVLS2fMatch.x < 0 || endVLS2fMatch.y < 0)
                    continue;
                cv::Point2f seVLS2fMatch = endVLS2fMatch - startVLS2fMatch;
                // filter out virtual line segments that the pair of points are too far away
                if (cv::norm(seVLS2fMatch) > MAX_VLS_LEN || cv::norm(seVLS2fMatch) < MIM_VLS_LEN)
                    continue;

                // line segments in the first keyframe
                std::vector<LineSegment>& vLS = mvLSpKF[pKF];
                // compute all intersections on the VLS
                std::vector<std::pair<LineSegment*,cv::Point3f>> vls3fIntersects;
                std::map<LineSegment*,double> mpLSuLS;

                for (size_t indexLS = 0; indexLS < vLS.size(); indexLS++) {
                    LineSegment &line = vLS[indexLS];

                    // find intersection on image
                    // http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
                    cv::Point2f startLS2f = line.mStart;
                    cv::Point2f endLS2f = line.mEnd;
                    cv::Point2f seLS2f = endLS2f - startLS2f;

                    double rxs = seVLS2f.cross(seLS2f);
                    // if rxs is zero
                    if (abs(rxs) <= std::numeric_limits<double>::epsilon())
                        continue;

                    double tVLS = (startLS2f - startVLS2f).cross(seLS2f) / rxs;
                    double uLS = (startLS2f - startVLS2f).cross(seVLS2f) / rxs;
                    // if not intersected with detected line segment
                    if (uLS < 0.0 || uLS > 1.0)
                        continue;
                    double vlsExtend = VLS_EXTEND;
                    // if not intersected with VLS with some expansion
                    if (tVLS < 0 - (vlsExtend-1) / 2 || tVLS > 1 + (vlsExtend-1) / 2)
                        continue;

                    cv::Point2f intersect2f = startLS2f + uLS * seLS2f;

                    // intersection found, project it to 3d
                    cv::Mat TwcKF = pKF->GetPoseInverse();
                    cv::Mat RwcKF = TwcKF.rowRange(0, 3).colRange(0, 3);
                    cv::Mat twcKF = TwcKF.rowRange(0, 3).col(3);
                    float PcX = (intersect2f.x - pKF->cx) / pKF->fx;
                    float PcY = (intersect2f.y - pKF->cy) / pKF->fy;

                    cv::Mat Pc = cv::Mat(3, 1, CV_32F);
                    Pc.at<float>(0) = PcX;
                    Pc.at<float>(1) = PcY;
                    Pc.at<float>(2) = 1.0;

                    // position of the line-crossing in world coordinate
                    cv::Mat Pw = RwcKF * Pc + twcKF;

                    // compute intersection between the ray from camera center towards line-crossing and the 3D VLS
                    // http://mathworld.wolfram.com/Line-LineIntersection.html
                    cv::Point3f Pw3f(Pw);
                    cv::Point3f Ow3f(twcKF);
                    cv::Point3f seOP3f = Pw3f - Ow3f;

                    cv::Point3f startVLS3f = vls.mStart;
                    cv::Point3f endVLS3f = vls.mEnd;
                    cv::Point3f seVLS3f = endVLS3f - startVLS3f;

                    cv::Point3f axbOP = seVLS3f.cross(seOP3f);
                    double normaxb = norm(axbOP);
                    // if norm of axb is zero, parallel
                    if (abs(normaxb) <= std::numeric_limits<double>::epsilon())
                        continue;

                    double sVLS = (Ow3f - startVLS3f).cross(seOP3f).dot(axbOP) / std::pow(normaxb, 2.0);

                    // intersection of ray and VLS, should always be on the line of VLS
                    cv::Point3f intersect3f = startVLS3f + sVLS * seVLS3f;

                    vls3fIntersects.push_back(std::make_pair(&line,intersect3f));
                    mpLSuLS.insert(std::map<LineSegment*,double>::value_type(&line,uLS));
                }

                // line segments in second keyframe
                std::vector<LineSegment>& vLSMatch = mvLSpKF[pKFMatch];
                // compute all intersections on the VLS in second keyframe
                std::vector<std::pair<LineSegment*,cv::Point2f>> vls2fIntersectsMatch;
                std::map<LineSegment*,double> mpLSuLSMatch;

                for (size_t indexLSMatch = 0; indexLSMatch < vLSMatch.size(); indexLSMatch++) {
                    LineSegment &lineMatch = vLSMatch[indexLSMatch];
                    cv::Point2f startLSMatch2f = lineMatch.mStart;
                    cv::Point2f endLSMatch2f = lineMatch.mEnd;

                    cv::Point2f seLSMatch2f = endLSMatch2f - startLSMatch2f;

                    // find intersection on image to match
                    // http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
                    double rxsMatch = seVLS2fMatch.cross(seLSMatch2f);
                    // if rxs is zero
                    if (abs(rxsMatch) <= std::numeric_limits<double>::epsilon())
                        continue;

                    double tVLSMatch = (startLSMatch2f - startVLS2fMatch).cross(seLSMatch2f) / rxsMatch;
                    double uLSMatch = (startLSMatch2f - startVLS2fMatch).cross(seVLS2fMatch) / rxsMatch;

                    // if not intersected with detected line segment
                    if (uLSMatch < 0.0 || uLSMatch > 1.0)
                        continue;
                    double vlsMatchExtend = VLS_EXTEND;
                    // if not intersected with VLS with some expansion
                    if (tVLSMatch < 0 - (vlsMatchExtend-1) / 2 || tVLSMatch > 1 + (vlsMatchExtend-1) / 2)
                        continue;

                    cv::Point2f intersectMatch2f = startLSMatch2f + uLSMatch * seLSMatch2f;

                    // don't need to compute 3d from here as we are comparing in image coordinate
                    vls2fIntersectsMatch.push_back(std::make_pair(&lineMatch, intersectMatch2f));
                    mpLSuLSMatch.insert(std::map<LineSegment*,double>::value_type(&lineMatch,uLSMatch));
                }

                // temp map for line segment and line points
                std::vector<LineSegment*> vpLStemp;
                std::vector<LineSegment*> vpLSMatchtemp;
                std::vector<LinePoint> vLPtemp;

                // loop through all intersections to find corresponding ones
                for (size_t indIntersect = 0; indIntersect < vls3fIntersects.size(); indIntersect++) {
                    LineSegment *line = vls3fIntersects[indIntersect].first;
                    cv::Point3f intersect3f = vls3fIntersects[indIntersect].second;

                    cv::Mat intersect3D = cv::Mat(3, 1, CV_32F);
                    intersect3D.at<float>(0) = intersect3f.x;
                    intersect3D.at<float>(1) = intersect3f.y;
                    intersect3D.at<float>(2) = intersect3f.z;

                    // project 3D intersection onto the second keyframe
                    cv::Point2f lineCrossMatch = pKFMatch->ProjectPointOnCamera(intersect3D);
                    if (lineCrossMatch.x < 0 || lineCrossMatch.y < 0)
                        continue;

                    // loop through line segment intersections in the second keyframe
                    for (size_t indIntersectMatch = 0;
                         indIntersectMatch < vls2fIntersectsMatch.size(); indIntersectMatch++) {
                        LineSegment *lineMatch = vls2fIntersectsMatch[indIntersectMatch].first;
                        cv::Point2f intersectMatch2f = vls2fIntersectsMatch[indIntersectMatch].second;

                        // check distance between projection of 3D intersection and intersection in second keyframe
                        double distLCIntersectMatch = cv::norm(lineCrossMatch - intersectMatch2f);
                        // the distance is small enough (in pixel)
                        if (distLCIntersectMatch <= TH_LP_MATCH) {
                            // there is a match, save it in the virtual line segment
                            LinePoint lp(intersect3f);
                            lp.addRefLineSegment(line, mpLSuLS[line]);
                            lp.addRefLineSegment(lineMatch, mpLSuLSMatch[lineMatch]);
                            lp.addRefVLS(pKF, vVLS[indexVLS]);
                            // add to temp vectors
                            vpLStemp.push_back(line);
                            vpLSMatchtemp.push_back(lineMatch);
                            vLPtemp.push_back(lp);
                        }
                    }
                }

                // check number of matches on a vls
                if (vLPtemp.size() >= 2){
                    for (size_t indTemp = 0; indTemp < vLPtemp.size(); indTemp++){
                        // add to the maps for line segment matches
                        mpLSvLP[vpLStemp[indTemp]].push_back(vLPtemp[indTemp]);
                        mpLSvLP[vpLSMatchtemp[indTemp]].push_back(vLPtemp[indTemp]);
                        vls.mvLPs.push_back(vLPtemp[indTemp]);
                    }
                }
            }

            //TODO: cluster line points to clean the line points cloud, cluster parameter depends on slam initialization
            //TODO: merge line segment refs when cluster to have matches of line segments across keyframes

            // find matches for line segments
            std::map<LineSegment*, LineSegment*> mpLSpLSMatch;
            std::map<LineSegment*,std::vector<LinePoint>> mpLSpLSMatchvLP;

            std::map<LineSegment*, LineSegment*> mpLSMatchpLS;
            std::map<LineSegment*,std::vector<LinePoint>> mpLSMatchpLSvLP;

            // find line segment in match kf that shares most line points with original kf
            for (size_t indexLS = 0; indexLS < mvLSpKF[pKF].size(); indexLS++) {
                LineSegment* pLS = &(mvLSpKF[pKF][indexLS]);
                std::vector<LinePoint>& vLP = mpLSvLP[pLS];
                LineSegment* pLSMaxMatch = NULL;
                std::vector<LinePoint> vLPonLSMaxMatch;
                for (size_t indexLSMatch = 0; indexLSMatch < mvLSpKF[pKFMatch].size(); indexLSMatch++){
                    LineSegment* pLSMatch = &(mvLSpKF[pKFMatch][indexLSMatch]);
                    std::vector<LinePoint>& vLPinLSMatch = mpLSvLP[pLSMatch];
                    // get common line points
                    std::vector<LinePoint> vLPonLS;
                    for (size_t indexLPinLS = 0; indexLPinLS < vLP.size(); indexLPinLS++){
                        if (std::find(vLPinLSMatch.begin(), vLPinLSMatch.end(), vLP[indexLPinLS]) != vLPinLSMatch.end()){
                            vLPonLS.push_back(vLP[indexLPinLS]);
                        }
                    }
                    if (vLPonLS.size() < TH_LPONLS) {
                        vLPonLS.clear();
                        continue;
                    }
                    // keep line segment that shares the most line points
                    if (pLSMaxMatch != NULL){
                        if (vLPonLS.size() <= vLPonLSMaxMatch.size()) {
                            continue;
                        }
                    }
                    pLSMaxMatch = pLSMatch;
                    vLPonLSMaxMatch.clear();
                    vLPonLSMaxMatch.insert(vLPonLSMaxMatch.end(), vLPonLS.begin(), vLPonLS.end());
                }
                // insert into map
                mpLSpLSMatch.insert(std::map<LineSegment*,LineSegment*>::value_type(pLS, pLSMaxMatch));
                mpLSpLSMatchvLP.insert(std::map<LineSegment*,std::vector<LinePoint>>::value_type(pLS, vLPonLSMaxMatch));
            }

            // find line segment in original kf that shares most line points with match kf
            for (size_t indexLS = 0; indexLS < mvLSpKF[pKFMatch].size(); indexLS++) {
                LineSegment* pLS = &(mvLSpKF[pKFMatch][indexLS]);
                std::vector<LinePoint>& vLP = mpLSvLP[pLS];
                LineSegment* pLSMaxMatch = NULL;
                std::vector<LinePoint> vLPonLSMaxMatch;
                for (size_t indexLSMatch = 0; indexLSMatch < mvLSpKF[pKF].size(); indexLSMatch++){
                    LineSegment* pLSMatch = &(mvLSpKF[pKF][indexLSMatch]);
                    std::vector<LinePoint>& vLPinLSMatch = mpLSvLP[pLSMatch];
                    // get common line points
                    std::vector<LinePoint> vLPonLS;
                    for (size_t indexLPinLS = 0; indexLPinLS < vLP.size(); indexLPinLS++){
                        if (std::find(vLPinLSMatch.begin(), vLPinLSMatch.end(), vLP[indexLPinLS]) != vLPinLSMatch.end()){
                            vLPonLS.push_back(vLP[indexLPinLS]);
                        }
                    }
                    // apply threshold on number of line points on a line segment
                    if (vLPonLS.size() < TH_LPONLS) {
                        vLPonLS.clear();
                        continue;
                    }
                    // keep line segment that shares the most line points
                    if (pLSMaxMatch != NULL){
                        if (vLPonLS.size() <= vLPonLSMaxMatch.size()) {
                            continue;
                        }
                    }
                    pLSMaxMatch = pLSMatch;
                    vLPonLSMaxMatch.clear();
                    vLPonLSMaxMatch.insert(vLPonLSMaxMatch.end(), vLPonLS.begin(), vLPonLS.end());
                }
                // insert into map
                mpLSMatchpLS.insert(std::map<LineSegment*,LineSegment*>::value_type(pLS, pLSMaxMatch));
                mpLSMatchpLSvLP.insert(std::map<LineSegment*,std::vector<LinePoint>>::value_type(pLS, vLPonLSMaxMatch));
            }

            // temp common points on line segments
            std::vector<LinePoint> vLPOnLineMatchedTemp;

            // check mutual max
            for (size_t indexLS = 0; indexLS < mvLSpKF[pKF].size(); indexLS++) {
                LineSegment* pLS = &(mvLSpKF[pKF][indexLS]);
                LineSegment* pLSMaxMatch = mpLSpLSMatch[pLS];

                if (pLSMaxMatch == NULL)
                    continue;

                if (mpLSMatchpLS[pLSMaxMatch] == pLS){
                    // check if same number of line points, should always be false
                    if (mpLSMatchpLSvLP[pLSMaxMatch].size() != mpLSpLSMatchvLP[pLS].size())
                        continue;
                    // check the number of line points, should always be false
                    if (mpLSpLSMatchvLP[pLS].size() < TH_LPONLS)
                        continue;

                    // check spatial order of line points
                    std::vector<LinePoint>& vLPCommon = mpLSpLSMatchvLP[pLS];
                    size_t N = vLPCommon.size();
                    // compute Kendall distance
                    double K = 0;
                    for (size_t indexLPi = 0; indexLPi < N; indexLPi++){
                        for (size_t indexLPj = indexLPi+1; indexLPj < N; indexLPj++){
                            double ui = vLPCommon[indexLPi].mmpLSuLS[pLS];
                            double uj = vLPCommon[indexLPj].mmpLSuLS[pLS];
                            double sigmai = vLPCommon[indexLPi].mmpLSuLS[pLSMaxMatch];
                            double sigmaj = vLPCommon[indexLPj].mmpLSuLS[pLSMaxMatch];
                            // count order inversion
                            if ( (ui < uj && sigmai > sigmaj) || (ui > uj && sigmai < sigmaj) ){
                                K += 1;
                            }
                        }
                    }
                    // filter out line segment matches that have too much order inversion
                    if (K > 5)
                        continue;

                    // compute number of good points
                    double Khat = 2*K/(N*(N-1));
                    double NG = computeNG(N,Khat);

                    // filter out line segment matches doesn't have enough good points
                    if (NG < N - 1.0)
                        continue;

                    std::cout << " NG K N: " << std::to_string(NG) << " " << std::to_string(K)
                              << " " << std::to_string(N) << std::endl;


                    // project each point onto other neighbour views
                    std::vector<KeyFrame*> vKFNeighbour = pKF->GetBestCovisibilityKeyFrames(10);
                    std::vector<KeyFrame*> vKFNeighbourMatch = pKFMatch->GetBestCovisibilityKeyFrames(10);
                    // merge neighbour views into one set, should have 8
                    std::vector<KeyFrame*> vKFNB;
                    for (size_t indKFN = 0; indKFN < vKFNeighbour.size(); indKFN++){
                        if (vKFNeighbour[indKFN] == pKF || vKFNeighbour[indKFN] == pKFMatch)
                            continue;
                        if (vKFNeighbour[indKFN]->isBad())
                            continue;
                        // avoid duplicate neighbour view
                        if (std::find(vKFNB.begin(),vKFNB.end(),vKFNeighbour[indKFN]) != vKFNB.end())
                            continue;
                        vKFNB.push_back(vKFNeighbour[indKFN]);
                    }
                    for (size_t indKFN = 0; indKFN < vKFNeighbourMatch.size(); indKFN++){
                        if (vKFNeighbourMatch[indKFN] == pKF || vKFNeighbourMatch[indKFN] == pKFMatch)
                            continue;
                        if (vKFNeighbourMatch[indKFN]->isBad())
                            continue;
                        // avoid duplicate neighbour view
                        if (std::find(vKFNB.begin(),vKFNB.end(),vKFNeighbourMatch[indKFN]) != vKFNB.end())
                            continue;
                        vKFNB.push_back(vKFNeighbourMatch[indKFN]);
                    }
                    if (vKFNB.size() < 2)
                        continue;

                    // check if points on segment are on segment in other kf
                    for (size_t indLP = 0; indLP < vLPCommon.size(); indLP++) {
                        LinePoint lpCommon = vLPCommon[indLP];
                        cv::Mat lp3d = cv::Mat(3, 1, CV_32F);
                        lp3d.at<float>(0) = lpCommon.mP.x;
                        lp3d.at<float>(1) = lpCommon.mP.y;
                        lp3d.at<float>(2) = lpCommon.mP.z;
                        VirtualLineSegment& vlsNB = lpCommon.mmpKFVLS.find(pKF)->second;

                        // count number of supporting views
                        unsigned int nSupportedView = 0;
                        for (size_t indKFN = 0; indKFN < vKFNB.size(); indKFN++) {
                            KeyFrame *pKFNB = vKFNB[indKFN];
                            std::vector<LineSegment> &vLSNB = mvLSpKF[pKFNB];

                            cv::Point2f lpOnKFNB = pKFMatch->ProjectPointOnCamera(lp3d);
                            // make sure projected onto image
                            if (lpOnKFNB.x < 0 || lpOnKFNB.y < 0)
                                continue;

                            // position of projection of vls end points in neighbour kf
                            cv::Point2f startVLS2fNB = pKFNB->ProjectPointOnCamera(vlsNB.mpMPStart->GetWorldPos());
                            cv::Point2f endVLS2fNB = pKFNB->ProjectPointOnCamera(vlsNB.mpMPEnd->GetWorldPos());
                            // test if the virtual line segment is in image, this should be always be false
                            if (startVLS2fNB.x < 0 || startVLS2fNB.y < 0 || endVLS2fNB.x < 0 || endVLS2fNB.y < 0)
                                continue;
                            cv::Point2f seVLS2fNB = endVLS2fNB - startVLS2fNB;
                            // filter out virtual line segments that the pair of points are too far away
                            if (cv::norm(seVLS2fNB) > MAX_VLS_LEN || cv::norm(seVLS2fNB) < MIM_VLS_LEN)
                                continue;

                            for (size_t indexLSNB = 0; indexLSNB < vLSNB.size(); indexLSNB++) {
                                LineSegment &lineNB = vLSNB[indexLSNB];
                                cv::Point2f startLSNB2f = lineNB.mStart;
                                cv::Point2f endLSNB2f = lineNB.mEnd;

                                cv::Point2f seLSNB2f = endLSNB2f - startLSNB2f;

                                // find intersection on image to match
                                // http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
                                double rxsNB = seVLS2fNB.cross(seLSNB2f);
                                // if rxs is zero
                                if (abs(rxsNB) <= std::numeric_limits<double>::epsilon())
                                    continue;

                                double tVLSNB = (startLSNB2f - startVLS2fNB).cross(seLSNB2f) / rxsNB;
                                double uLSNB = (startLSNB2f - startVLS2fNB).cross(seVLS2fNB) / rxsNB;

                                // if not intersected with detected line segment
                                if (uLSNB < 0.0 || uLSNB > 1.0)
                                    continue;
                                double vlsNBExtend = VLS_EXTEND;
                                // if not intersected with VLS with some expansion
                                if (tVLSNB < 0 - (vlsNBExtend-1) / 2 || tVLSNB > 1 + (vlsNBExtend-1) / 2)
                                    continue;

                                cv::Point2f intersectNB2f = startLSNB2f + uLSNB * seLSNB2f;

                                // check distance between projection of 3D intersection and intersection in second keyframe
                                double distLCIntersectNB = cv::norm(lpOnKFNB - intersectNB2f);
                                // the distance is small enough (in pixel)
                                if (distLCIntersectNB <= TH_LP_MATCH*2) {
                                    nSupportedView++;
                                }
                            }



//                            // find a line segment that all points projected on
//                            for (size_t indLSNB = 0; indLSNB < vLSNB.size(); indLSNB++) {
//                                LineSegment &lineNB = vLSNB[indLSNB];
//                                // line segment position for computing distance
//                                cv::Point2f startLSNB2f = lineNB.mStart;
//                                cv::Point2f endLSNB2f = lineNB.mEnd;
//                                cv::Point2f seLSNB2f = endLSNB2f - startLSNB2f;
//                                double distSE = cv::norm(seLSNB2f);
//
//                                // check distance between the point and the line segment
//                                cv::Point2f LPS = lpOnKFNB - startLSNB2f;
//                                double distLPLS;
//                                if (distSE == 0.0) {
//                                    distLPLS = cv::norm(LPS);
//                                } else {
//                                    double t = std::max(0.0, std::min(1.0, (lpOnKFNB - startLSNB2f).dot(seLSNB2f)
//                                                                           / std::pow(distSE, 2.0)));
//                                    cv::Point2f projLPLS = startLSNB2f + t * seLSNB2f;
//                                    distLPLS = cv::norm(lpOnKFNB - projLPLS);
//                                }
//                                // stop loop if a point is on segment
//                                if (distLPLS < TH_LP_MATCH) {
//                                    nSupportedView++;
//                                    continue;
//                                }
//                            }

                        }

                        if (nSupportedView >= 1){

                            std::cout << "Supporting View: " << std::to_string(nSupportedView)
                                      << " total: " << std::to_string(vKFNB.size()) << std::endl;

                            vLPOnLineMatchedTemp.push_back(lpCommon);
                            vLPOnLineMatched.push_back(lpCommon);
                        }
                    }

//                    // save the common line points to output list
//                    // TODO: might have multiple line points at same 3d position across different keyframe pairs
//                    for (size_t indexCommonLP = 0; indexCommonLP < vLPCommon.size(); indexCommonLP++){
//                        LinePoint lpCommon = vLPCommon[indexCommonLP];
//                        vLPOnLineMatched.push_back(lpCommon);
//                    }
                }
            }

            std::cout << "#Line points on matched line segments: " << std::to_string(vLPOnLineMatchedTemp.size()) << std::endl;


            // save good vls map points into result
            for (size_t indexLPOnLine = 0; indexLPOnLine < vLPOnLineMatchedTemp.size(); indexLPOnLine++){
                VirtualLineSegment& tempVLS = vLPOnLineMatchedTemp[indexLPOnLine].mmpKFVLS.find(pKF)->second;

                // increase score for the vls end points
                mpMPScore[tempVLS.mpMPStart]++;
                mpMPScore[tempVLS.mpMPEnd]++;
            }


            // save all virtual line segments matched in the two keyframes into the set
            for (size_t indexVLS = 0; indexVLS < vVLS.size(); indexVLS++) {
                VirtualLineSegment& vls = vVLS[indexVLS];
                // avoid adding VLS that have less than threshold line point matches
                if (vls.mvLPs.size() < 2)
                    continue;

                // avoid adding duplicate virtual line segment
                size_t indexVLSAll;
                // find duplicate VLS index
                // TODO: change it to be more efficient
                for (indexVLSAll = 0; indexVLSAll < vVLSAll.size(); indexVLSAll++) {
                    if (vls == vVLSAll[indexVLSAll])
                        break;
                }
                if (indexVLSAll < vVLSAll.size()) {
                    VirtualLineSegment &vlsRef = vVLSAll[indexVLSAll];
                    for (size_t indexVLSLP = 0; indexVLSLP < vls.mvLPs.size(); indexVLSLP++){
                        LinePoint& lp = vls.mvLPs[indexVLSLP];
                        // find existing line point in the VLS's list
                        // TODO: change it to be more efficient
                        size_t indexVLSRefLP;
                        for (indexVLSRefLP = 0; indexVLSRefLP < vlsRef.mvLPs.size(); indexVLSRefLP++) {
                            if (lp == vlsRef.mvLPs[indexVLSLP])
                                break;
                        }
                        if (indexVLSRefLP < vlsRef.mvLPs.size()) {
                            LinePoint &lpRef = vlsRef.mvLPs[indexVLSRefLP];
                            std::map<LineSegment*,double>::iterator itLS;
                            for (itLS = lp.mmpLSuLS.begin(); itLS != lp.mmpLSuLS.end(); itLS++){
                                lpRef.addRefLineSegment((*itLS).first, (*itLS).second);
                            }
                        } else {
                            vlsRef.mvLPs.push_back(lp);
                        }
                    }
                } else {
                    vVLSAll.push_back(vls);
                }
            }
        }

        // get line points out of virtual line segments
        int numTotalLP = 0;
        for (std::vector<VirtualLineSegment>::iterator it = vVLSAll.begin(); it != vVLSAll.end(); it++){
            for (size_t indexLP = 0; indexLP < (*it).mvLPs.size(); indexLP++){
                numTotalLP++;
                LinePoint& currLP = (*it).mvLPs[indexLP];
                if (currLP.mmpLSuLS.size() >= 3) {
                    vLPOnLine.push_back(currLP);
                }

            }
        }

        // get map points that have good score
        std::vector<LinePoint> vLPScoreMapPoint;
        std::vector<MapPoint*> vpMPBestScore;
        for (auto it = mpMPScore.begin(); it != mpMPScore.end(); it++){
            if (it->second >= 2){
                vpMPBestScore.push_back(it->first);
                LinePoint lpMapPoint(cv::Point3f(it->first->GetWorldPos()));
                vLPScoreMapPoint.push_back(lpMapPoint);
            }
        }

        std::vector<LinePoint> vLPScore;
        for (auto itLP = vLPOnLineMatched.begin(); itLP != vLPOnLineMatched.end(); itLP++){
            for (auto itVLS = itLP->mmpKFVLS.begin(); itVLS != itLP->mmpKFVLS.end(); itVLS++){
                MapPoint* pMPs = itVLS->second.mpMPStart;
                MapPoint* pMPe = itVLS->second.mpMPEnd;

                if (std::find(vpMPBestScore.begin(),vpMPBestScore.end(),pMPs) == vpMPBestScore.end()
                    || std::find(vpMPBestScore.begin(),vpMPBestScore.end(),pMPe) == vpMPBestScore.end()){
                    break;
                }
                vLPScore.push_back(*itLP);
            }
        }

        // all map points
        std::vector<LinePoint> vLPAllMapPoint;
        for (auto it = mpMPScore.begin(); it != mpMPScore.end(); it++){
            LinePoint lpMapPoint(cv::Point3f(it->first->GetWorldPos()));
            vLPAllMapPoint.push_back(lpMapPoint);
        }

        std::cout << "#Total line points:" << std::to_string(numTotalLP) << std::endl;

        std::cout << "#Multiple referenced line points:" << std::to_string(vLPOnLine.size()) << std::endl;

        std::cout << "#Filtered line points:" << std::to_string(vLPOnLineMatched.size()) << std::endl;

        std::cout << "#Scored map points:" << std::to_string(vLPScoreMapPoint.size()) << std::endl;

        std::cout << "#Scored line points:" << std::to_string(vLPScore.size()) << std::endl;

        std::cout << "#All map points:" << std::to_string(vLPAllMapPoint.size()) << std::endl;


        // put vls end points into return value
//        vLPOnLineMatched.insert(vLPOnLineMatched.end(), vLPVLSMapPoint.begin(), vLPVLSMapPoint.end());

        saveLinePointToFile(vLPScore, "line_scoreLP.obj");

        saveLinePointToFile(vLPScoreMapPoint, "line_scoreMP.obj");

        saveLinePointToFile(vLPAllMapPoint, "line_allmappoints.obj");

        return vLPOnLineMatched;
    }

    void Modeler::saveLinePointToFile(std::vector<LinePoint>& vPOnLine, const std::string & strFileName){

        std::ofstream fileOut(strFileName.c_str(), std::ios::out);
        if(!fileOut){
            std::cerr << "Failed to save points on line" << std::endl;
            return;
        }

        for (size_t indexLP = 0; indexLP < vPOnLine.size(); indexLP++){
            fileOut << vPOnLine[indexLP].toObj() << "\n";
        }

        fileOut.flush();
        fileOut.close();

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
                cv::Point2f pt = line.mpRefKF->ProjectPointOnCamera(pMP->GetWorldPos());
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
