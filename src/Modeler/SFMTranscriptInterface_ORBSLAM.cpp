#ifndef __SFMTRANSCRIPTINTERFACE_ORBSLAM_CPP
#define __SFMTRANSCRIPTINTERFACE_ORBSLAM_CPP

#include "Modeler/SFMTranscriptInterface_ORBSLAM.h"
#include "Modeler/Exception.h"
#include <sstream>
#include "MapPoint.h"
#include "KeyFrame.h"
#include "Map.h"
#include "Modeler/Matrix.h"
// #include "opencv2/opencv.hpp"

using namespace std;
using namespace dlovi;

// Constructors and Destructors

SFMTranscriptInterface_ORBSLAM::SFMTranscriptInterface_ORBSLAM(){
    try{
        // No suppression of logging by default
        m_bSuppressBundleAdjustmentLogging = false;
        m_bSuppressRefindLogging = false;

        // Write transcript header
        m_SFMTranscript.addLine("SFM Transcript: ORBSLAM");
        m_SFMTranscript.addLine("*** BODY ***");
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_ORBSLAM", "SFMTranscriptInterface_ORBSLAM"); cerr << ex2.what() << endl; //ex2.raise();
    }
}

SFMTranscriptInterface_ORBSLAM::~SFMTranscriptInterface_ORBSLAM(){
    // NOP
}

// Getters
dlovi::compvis::SFMTranscript * SFMTranscriptInterface_ORBSLAM::getTranscriptRef(){
    try{
        return & m_SFMTranscript;
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_ORBSLAM", "getTranscriptRef"); cerr << ex2.what() << endl; //ex2.raise();
    }
}

// Public Methods

void SFMTranscriptInterface_ORBSLAM::addResetEntry(){
    try{
        m_SFMTranscript.addLine("reset");
        // Reset the pointer -> index maps
        m_mMapPoint_Index.clear();
        m_mKeyFrame_Index.clear();
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_ORBSLAM", "addResetEntry"); cerr << ex2.what() << endl; //ex2.raise();
    }
}

void SFMTranscriptInterface_ORBSLAM::addPointDeletionEntry(MapPoint *p){
    try{
        std::stringstream ssTmp;

        // Set nPointIndex based on argument.
        std::map<MapPoint *, int>::iterator itMapPoint = m_mMapPoint_Index.find(p);
        if(itMapPoint == m_mMapPoint_Index.end()) // The logger has no record of this point?  That's bad.
            throw dlovi::Exception("Could not compute MapPoint index: no record.");
        int nPointIndex = itMapPoint->second;

        ssTmp << "del point: " << nPointIndex;
        m_SFMTranscript.addLine(ssTmp.str());
    }

    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_ORBSLAM", "addPointDeletionEntry"); cerr << ex2.what() << endl; //ex2.raise();
    }
}

void SFMTranscriptInterface_ORBSLAM::addVisibilityRayInsertionEntry(KeyFrame *k, MapPoint *p){
    try{
        if(! m_bSuppressRefindLogging){
            std::stringstream ssTmp;

            // Set nCamIndex and nPointIndex based on arguments.
            std::map<KeyFrame *, int>::iterator itKeyFrame = m_mKeyFrame_Index.find(k);
            if(itKeyFrame == m_mKeyFrame_Index.end()) // The logger has no record of this KF?  That's bad.
                throw dlovi::Exception("Could not compute KeyFrame index: no record.");
            std::map<MapPoint *, int>::iterator itMapPoint = m_mMapPoint_Index.find(p);
            if(itMapPoint == m_mMapPoint_Index.end()) // The logger has no record of this point?  That's bad.
                throw dlovi::Exception("Could not compute MapPoint index: no record.");

            int nCamIndex = itKeyFrame->second;
            int nPointIndex = itMapPoint->second;

            ssTmp << "observation: " << nCamIndex << ", " << nPointIndex;
            m_SFMTranscript.addLine(ssTmp.str());
        }
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_ORBSLAM", "addVisibilityRayInsertionEntry"); cerr << ex2.what() << endl; //ex2.raise();
        assert(0); // TODO: remove me
    }
}

void SFMTranscriptInterface_ORBSLAM::addVisibilityRayDeletionEntry(KeyFrame *k, MapPoint *p){
    try{
        std::stringstream ssTmp;

        // Set nCamIndex and nPointIndex based on arguments.
        std::map<KeyFrame *, int>::iterator itKeyFrame = m_mKeyFrame_Index.find(k);
        if(itKeyFrame == m_mKeyFrame_Index.end()) // The logger has no record of this KF?  That's bad.
            throw dlovi::Exception("Could not compute KeyFrame index: no record.");
        std::map<MapPoint *, int>::iterator itMapPoint = m_mMapPoint_Index.find(p);
        if(itMapPoint == m_mMapPoint_Index.end()) // The logger has no record of this point?  That's bad.
            throw dlovi::Exception("Could not compute MapPoint index: no record.");

        int nCamIndex = itKeyFrame->second;
        int nPointIndex = itMapPoint->second;

        ssTmp << "del observation: " << nCamIndex << ", " << nPointIndex;
        m_SFMTranscript.addLine(ssTmp.str());
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_ORBSLAM", "addVisibilityRayDeletionEntry"); cerr << ex2.what() << endl; //ex2.raise();
        assert(0); // TODO: remove me
    }
}

void SFMTranscriptInterface_ORBSLAM::addFirstKeyFrameInsertionEntry(KeyFrame *k){
    try{
        std::stringstream ssTmp;
        dlovi::Matrix matNewCam(3, 1);
        dlovi::Matrix matNewPoint(3, 1);
        int nPointIndex, nCamIndex;

        if(m_mKeyFrame_Index.count(k) > 0)
            throw dlovi::Exception("KeyFrame already has a record.  Double addition.");

        // // TODO: Instead of inverting the whole transform, we should be able to just use the negative translation.
        // GetPose instead GetPoseInverse, seems camera position need to be inversed
        cv::Mat se3WfromC = k->GetPose();
//        se3WfromC = se3WfromC.inv();
        matNewCam(0) = se3WfromC.at<float>(0,3);
        matNewCam(1) = se3WfromC.at<float>(1,3);
        matNewCam(2) = se3WfromC.at<float>(2,3);
        ssTmp << "new cam: [" << matNewCam(0) << "; " << matNewCam(1) << "; " << matNewCam(2) << "] {";
        m_SFMTranscript.addLine(ssTmp.str()); ssTmp.str("");

        // Add a record of the new camera to internal map.
        nCamIndex = m_mKeyFrame_Index.size();
        m_mKeyFrame_Index[k] = nCamIndex;

        std::set<MapPoint*> mvpMapPoints = k->GetMapPoints();
        // Process new points and visibility information in this KF
        for(std::set<MapPoint*>::iterator it = mvpMapPoints.begin(); it != mvpMapPoints.end(); it++){
            MapPoint * point = *it;

            if(m_mMapPoint_Index.count(point) == 0){
                // It's a new point:
                cv::Mat mWorldPos = point->GetWorldPos();
                matNewPoint(0) = mWorldPos.at<float>(0);
                matNewPoint(1) = mWorldPos.at<float>(1);
                matNewPoint(2) = mWorldPos.at<float>(2);

                if (point->isBad())
                    continue;

                ssTmp << "new point: [" << matNewPoint(0) << "; " << matNewPoint(1) << "; " << matNewPoint(2) << "]";
                // Append this point's vis list with special handling.  (Point initialized from epipolar search: > 1 KF, but only 1 KF in our internal structures.)
                ssTmp << ", 0"; // KF 0 observed it.
                m_SFMTranscript.addLine(ssTmp.str()); ssTmp.str("");

                // Add a record of the new point to internal map.
                nPointIndex = m_mMapPoint_Index.size();
                m_mMapPoint_Index[point] = nPointIndex;
            }
            else
                throw dlovi::Exception("The FIRST KF observed a point that was already added."); // That's bad!  Points are only added through KF-addition.
        }

        // Close this new-KF entry in the transcript
        m_SFMTranscript.addLine("}");
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_ORBSLAM", "addFirstKeyFrameInsertionEntry"); cerr << ex2.what() << endl; //ex2.raise();
    }
}

void SFMTranscriptInterface_ORBSLAM::addKeyFrameInsertionEntry(KeyFrame *k){
    try{
        std::stringstream ssTmp;
        dlovi::Matrix matNewCam(3, 1);
        dlovi::Matrix matNewPoint(3, 1);
        int nPointIndex, nCamIndex;

        if(m_mKeyFrame_Index.count(k) > 0)
            throw dlovi::Exception("KeyFrame already has a record.  Double addition.");

        // TODO: Instead of inverting the whole transform, we should be able to just use the negative translation.
        // GetPose instead GetPoseInverse, seems camera position need to be inversed
        cv::Mat se3WfromC = k->GetPose();
        se3WfromC = se3WfromC.inv();
        matNewCam(0) = se3WfromC.at<float>(0,3);
        matNewCam(1) = se3WfromC.at<float>(1,3);
        matNewCam(2) = se3WfromC.at<float>(2,3);
        ssTmp << "new cam: [" << matNewCam(0) << "; " << matNewCam(1) << "; " << matNewCam(2) << "] {";
        m_SFMTranscript.addLine(ssTmp.str()); ssTmp.str("");

        // Add a record of the new camera to internal map.
        nCamIndex = m_mKeyFrame_Index.size();
        m_mKeyFrame_Index[k] = nCamIndex;

        // Process new points and visibility information in this KF
        std::set<int> sVisListExcludingNewPoints;
        std::set<MapPoint*> mvpMapPoints = k->GetMapPoints();
        for(std::set<MapPoint*>::iterator it = mvpMapPoints.begin(); it != mvpMapPoints.end(); it++){
            MapPoint * point = *it;

            if(m_mMapPoint_Index.count(point) == 0){

                // check if it has valid observation first
                std::map<KeyFrame*, size_t> mObservations = point->GetObservations();
                bool hasObservation = false;
                for(std::map<KeyFrame *,size_t>::iterator it2 = mObservations.begin(); it2 != mObservations.end(); it2++){
                    if(m_mKeyFrame_Index.count(it2->first) != 0) {
                        hasObservation = true;
                        break;
                    }
                }

                if (hasObservation) {
                    // It's a new point:
                    cv::Mat mWorldPos = point->GetWorldPos();
                    matNewPoint(0) = mWorldPos.at<float>(0);
                    matNewPoint(1) = mWorldPos.at<float>(1);
                    matNewPoint(2) = mWorldPos.at<float>(2);

                    if (point->isBad())
                        continue;

                    ssTmp << "new point: [" << matNewPoint(0) << "; " << matNewPoint(1) << "; " << matNewPoint(2)
                          << "]";
                    // Append this point's vis list.  (Point initialized from epipolar search: > 1 KF)
                    for (std::map<KeyFrame *, size_t>::iterator it2 = mObservations.begin();
                         it2 != mObservations.end(); it2++) {
                        if (m_mKeyFrame_Index.count(it2->first) != 0) {
                            ssTmp << ", " << m_mKeyFrame_Index[it2->first];
                        }
                    }

                    m_SFMTranscript.addLine(ssTmp.str()); ssTmp.str("");

                    // Add a record of the new point to internal map.
                    nPointIndex = m_mMapPoint_Index.size();
                    m_mMapPoint_Index[point] = nPointIndex;
                }
            }
            else{
                // It's not a new point:
                sVisListExcludingNewPoints.insert(m_mMapPoint_Index[point]); // To be added after the new points in the following loop
            }
        }

        // Log all the visibility-ray observations for this KF excluding the newly added points
        for(std::set<int>::iterator it = sVisListExcludingNewPoints.begin(); it != sVisListExcludingNewPoints.end(); it++){
            ssTmp << "observation: " << *it;
            m_SFMTranscript.addLine(ssTmp.str()); ssTmp.str("");
        }

        // Close this new-KF entry in the transcript
        m_SFMTranscript.addLine("}");
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_ORBSLAM", "addKeyFrameInsertionEntry"); cerr << ex2.what() << endl; //ex2.raise();
    }
}

// void SFMTranscriptInterface_ORBSLAM::addBundleAdjustmentEntry(set<KeyFrame *> & sAdjustSet, set<MapPoint *> & sMapPoints){
//   try{
//     if(! m_bSuppressBundleAdjustmentLogging){
//       std::stringstream ssTmp;
//       int nPointIndex, nCamIndex;

//       // TODO: stub

//       m_SFMTranscript.addLine("bundle {");

//       // Log point-move entries
//       for(set<MapPoint *>::iterator it = sMapPoints.begin(); it != sMapPoints.end(); it++){
//         if(m_mMapPoint_Index.count(*it) == 0)
//           throw dlovi::Exception("Could not compute MapPoint index: no record.");
//         nPointIndex = m_mMapPoint_Index[*it];
//         ssTmp << "move point: " << nPointIndex << ", [" << (*it)->v3WorldPos[0] << "; " << (*it)->v3WorldPos[1] << "; " << (*it)->v3WorldPos[2] << "]";
//         m_SFMTranscript.addLine(ssTmp.str()); ssTmp.str("");
//       }

//       // Log KF-move entries
//       for(set<KeyFrame *>::iterator it = sAdjustSet.begin(); it != sAdjustSet.end(); it++){
//         if(m_mKeyFrame_Index.count(*it) == 0)
//           throw dlovi::Exception("Could not compute KeyFrame index: no record.");
//         nCamIndex = m_mKeyFrame_Index[*it];

//         // TODO: Instead of inverting the whole transform, we should be able to just use the negative translation.
//         SE3<> se3WfromC = (*it)->se3CfromW.inverse();
//         ssTmp << "move cam: " << nCamIndex << ", [" << se3WfromC.get_translation()[0] << "; " << se3WfromC.get_translation()[1] << "; "
//             << se3WfromC.get_translation()[2] << "]";
//         m_SFMTranscript.addLine(ssTmp.str()); ssTmp.str("");
//       }

//       // Close this bundle-adjust entry in the transcript
//       m_SFMTranscript.addLine("}");
//     }
//   }
//   catch(std::exception & ex){
//     dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_ORBSLAM", "addBundleAdjustmentEntry"); cerr << ex2.what() << endl; //ex2.raise();
//   }
// }

void SFMTranscriptInterface_ORBSLAM::writeToFile(const std::string & strFileName) const{
    try{
        m_SFMTranscript.writeToFile(strFileName);
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_ORBSLAM", "writeToFile"); cerr << ex2.what() << endl; //ex2.raise();
    }
}

void SFMTranscriptInterface_ORBSLAM::suppressBundleAdjustmentLogging(){
    try{
        m_bSuppressBundleAdjustmentLogging = true;
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_ORBSLAM", "suppressBundleAdjustmentLogging"); cerr << ex2.what() << endl; //ex2.raise();
    }
}

void SFMTranscriptInterface_ORBSLAM::unsuppressBundleAdjustmentLogging(){
    try{
        m_bSuppressBundleAdjustmentLogging = false;
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_ORBSLAM", "unsuppressBundleAdjustmentLogging"); cerr << ex2.what() << endl; //ex2.raise();
    }
}

void SFMTranscriptInterface_ORBSLAM::suppressRefindLogging(){
    try{
        m_bSuppressRefindLogging = true;
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_ORBSLAM", "suppressRefindLogging"); cerr << ex2.what() << endl; //ex2.raise();
    }
}

void SFMTranscriptInterface_ORBSLAM::unsuppressRefindLogging(){
    try{
        m_bSuppressRefindLogging = false;
    }
    catch(std::exception & ex){
        dlovi::Exception ex2(ex.what()); ex2.tag("SFMTranscriptInterface_ORBSLAM", "unsuppressRefindLogging"); cerr << ex2.what() << endl; //ex2.raise();
    }
}


#endif
