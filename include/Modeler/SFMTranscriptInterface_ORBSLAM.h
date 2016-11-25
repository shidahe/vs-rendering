#ifndef __SFMTRANSCRIPTINTERFACE_ORBSLAM_H
#define __SFMTRANSCRIPTINTERFACE_ORBSLAM_H

#include "Modeler/SFMTranscript.h"
#include <string>
#include <set>
#include <map>
#include "MapPoint.h"
#include "KeyFrame.h"

// class ORB_SLAM2::MapPoint;
// class ORB_SLAM2::KeyFrame;

typedef ORB_SLAM2::MapPoint MapPoint;
typedef ORB_SLAM2::KeyFrame KeyFrame;

class SFMTranscriptInterface_ORBSLAM;
class SFMTranscriptInterface_ORBSLAM{
public:
    // Constructors and Destructors
    SFMTranscriptInterface_ORBSLAM();
    ~SFMTranscriptInterface_ORBSLAM();

    // Getters
    dlovi::compvis::SFMTranscript * getTranscriptRef();

    // Public Methods
    void addResetEntry();
    void addPointDeletionEntry(MapPoint *p);
    void addVisibilityRayInsertionEntry(KeyFrame *k, MapPoint *p);
    void addVisibilityRayDeletionEntry(KeyFrame *k, MapPoint *p);
    void addFirstKeyFrameInsertionEntry(KeyFrame *k);
    void addKeyFrameInsertionEntry(KeyFrame *k);
    //void addBundleAdjustmentEntry(std::set<KeyFrame *> & sAdjustSet, std::set<MapPoint *> & sMapPoints);
    void writeToFile(const std::string & strFileName) const;
    void suppressBundleAdjustmentLogging();
    void unsuppressBundleAdjustmentLogging();
    void suppressRefindLogging();
    void unsuppressRefindLogging();
private:
    // Member Variables
    dlovi::compvis::SFMTranscript m_SFMTranscript;
    bool m_bSuppressRefindLogging;
    bool m_bSuppressBundleAdjustmentLogging;

    std::map<MapPoint *, int> m_mMapPoint_Index;	// TODO: possibly change to hashed?
    std::map<KeyFrame *, int> m_mKeyFrame_Index;	// TODO: possibly change to hashed?
};

#endif
