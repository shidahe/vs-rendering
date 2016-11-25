//
// Created by theia on 11/24/16.
//

#ifndef ORB_SLAM2_MODELER_H
#define ORB_SLAM2_MODELER_H

#include "Modeler/SFMTranscriptInterface_ORBSLAM.h"
#include "Modeler/SFMTranscriptInterface_Delaunay.h"

#include <mutex>

class Modeler{
public:
    Modeler(){
        //CARV
        mAlgInterface.setAlgorithmRef(& mObjAlgorithm);
        mAlgInterface.setTranscriptRef(mTranscriptInterface.getTranscriptRef());
        mAlgInterface.rewind();
        mbFirstKeyFrame = true;
    }

    // This avoid that two transcript entries are created simultaneously in separate threads
    std::mutex mMutexWriteTranscript;

    // CARV interface
    SFMTranscriptInterface_ORBSLAM mTranscriptInterface; // An interface to a transcript / log of the map's work.
    //CARV runner instance
    dlovi::FreespaceDelaunayAlgorithm mObjAlgorithm;
    SFMTranscriptInterface_Delaunay mAlgInterface; // An encapsulation of the interface between the transcript and the surface inferring algorithm.
    bool mbFirstKeyFrame;

};

#endif //ORB_SLAM2_MODELVIEWER_H
