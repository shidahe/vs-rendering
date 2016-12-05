
#ifndef __MODELER_H
#define __MODELER_H

#include "Modeler/SFMTranscriptInterface_ORBSLAM.h"
#include "Modeler/SFMTranscriptInterface_Delaunay.h"

#include <mutex>

namespace ORB_SLAM2 {

    class Modeler {
    public:
        Modeler() {
            //CARV
            mAlgInterface.setAlgorithmRef(&mObjAlgorithm);
            mAlgInterface.setTranscriptRef(mTranscriptInterface.getTranscriptRef());
            mAlgInterface.rewind();
            mbFirstKeyFrame = true;
        }

        // This avoid that two transcript entries are created simultaneously in separate threads
        std::mutex mMutexWriteTranscript;

        //CARV interface
        SFMTranscriptInterface_ORBSLAM mTranscriptInterface; // An interface to a transcript / log of the map's work.
        //CARV runner instance
        dlovi::FreespaceDelaunayAlgorithm mObjAlgorithm;
        SFMTranscriptInterface_Delaunay mAlgInterface; // An encapsulation of the interface between the transcript and the surface inferring algorithm.
        bool mbFirstKeyFrame;

    };
}

#endif //__MODELVIEWER_H
