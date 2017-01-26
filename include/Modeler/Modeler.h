
#ifndef __MODELER_H
#define __MODELER_H

#include <mutex>

#include "Modeler/SFMTranscriptInterface_ORBSLAM.h"
#include "Modeler/SFMTranscriptInterface_Delaunay.h"
#include "Modeler/ModelDrawer.h"
#include "Modeler/TextureFrame.h"

#include "Thirdparty/EDLines/LS.h"

namespace ORB_SLAM2 {

    class Tracking;
    class LoopClosing;
    class LocalMapping;
    class KeyFrame;
    class Frame;
    class ModelDrawer;


    class LineSegment {
    public:
        LineSegment(LS* pLS){
            mStart.x = pLS->sx;
            mStart.y = pLS->sy;
            mEnd.x = pLS->ex;
            mEnd.y = pLS->ey;
            mpRefKF = NULL;
        }

        map<MapPoint*, float> mmpMPProj;
        KeyFrame* mpRefKF;
        cv::Point2f mStart;
        cv::Point2f mEnd;
    };

    class Modeler {
    public:
        Modeler(ModelDrawer* pModelDrawer);

        void SetLoopCloser(LoopClosing* pLoopCloser);

        void SetLocalMapper(LocalMapping* pLocalMapper);

        void SetTracker(Tracking* pTracker);

        // Main function
        void Run();

        void UpdateModelDrawer();
        bool CheckNewTranscriptEntry();
        void RunRemainder();

        void AddPointsOnLineSegments();
        void DetectLineSegmentsLater(KeyFrame* pKF);
        std::vector<LineSegment> DetectLineSegments(cv::Mat& im);
        std::vector<cv::Point3f> GetPointsOnLineSegments(KeyFrame* pKF);

        void AddKeyFrameEntry(KeyFrame* pKF);
        void AddDeletePointEntry(MapPoint* pMP);
        void AddDeleteObservationEntry(KeyFrame* pKF, MapPoint* pMP);
        void AddAdjustmentEntry(std::set<KeyFrame*> & sAdjustSet, std::set<MapPoint*> & sMapPoints);

        // Thread Synch
        void RequestReset();
        void RequestFinish();
        bool isFinished();

        void AddTexture(KeyFrame* pKF);
        void AddTexture(Frame* pF);
        void AddFrameImage(const long unsigned int &frameID, const cv::Mat &im);

        // get last n keyframes for texturing
        std::vector<pair<cv::Mat,TextureFrame>> GetTextures(int n);

        // get last detected lines and cooresponding image
        cv::Mat GetImageWithLines();

    public:
        void ResetIfRequested();
        bool mbResetRequested;
        std::mutex mMutexReset;

        bool CheckFinish();
        void SetFinish();
        bool mbFinishRequested;
        bool mbFinished;
        std::mutex mMutexFinish;

        Tracking* mpTracker;
        LocalMapping* mpLocalMapper;
        LoopClosing* mpLoopCloser;

        ModelDrawer* mpModelDrawer;

        // This avoid that two transcript entries are created simultaneously in separate threads
        std::mutex mMutexTranscript;

        //number of lines in transcript last time checked
        int mnLastNumLines;

        //CARV interface
        SFMTranscriptInterface_ORBSLAM mTranscriptInterface; // An interface to a transcript / log of the map's work.
        //CARV runner instance
        dlovi::FreespaceDelaunayAlgorithm mObjAlgorithm;
        SFMTranscriptInterface_Delaunay mAlgInterface; // An encapsulation of the interface between the transcript and the surface inferring algorithm.
        bool mbFirstKeyFrame;

        //queue for the keyframes used to texture the model, keyframe mnFrameId
        std::deque<TextureFrame> mdTextureQueue;
        size_t mnMaxTextureQueueSize;
        std::mutex mMutexTexture;

        //queue for the frames recieved, pair<mnID,image>
        std::map<long unsigned int, cv::Mat> mmFrameQueue;
        size_t mnMaxFrameQueueSize;
        std::mutex mMutexFrame;

        //queue for the texture frames used to detect lines
        std::deque<KeyFrame*> mdToLinesQueue;
        size_t mnMaxToLinesQueueSize;
        std::mutex mMutexToLines;

        //lines detected
        std::vector<LineSegment> mvLines;
        cv::Mat mImLines;
        std::mutex mMutexLines;

    };
}


#endif //__MODELVIEWER_H
