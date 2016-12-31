//
// Created by shida on 06/12/16.
//

#ifndef __MODELDRAWER_H
#define __MODELDRAWER_H

#include"Map.h"
#include"MapPoint.h"
#include"KeyFrame.h"
#include<pangolin/pangolin.h>

#include<mutex>

#include "Frame.h"
#include "Tracking.h"
#include <map>
#include <list>
#include <deque>
#include "Modeler/Matrix.h"


namespace ORB_SLAM2
{

    class KeyFrame;

    class ModelDrawer
    {
    public:
        ModelDrawer();

        void DrawModel(float light_x, float light_y , float light_z);

        void UpdateModel();
        void SetUpdatedModel(const vector<dlovi::Matrix> & modelPoints, const list<dlovi::Matrix> & modelTris);

        void MarkUpdateDone();
        bool UpdateRequested();
        bool UpdateDone();

        vector<dlovi::Matrix> & GetPoints();
        list<dlovi::Matrix> & GetTris();

    private:

        bool mbModelUpdateRequested;
        bool mbModelUpdateDone;

        std::pair<vector<dlovi::Matrix>, list<dlovi::Matrix>> mModel;
        std::pair<vector<dlovi::Matrix>, list<dlovi::Matrix>> mUpdatedModel;

    };

} //namespace ORB_SLAM

#endif //__MODELDRAWER_H
