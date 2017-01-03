//
// Created by shida on 06/12/16.
//

#include "Modeler/ModelDrawer.h"
#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>

namespace ORB_SLAM2
{

    ModelDrawer::ModelDrawer():mbModelUpdateRequested(false), mbModelUpdateDone(true)
    {
    }

    void ModelDrawer::DrawModel(float light_x, float light_y , float light_z)
    {
        // select 4 KFs
        int numKFs = 1;
        vector<pair<cv::Mat,TextureFrame>> imAndTexFrame = mpModeler->GetTextures(numKFs);

        if (imAndTexFrame.size() >= 1) {
//            static unsigned int frameTex[4] = {0, 0, 0, 0};
            static unsigned int frameTex[1] = {0};
            if (!frameTex[0])
                glGenTextures(numKFs, frameTex);

            for (int i = 0; i < numKFs; i++) {
                cv::Size s = imAndTexFrame[i].first.size();
                glBindTexture(GL_TEXTURE_2D, frameTex[i]);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
                             s.height, s.width, 0,
                             GL_RGB,
                             GL_UNSIGNED_BYTE,
                             imAndTexFrame[i].first.data);
            }

            UpdateModel();

//        std::ofstream pointsOutput("drawingPoints.txt");

//            glEnable(GL_POINT_SMOOTH);
//            glEnable(GL_LINE_SMOOTH);
            glEnable(GL_TEXTURE_2D);
//            glBindTexture(GL_TEXTURE_2D, frameTex[0]);

            glPointSize(3);
            glBegin(GL_POINTS);
            glColor3f(0.5, 0.5, 0.5);

            for (size_t i = 0; i < GetPoints().size(); i++) {
                glVertex3d(GetPoints()[i](0), GetPoints()[i](1), GetPoints()[i](2));
//            pointsOutput << GetPoints()[i](0) << " " << GetPoints()[i](1) << " " << GetPoints()[i](2) << endl;
            }
            glEnd();
//        pointsOutput.close();

//        GLfloat light_position[] = { light_x, light_y, light_z, 0.0 };
//            GLfloat light_position[] = {10, 10, 10, 0};
//
//            glShadeModel(GL_FLAT);
//
//            GLfloat material_diffuse[] = {0, 0, 1, 1};
//            GLfloat material_specular[] = {1, 1, 1, 1};
//            GLfloat material_shininess[] = {100};
//            glMaterialfv(GL_FRONT, GL_DIFFUSE, material_diffuse);
//            glMaterialfv(GL_FRONT, GL_SPECULAR, material_specular);
//            glMaterialfv(GL_FRONT, GL_SHININESS, material_shininess);
//
//            glLightfv(GL_LIGHT0, GL_POSITION, light_position);
//
//            glEnable(GL_LIGHTING);
//            glEnable(GL_LIGHT0);

            glBegin(GL_TRIANGLES);
            for (list<dlovi::Matrix>::const_iterator it = GetTris().begin(); it != GetTris().end(); it++) {

                dlovi::Matrix point0 = GetPoints()[(*it)(0)];
                dlovi::Matrix point1 = GetPoints()[(*it)(1)];
                dlovi::Matrix point2 = GetPoints()[(*it)(2)];

                dlovi::Matrix edge10 = point1 - point0;
                dlovi::Matrix edge20 = point2 - point0;

                dlovi::Matrix normal = edge20.cross(edge10);
                normal = normal / normal.norm();

                TextureFrame tex = imAndTexFrame[0].second;
                vector<float> uv0 = tex.GetTexCoordinate(point0(0),point0(1),point0(2));
                vector<float> uv1 = tex.GetTexCoordinate(point1(0),point1(1),point1(2));
                vector<float> uv2 = tex.GetTexCoordinate(point2(0),point2(1),point2(2));

                glNormal3d(normal(0), normal(1), normal(2));

//                if (uv0[0] > 0 && uv0[0] < 1 && uv0[1] > 0 && uv0[1] < 1)
//                    glTexCoord2f(uv0[0],uv0[1]);
//                glTexCoord3d(point0(0),point0(1),point0(2));
                glTexCoord2d(point0(0),point0(1));
                glVertex3d(point0(0),point0(1),point0(2));

//                if (uv1[0] > 0 && uv1[0] < 1 && uv1[1] > 0 && uv1[1] < 1)
//                    glTexCoord2f(uv1[0],uv1[1]);
//                glTexCoord3d(point1(0),point1(1),point1(2));
                glTexCoord2d(point1(0),point1(1));
                glVertex3d(point1(0),point1(1)  ,point1(2));

//                if (uv2[0] > 0 && uv2[0] < 1 && uv2[1] > 0 && uv2[1] < 1)
//                    glTexCoord2f(uv2[0],uv2[1]);
//                glTexCoord3d(point2(0),point2(1),point2(2));
                glTexCoord2d(point2(0),point2(1));
                glVertex3d(point2(0),point2(1),point2(2));
            }
            glEnd();

//            glDisable(GL_LIGHTING);
            glDisable(GL_TEXTURE_2D);
        }

    }


    void ModelDrawer::UpdateModel()
    {
        if(mbModelUpdateRequested && ! mbModelUpdateDone)
            return;

        if(mbModelUpdateRequested && mbModelUpdateDone){
            mModel = mUpdatedModel;
            mbModelUpdateRequested = false;
            return;
        }

        mbModelUpdateDone = false;
        mbModelUpdateRequested = true; // implicitly signals SurfaceInferer thread which is polling
    }

    void ModelDrawer::SetUpdatedModel(const vector<dlovi::Matrix> & modelPoints, const list<dlovi::Matrix> & modelTris)
    {
        mUpdatedModel.first = modelPoints;
        mUpdatedModel.second = modelTris;
    }

    vector<dlovi::Matrix> & ModelDrawer::GetPoints()
    {
        return mModel.first;
    }

    list<dlovi::Matrix> & ModelDrawer::GetTris()
    {
        return mModel.second;
    }

    void ModelDrawer::MarkUpdateDone()
    {
        mbModelUpdateDone = true;
    }

    bool ModelDrawer::UpdateRequested()
    {
        return mbModelUpdateRequested;
    }

    bool ModelDrawer::UpdateDone()
    {
        return mbModelUpdateDone;
    }

    void ModelDrawer::SetModeler(Modeler* pModeler)
    {
        mpModeler = pModeler;
    }

} //namespace ORB_SLAM
