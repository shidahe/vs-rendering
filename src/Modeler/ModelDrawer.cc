//
// Created by shida on 06/12/16.
//

#include <Eigen/Geometry>
#include "Modeler/ModelDrawer.h"

namespace ORB_SLAM2
{
    ModelDrawer::ModelDrawer():mbModelUpdateRequested(false), mbModelUpdateDone(true)
    {
        target.setZero();
    }

    void ModelDrawer::DrawModel(bool bRGB)
    {
        // select 10 KFs
        int numKFs = 10;
        vector<pair<cv::Mat,TextureFrame>> imAndTexFrame = mpModeler->GetTextures(numKFs);

        UpdateModel();

        if ((int)imAndTexFrame.size() >= numKFs) {

//            static unsigned int frameTex[4] = {0,0,0,0};
            static unsigned int frameTex = 0;

            if (!frameTex)
                glGenTextures(numKFs, &frameTex);

            cv::Size imSize = imAndTexFrame[0].first.size();

            cv::Mat mat_array[numKFs];
            int count = 0;
            for (int i = 0; i < numKFs; i++) {
                if (imAndTexFrame[i].first.channels() == 3) {
                    mat_array[count] = imAndTexFrame[i].first;
                    count++;
//                    std::cout << "imsize" << i << " " << imSize.height << "x" << imSize.width << "x"
//                              << mat_array[i].channels() << endl;
                }
            }
            if (count < numKFs)
                return;

            cv::Mat texture;
            cv::vconcat(mat_array, numKFs, texture);

            glEnable(GL_TEXTURE_2D);

            glBindTexture(GL_TEXTURE_2D, frameTex);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
            // image are saved in RGB format, grayscale images are converted
            if (bRGB) {
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
                             imSize.width, imSize.height*numKFs, 0,
                             GL_BGR,
                             GL_UNSIGNED_BYTE,
                             texture.data);
            } else {
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
                             imSize.width, imSize.height*numKFs, 0,
                             GL_RGB,
                             GL_UNSIGNED_BYTE,
                             texture.data);
            }

//            // select one texture for each triangle
//            std::vector<int> tex4tri;
//            for (list<dlovi::Matrix>::const_iterator it = GetTris().begin(); it != GetTris().end(); it++) {
//                int tex_ind_curr = -1;
//
//                for (int i = 0; i < numKFs; i++) {
//                    dlovi::Matrix point0 = GetPoints()[(*it)(0)];
//                    dlovi::Matrix point1 = GetPoints()[(*it)(1)];
//                    dlovi::Matrix point2 = GetPoints()[(*it)(2)];
//
//                    TextureFrame tex = imAndTexFrame[i].second;
//                    vector<float> uv0 = tex.GetTexCoordinate(point0(0), point0(1), point0(2), imSize);
//                    vector<float> uv1 = tex.GetTexCoordinate(point1(0), point1(1), point1(2), imSize);
//                    vector<float> uv2 = tex.GetTexCoordinate(point2(0), point2(1), point2(2), imSize);
//
//                    if (uv0.size() == 2 && uv1.size() == 2 && uv2.size() == 2) {
//                        tex_ind_curr = i;
//                        break;
//                    }
//                }
//
//                if (tex_ind_curr < 0)
//                    tex_ind_curr = 0;
//
//                tex4tri.push_back(tex_ind_curr);
//            }


            glBegin(GL_TRIANGLES);
            glColor3f(1.0, 1.0, 1.0);

            int index = 0;
            for (list<dlovi::Matrix>::const_iterator it = GetTris().begin(); it != GetTris().end(); it++, index++) {

                dlovi::Matrix point0 = GetPoints()[(*it)(0)];
                dlovi::Matrix point1 = GetPoints()[(*it)(1)];
                dlovi::Matrix point2 = GetPoints()[(*it)(2)];

                for (int i = 0; i < numKFs; i++) {

//                    int tex_ind = tex4tri[index];

                    TextureFrame tex = imAndTexFrame[i].second;
                    vector<float> uv0 = tex.GetTexCoordinate(point0(0), point0(1), point0(2), imSize);
                    vector<float> uv1 = tex.GetTexCoordinate(point1(0), point1(1), point1(2), imSize);
                    vector<float> uv2 = tex.GetTexCoordinate(point2(0), point2(1), point2(2), imSize);

                    if (uv0.size() == 2 && uv1.size() == 2 && uv2.size() == 2) {

                        glTexCoord2f(uv0[0], (uv0[1]+i)/numKFs);
                        glVertex3d(point0(0), point0(1), point0(2));

                        glTexCoord2f(uv1[0], (uv1[1]+i)/numKFs);
                        glVertex3d(point1(0), point1(1), point1(2));

                        glTexCoord2f(uv2[0], (uv2[1]+i)/numKFs);
                        glVertex3d(point2(0), point2(1), point2(2));

                        break;

                    }
                }

            }
            glEnd();

            glDisable(GL_TEXTURE_2D);

        }

    }

    void ModelDrawer::DrawModelPoints()
    {
        UpdateModel();

        glPointSize(3);
        glBegin(GL_POINTS);
        glColor3f(0.5, 0.5, 0.5);
        for (size_t i = 0; i < GetPoints().size(); i++) {
            glVertex3d(GetPoints()[i](0), GetPoints()[i](1), GetPoints()[i](2));
        }
        glEnd();
    }

    void ModelDrawer::SetTarget(pangolin::OpenGlMatrix &Twc, Eigen::Vector3d t)
    {
        Eigen::Vector3d diff = t - target;
        if (diff.squaredNorm() > 0.0) {
            target = t;
            Eigen::Matrix<double,4,4> m = Twc;
            target_w = Eigen::Transform<double,3,Eigen::Affine>(m) * t;
        }
    }

    cv::Mat ModelDrawer::GetTarget()
    {
        cv::Mat T = (cv::Mat_<float>(3,1) << target_w[0], target[1], target[2]);
        return T;
    }

    void ModelDrawer::DrawTarget()
    {
        glPointSize(10);
        glBegin(GL_POINTS);
        glColor3f(0.0, 1.0, 0.0);
        glVertex3d(target_w[0], target_w[1], target_w[2]);
        glEnd();
    }

    void ModelDrawer::DrawTarget(pangolin::OpenGlMatrix &Twc, Eigen::Vector3d t)
    {
        SetTarget(Twc, t);
        DrawTarget();
    }

    void ModelDrawer::DrawTriangles(pangolin::OpenGlMatrix &Twc)
    {
        UpdateModel();

        glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

        GLfloat light_position[] = { 0.0, 0.0, 1.0, 0.0 };
        glLightfv(GL_LIGHT0, GL_POSITION, light_position);

        glPopMatrix();

        glEnable(GL_LIGHTING);
        glEnable(GL_LIGHT0);

        glShadeModel(GL_FLAT);

        GLfloat material_diffuse[] = {0.2, 0.5, 0.8, 1};
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, material_diffuse);

        glBegin(GL_TRIANGLES);
        glColor3f(1.0,1.0,1.0);

        for (list<dlovi::Matrix>::const_iterator it = GetTris().begin(); it != GetTris().end(); it++) {

            dlovi::Matrix point0 = GetPoints()[(*it)(0)];
            dlovi::Matrix point1 = GetPoints()[(*it)(1)];
            dlovi::Matrix point2 = GetPoints()[(*it)(2)];

            dlovi::Matrix edge10 = point1 - point0;
            dlovi::Matrix edge20 = point2 - point0;

            dlovi::Matrix normal = edge20.cross(edge10);
            normal = normal / normal.norm();

            glNormal3d(normal(0), normal(1), normal(2));

            glVertex3d(point0(0), point0(1), point0(2));
            glVertex3d(point1(0), point1(1), point1(2));
            glVertex3d(point2(0), point2(1), point2(2));

        }
        glEnd();

        glDisable(GL_LIGHTING);

    }

    void ModelDrawer::DrawFrame(bool bRGB)
    {
        // select the last frame
        int numKFs = 1;
        vector<pair<cv::Mat,TextureFrame>> imAndTexFrame = mpModeler->GetTextures(numKFs);

        if ((int)imAndTexFrame.size() >= numKFs) {
            glColor3f(1.0,1.0,1.0);

            if (imAndTexFrame[0].first.empty()){
                std::cerr << "ERROR: empty frame image" << endl;
                return;
            }
            cv::Size imSize = imAndTexFrame[0].first.size();

            if(bRGB) {
                pangolin::GlTexture imageTexture(imSize.width, imSize.height, GL_RGB, false, 0, GL_BGR,
                                                 GL_UNSIGNED_BYTE);
                imageTexture.Upload(imAndTexFrame[0].first.data, GL_BGR, GL_UNSIGNED_BYTE);
                imageTexture.RenderToViewportFlipY();
            } else {
                pangolin::GlTexture imageTexture(imSize.width, imSize.height, GL_RGB, false, 0, GL_RGB,
                                                 GL_UNSIGNED_BYTE);
                imageTexture.Upload(imAndTexFrame[0].first.data, GL_RGB, GL_UNSIGNED_BYTE);
                imageTexture.RenderToViewportFlipY();
            }

        }
    }

    cv::Mat ModelDrawer::DrawLines()
    {
        cv::Mat im = mpModeler->GetImageWithLines();
        return im;
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
