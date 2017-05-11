/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Viewer.h"
#include <pangolin/pangolin.h>


#include <mutex>

int a = 0;


namespace ORB_SLAM2
{

    Viewer::Viewer(System* pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking, const string &strSettingPath):
        mpSystem(pSystem), mpFrameDrawer(pFrameDrawer),mpMapDrawer(pMapDrawer), mpTracker(pTracking),
        mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false)
    {
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        float fps = fSettings["Camera.fps"];
        if(fps<1)
            fps=30;
        mT = 1e3/fps;

        mImageWidth = fSettings["Camera.width"];
        mImageHeight = fSettings["Camera.height"];
        if(mImageWidth<1 || mImageHeight<1)
        {
            mImageWidth = 640;
            mImageHeight = 480;
        }

        mViewpointX = fSettings["Viewer.ViewpointX"];
        mViewpointY = fSettings["Viewer.ViewpointY"];
        mViewpointZ = fSettings["Viewer.ViewpointZ"];
        mViewpointF = fSettings["Viewer.ViewpointF"];

        dilate_element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(9, 9));
    }

    void Viewer::Run()
    {
        mbFinished = false;
        mbStopped = false;

        pangolin::CreateWindowAndBind("ORB-SLAM2: Map Viewer",1024,768);

        // 3D Mouse handler requires depth testing to be enabled
        glEnable(GL_DEPTH_TEST);

        // Issue specific OpenGl we might need
        glEnable (GL_BLEND);
        glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
        pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
        pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
        pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
        pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);
        pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",false,true);
        pangolin::Var<bool> menuReset("menu.Reset",false,false);

        // Define Camera Render Object (for view / scene browsing)
        pangolin::OpenGlRenderState s_cam(
                    pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
                    pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
                    );

        // Add named OpenGL viewport to window and provide 3D Handler
        pangolin::View& d_cam = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
                .SetHandler(new pangolin::Handler3D(s_cam));

        pangolin::OpenGlMatrix Twc;
        Twc.SetIdentity();

//        cv::namedWindow("ORB-SLAM2: Current Frame");
//        cv::namedWindow("Depth Image");


        bool bFollow = true;
        bool bLocalizationMode = false;

        while(1)
        {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);

            if(menuFollowCamera && bFollow)
            {
                s_cam.Follow(Twc);
            }
            else if(menuFollowCamera && !bFollow)
            {
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
                s_cam.Follow(Twc);
                bFollow = true;
            }
            else if(!menuFollowCamera && bFollow)
            {
                bFollow = false;
            }

            if(menuLocalizationMode && !bLocalizationMode)
            {
                mpSystem->ActivateLocalizationMode();
                bLocalizationMode = true;
            }
            else if(!menuLocalizationMode && bLocalizationMode)
            {
                mpSystem->DeactivateLocalizationMode();
                bLocalizationMode = false;
            }

            d_cam.Activate(s_cam);
            glClearColor(1.0f,1.0f,1.0f,1.0f);
            mpMapDrawer->DrawCurrentCamera(Twc);
            if(menuShowKeyFrames || menuShowGraph)
                mpMapDrawer->DrawKeyFrames(menuShowKeyFrames,menuShowGraph);
            if(menuShowPoints)
                mpMapDrawer->DrawMapPoints();

            pangolin::FinishFrame();

            cv::Mat im = mpFrameDrawer->DrawFrame();

            cv::Mat imDepth = mpFrameDrawer->DrawFrameDepth();

            a++;
            if (imDepth.data)
            {
                highPrecisionNaviByDepth(imDepth);
            }




            if (!mpFrameDrawer->curKeyFrame.empty())
            {

                cv::Mat camPose = mpSystem->getPose();

                cv::Mat sample = (cv::Mat_<float>(4,4));

                if(camPose.size() == sample.size())
                {
                    float thetay = atan2(-1 * camPose.ptr<float>(0)[2], sqrt(camPose.ptr<float>(1)[2]*camPose.ptr<float>(1)[2]
                                                                         + camPose.ptr<float>(2)[2]*camPose.ptr<float>(2)[2])) / PI * 180;
                    absTheta = cameraRotationAngle(thetay, camPose);
                    x0 = camPose.ptr<float>(2)[3];
                    y0 = camPose.ptr<float>(0)[3];

                }



                if (!mpSystem->GetTrees().empty())
                {

//                    cv::Mat camPose = mpFrameDrawer->curKeyFrame.back()->GetPose();

//                    float thetay = atan2(-1 * camPose.ptr<float>(0)[2], sqrt(camPose.ptr<float>(1)[2]*camPose.ptr<float>(1)[2]
//                                                                         + camPose.ptr<float>(2)[2]*camPose.ptr<float>(2)[2])) / PI * 180;

                    //left means positive +++++++
//                    double thetay1 = atan2(-1 * camPose.ptr<float>(2)[0], sqrt(camPose.ptr<float>(2)[1]*camPose.ptr<float>(2)[1]
//                                                                         + camPose.ptr<float>(2)[2]*camPose.ptr<float>(2)[2])) / PI * 180;

//                    cout<<"absTheta: *1.03"<<endl;
//                    absTheta = cameraRotationAngle(thetay, camPose);

//                    cv::Mat camPose = mpFrameDrawer->curKeyFrame.back()->GetPose();

                    thetaList.push_back(absTheta);


                    camera_k = tan((90+absTheta)*PI/180);

                    if (thetaList.size() >= 3 && fabs(thetaList[thetaList.size()-1] - thetaList[thetaList.size()-2]) < 20 )
                        absTheta = (thetaList[thetaList.size()-1] + thetaList[thetaList.size()-2] + thetaList[thetaList.size()-3])/3;

                    sawTrees(im);

                    Tree closestTree = findClosestTree( );

//                    cout<<"closestTreex: "<<closestTree.center.x<<endl;
//                    cout<<"closestTreey: "<<closestTree.center.y<<endl;

//                    cv::Mat outDepthImage;

                    NavigateToClosestTree(im, closestTree);

//                    cout<<"------------------------------------------------------------------------: "<<endl<<endl;

                }
            }

            cv::imshow("ORB-SLAM2: Current Frame",im);

//            cv::imshow("Depth Image",imDepth);

            //cout<<"imshow ORB-SLAM2: Current Frame "<<im.size()<<endl;

            cv::waitKey(mT);

            if(menuReset)
            {
                menuShowGraph = true;
                menuShowKeyFrames = true;
                menuShowPoints = true;
                menuLocalizationMode = false;
                if(bLocalizationMode)
                    mpSystem->DeactivateLocalizationMode();
                bLocalizationMode = false;
                bFollow = true;
                menuFollowCamera = true;
                mpSystem->Reset();
                menuReset = false;
            }

            if(Stop())
            {
                while(isStopped())
                {
                    usleep(3000);
                }
            }

            if(CheckFinish())
                break;
        }

        SetFinish();
    }


    void Viewer::highPrecisionNaviByDepth(const cv::Mat& depth)
    {
        int nr=depth.rows;
        int nc=depth.cols;

        cv::Mat out;
        cv::dilate(depth,out, dilate_element);


        for(int i = 0; i< nr;i+=10)
        {

            for (int j = 0;j<nc;j+=10)
            {

            }

        }


        cv::imshow("Depth Image",out);

    }

    void Viewer::drawCorrectPose(cv::Mat& picture)
    {
        // draw square
        cv::Rect rec(2,2,637,477);
        rectangle(picture, rec, cv::Scalar(20, 255, 20), 3, 8, 0);
    }

    void Viewer::drawLeftArrow(cv::Mat& picture)
    {
        // draw arrow
        cv::Point point1 = cv::Point(140, 400);
        cv::Point point2 = cv::Point(40, 400);
        arrowedLine(picture, point1, point2, cv::Scalar(0, 0, 255), 3,8,0,0.3);
    }

    void Viewer::drawRightArrow(cv::Mat& picture)
    {
        // draw arrow
        cv::Point point1 = cv::Point(500, 400);
        cv::Point point2 = cv::Point(600, 400);
        arrowedLine(picture, point1, point2, cv::Scalar(0, 0, 255), 3,8,0,0.3);
    }

    void Viewer::drawFrontArrow(cv::Mat& picture)
    {
        // draw arrow
        cv::Point point1 = cv::Point(320, 400);
        cv::Point point2 = cv::Point(320, 350);
        arrowedLine(picture, point1, point2, cv::Scalar(0, 0, 255), 3,8,0,0.3);
    }

    /**
     * @brief Viewer::NavigateToClosestTree is the navigation method to allow the camera towards
     * the target tree
     * @param im - draw arrow to the image
     * @param tree - target tree
     */
    void Viewer::NavigateToClosestTree(cv::Mat& im, Tree tree)
    {
        float x1 = tree.center.x;
        float y1 = tree.center.y;

        float xtree = x1-x0;
        float ytree = y1-y0;

        float treeTheta = getTreeTheta(xtree, ytree);



        int treePosition = getTreePosition(tree);
        if(treePosition > 10 && treePosition < 630)
        {
            if (treePosition > 220 && treePosition < 420)
            {
                //toward
                cout<<"toward: "<<sqrt((xtree*xtree)+(ytree*ytree))<<endl;

                if (sqrt((xtree*xtree)+(ytree*ytree)) < 0.8)
                {
                    //stop
                    drawCorrectPose(im);
    //                cout<<"go ahead and Now Stop"<<endl;
                }else{
                    drawFrontArrow(im);
                }
            }
            else if (treePosition <= 220)
            {
                drawLeftArrow(im);
            }
            else if (treePosition >= 420)
            {
                drawRightArrow(im);
            }
        }
        else if (absTheta < 180)
        {
            if (treeTheta>absTheta && treeTheta < (absTheta+180))
            {
                //left
                drawLeftArrow(im);
//                cout<<"left"<<endl;
            }else
            {
                //right
                drawRightArrow(im);
//                cout<<"right"<<endl;
            }
        }else{
            if (treeTheta<absTheta && treeTheta > (absTheta-180))
            {
                //right
                drawRightArrow(im);
//                cout<<"right"<<endl;
            }else
            {
                //left
                drawLeftArrow(im);
//                cout<<"left"<<endl;
            }
        }
    }

    /**
     * @brief Viewer::findClosestTree find the closest tree from detected trees
     *
     * @param currentPose
     * @return the closest tree
     */
    Tree Viewer::findClosestTree()
    {
        Tree closestTree = mpSystem->GetTrees()[0];


        for (size_t i = 0; i < mpSystem->GetTrees().size(); i++)
        {

            float x1 = mpSystem->GetTrees()[i].center.x;
            float y1 = mpSystem->GetTrees()[i].center.y;
            float disTreeCamera = sqrt( (x0-x1)*(x0-x1)+(y0-y1)*(y0-y1) );

            float closest_x1 = closestTree.center.x;
            float closest_y1 = closestTree.center.y;
            float closest_disTreeCamera = sqrt( (x0-closest_x1)*(x0-closest_x1)+(y0-closest_y1)*(y0-closest_y1) );

            if (disTreeCamera < closest_disTreeCamera)
            {
                closestTree = mpSystem->GetTrees()[i];

            }
        }

        return closestTree;
    }

    /**
     * @brief Viewer::getTreeTheta get the angle between tree to camera and camera initial vector
     * @param xtree - tree position in x axis
     * @param ytree - tree position in y (z) axis
     * @return the angle between x-axis and view line from camera to tree.
     */
    float Viewer::getTreeTheta(float xtree, float ytree)
    {
        float thetaTree = 0;

        //right top area
        if (xtree > 0 && ytree > 0)
        {
            thetaTree = 360-(atan(xtree/ytree)/3.1415926*180);
        }
        //left top area
        if (xtree < 0 && ytree > 0)
        {
            thetaTree = -(atan(xtree/ytree)/3.1415926*180);
        }
        //left bottom area
        if (xtree < 0 && ytree < 0)
        {
            thetaTree = 180-(atan(xtree/ytree)/3.1415926*180);
        }
        //right bottom area
        if (xtree > 0 && ytree < 0)
        {
            thetaTree = 180-(atan(xtree/ytree)/3.1415926*180);
        }

        return thetaTree;
    }

    /**
     * @brief Viewer::getTreePosition get the tree projection in 2D image 640 x 480
     * @param x1 - the tree position in x axis
     * @param y1 - the tree position in y(z) axis
     * @return the tree position in the 2D image
     */
    float Viewer::getTreePosition(Tree tree)
    {
        float treePosiX;

        float disMidLine = (camera_k*tree.center.x + (-tree.center.y) + (-((x0 * camera_k) - y0))) / sqrt((camera_k*camera_k) + 1);

        float disTreeCamera = sqrt( (x0-tree.center.x)*(x0-tree.center.x)+(y0-tree.center.y)*(y0-tree.center.y) );

        float disCamera = sqrt( (disTreeCamera*disTreeCamera) - (disMidLine*disMidLine) );

        float screenWideHalf = disCamera/2;

        float thetaTree = getTreeTheta(tree.center.x-x0, tree.center.y-y0);

//        float radius = mpSystem->GetTrees()[i].radius;

//        float realRadius = fabs((mImageWidth/2) * radius / screenWideHalf)/2;

        float errorPosi = 30/(2-disCamera);

        if ( ( absTheta > 0 && absTheta < 30 ) && thetaTree > 330 )
        {
            treePosiX = 320 +errorPosi + abs(disMidLine * 320 / screenWideHalf);
        }
        else if ( absTheta > 330  && ( thetaTree > 0 && thetaTree < 30 ))
        {
            treePosiX = 320 +errorPosi - abs(disMidLine * 320 / screenWideHalf);
        }
        else if (thetaTree > absTheta)
        {
            treePosiX = 320+errorPosi - abs(disMidLine * 320 / screenWideHalf);
        }
        else
        {
            treePosiX = 320+errorPosi + abs(disMidLine * 320 / screenWideHalf);
        }
        return treePosiX;
    }

    /**
     * @brief Viewer::getRealRadius get real radius from the rate of distances from
     * camera and tree.
     * @param tree - target measure tree
     * @param x1 - camera vector (pose in x)
     * @param y1 - camera vector (pose in y)
     * @return the radius shows in 2D image
     */
    float Viewer::getRealRadius(Tree tree)
    {
        float disMidLine = (camera_k*tree.center.x + (-tree.center.y) + (-((x0 * camera_k) - y0))) / sqrt((camera_k*camera_k) + 1);

        float disTreeCamera = sqrt( (x0-tree.center.x)*(x0-tree.center.x)+(y0-tree.center.y)*(y0-tree.center.y) );

        float disCamera = sqrt( (disTreeCamera*disTreeCamera) - (disMidLine*disMidLine) );

        float screenWideHalf = disCamera/2;


        float radius = tree.radius;

        return fabs((mImageWidth/2) * (radius) / screenWideHalf/2);
    }

    /**
     * @brief Viewer::getCamVectorX
     * @return
     */
    float Viewer::getCamVectorX()
    {
        float ux = 0;
        if (absTheta==0 || absTheta==360)
        {
            ux = 0;
        }
        else if (absTheta==90)
        {
            ux = -1;
        }
        else if (absTheta==180)
        {
            ux = 0;
        }
        else if (absTheta==270)
        {
            ux = 1;
        }

        else if (absTheta>0 && absTheta<180)
        {
            ux = - 1;
        }
        else if (absTheta>180 && absTheta<360)
        {
            ux = + 1;
        }

        return ux;
    }

    /**
     * @brief Viewer::getCamVectorY
     * @return
     */
    float Viewer::getCamVectorY()
    {
        float uy = 0;
        if (absTheta==0 || absTheta==360)
        {
            uy = 1;
        }
        else if (absTheta==90)
        {
            uy = 0;
        }
        else if (absTheta==180)
        {
            uy = -1;
        }
        else if (absTheta==270)
        {
            uy = 0;
        }

        else if (absTheta>0 && absTheta<180)
        {
            uy = (1/tan(absTheta*3.1415926/180));

        }
        else if (absTheta>180 && absTheta<360)
        {
            uy = - (1/tan(absTheta*3.1415926/180));
        }

        return uy;
    }

    /**
     * @brief Viewer::sawTrees
     * @param im
     */
    void Viewer::sawTrees(cv::Mat& im)
    {


        cout<<"absTheta: "<<absTheta<<endl;
        for (size_t i = 0 ; i < mpSystem->GetTrees().size(); i++)
        {
            cout<<"tree: "<<i<<endl<<mpSystem->GetTrees()[i].center<<"------"<<mpSystem->GetTrees()[i].radius<<endl;

            Tree tree = mpSystem->GetTrees()[i];

            float treePosiX = getTreePosition(tree);

            cout<<"treePosiX: "<< treePosiX <<endl;

            float ux = getCamVectorX();
            float uy = getCamVectorY();

//            if (treePosiX <600 && treePosiX > 30 && IsPointInCircularSector3(x0, y0, ux, uy, 100, 0.8660254, x1, y1))

            if (treePosiX <620 && treePosiX > 20 && IsPointInCircularSector(x0,y0,ux+x0,uy+y0,100,0.454,tree.center.x,tree.center.y))
            {

                float realRadius = getRealRadius(tree);

                cout<<"realRadius: "<<realRadius<<endl;

                cv::rectangle(im,cv::Point(treePosiX-realRadius,20),cv::Point(treePosiX+realRadius, 300),cv::Scalar(255,0,0), 3);
            }
        }

        cout<<endl<<endl;

    }



    /**
     * @brief Viewer::IsPointInCircularSector
     * @param cx
     * @param cy
     * @param ux
     * @param uy
     * @param r
     * @param theta
     * @param px
     * @param py
     * @return
     */
    bool Viewer::IsPointInCircularSector(
        float cx, float cy, float ux, float uy, float r, float theta,
        float px, float py)
    {
        assert(cosTheta > -1 && cosTheta < 1);
        assert(squaredR > 0.0f);

        // D = P - C
        float dx = px - cx;
        float dy = py - cy;

        // |D| = (dx^2 + dy^2)^0.5
        float length = sqrt((dx * dx) + (dy * dy));

        // |D| > r
        if (length > r)
            return false;

        // Normalize D
        dx /= length;
        dy /= length;

        float unx = (ux-cx)/sqrt(((ux-cx)*(ux-cx)) + ((uy-cy)*(uy-cy)));
        float uny = (uy-cy)/sqrt(((ux-cx)*(ux-cx)) + ((uy-cy)*(uy-cy)));



        // acos(D dot U) < theta
        return acos((dx * unx) + (dy * uny)) < theta;
    }

    bool Viewer::IsPointInCircularSector1(
        float cx, float cy, float ux, float uy, float squaredR, float cosTheta,
        float px, float py)
    {
        assert(cosTheta > -1 && cosTheta < 1);
        assert(squaredR > 0.0f);

        // D = P - C
        float dx = px - cx;
        float dy = py - cy;

        // |D|^2 = (dx^2 + dy^2)
        float squaredLength = dx * dx + dy * dy;

        // |D|^2 > r^2
        if (squaredLength > squaredR)
            return false;

        // |D|
        float length = sqrt(squaredLength);

        float unx = ux/sqrt((ux*ux) + (uy*uy));
        float uny = uy/sqrt((ux*ux) + (uy*uy));
    //    ux = ux * ;
//        cout<<"dx: "<<dx<<endl;
//        cout<<"unx: "<<unx<<endl;
//        cout<<"dy: "<<dy<<endl;
//        cout<<"uny: "<<uny<<endl;
//        cout<<"length * cosTheta: "<<length * cosTheta<<endl;
//        cout<<"(dx * unx) + (dy * uny): "<<(dx * unx) + (dy * uny)<<endl;

        // D dot U > |D| cos(theta)
        return dx * unx + dy * uny > length * cosTheta;
    }



    float Viewer::cameraRotationAngle(float theta, cv::Mat currentPose)
    {

        if (theta < 0 && currentPose.at<float>(2,2) > 0)
        {
    //        one = true;
            return -theta;
        }
        else if (theta < 0 && currentPose.at<float>(2,2) < 0)
        {
            //two = true;
            return -(-90-(theta)) + 90;
        }
        else if (theta > 0 && currentPose.at<float>(2,2) < 0)
        {
            //three = true;
            return theta + 180;
        }
        else if (theta > 0 && currentPose.at<float>(2,2) > 0)
        {
            //four = true;
            return (90-theta) + 270;
        }
        else
        {
            return 0;
        }
    }

    void Viewer::RequestFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinishRequested = true;
    }

    bool Viewer::CheckFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinishRequested;
    }

    void Viewer::SetFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinished = true;
    }

    bool Viewer::isFinished()
    {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinished;
    }

    void Viewer::RequestStop()
    {
        unique_lock<mutex> lock(mMutexStop);
        if(!mbStopped)
            mbStopRequested = true;
    }

    bool Viewer::isStopped()
    {
        unique_lock<mutex> lock(mMutexStop);
        return mbStopped;
    }

    bool Viewer::Stop()
    {
        unique_lock<mutex> lock(mMutexStop);
        unique_lock<mutex> lock2(mMutexFinish);

        if(mbFinishRequested)
            return false;
        else if(mbStopRequested)
        {
            mbStopped = true;
            mbStopRequested = false;
            return true;
        }

        return false;

    }

    void Viewer::Release()
    {
        unique_lock<mutex> lock(mMutexStop);
        mbStopped = false;
    }

}
