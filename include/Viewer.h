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


#ifndef VIEWER_H
#define VIEWER_H

#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Tracking.h"
#include "System.h"
#include "Tree.h"

#include <mutex>

namespace ORB_SLAM2
{

class Tracking;
class FrameDrawer;
class MapDrawer;
class System;
class Tree;

class Viewer
{
public:
    Viewer(System* pSystem, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Tracking *pTracking, const string &strSettingPath);

    // Main thread function. Draw points, keyframes, the current camera pose and the last processed
    // frame. Drawing is refreshed according to the camera fps. We use Pangolin.
    void Run();

    void RequestFinish();

    void RequestStop();

    bool isFinished();

    bool isStopped();

    void Release();

    float cameraRotationAngle(float theta, cv::Mat currentPose);

    void sawTrees(cv::Mat& im);

    bool IsPointInCircularSector(
        float cx, float cy, float ux, float uy, float r, float theta,
        float px, float py);

    bool IsPointInCircularSector1(
        float cx, float cy, float ux, float uy, float squaredR, float cosTheta,
        float px, float py);

    float getTreeTheta(float xtree, float ytree);

    float getTreePosition(Tree tree);
    float getRealRadius(Tree tree);

    Tree findClosestTree();

    void highPrecisionNaviByDepth(const cv::Mat& depth);

    void NavigateToClosestTree(cv::Mat& im, Tree tree);

    float getCamVectorX();
    float getCamVectorY();

    float absTheta;


private:

    bool Stop();

    System* mpSystem;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;
    Tracking* mpTracker;

    // 1/fps in ms
    double mT;
    float mImageWidth, mImageHeight;

    float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    bool mbStopped;
    bool mbStopRequested;
    std::mutex mMutexStop;

    vector<float> thetaList;

    double PI = 3.1415926;

    float x0;
    float y0;
//    float x1;
//    float y1;
    float camera_k;

    cv::Mat dilate_element;



    void drawLeftArrow(cv::Mat& picture);
    void drawRightArrow(cv::Mat& picture);
    void drawFrontArrow(cv::Mat& picture);
    void drawCorrectPose(cv::Mat& picture);

};

}


#endif // VIEWER_H
	

