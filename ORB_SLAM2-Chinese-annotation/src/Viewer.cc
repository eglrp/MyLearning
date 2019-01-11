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
    ///Viewer.ViewpointX: 0
    ///Viewer.ViewpointY: -0.7
    /// Viewer.ViewpointZ: -1.8
    ///Viewer.ViewpointF: 500

    mViewpointX = fSettings["Viewer.ViewpointX"];
    mViewpointY = fSettings["Viewer.ViewpointY"];
    mViewpointZ = fSettings["Viewer.ViewpointZ"];
    mViewpointF = fSettings["Viewer.ViewpointF"];
}

//视图类主函数
void Viewer::Run()
{
    mbFinished = false;//没结束
    mbStopped = false;//也没停止

    pangolin::CreateWindowAndBind("ORB-SLAM2: Map Viewer",1024,768);//创建窗口

    // 3D Mouse handler requires depth testing to be enabled
    //GL_DEPTH_TEST为16进制的数，启动深度测试, 打开OpenGL的特殊功能
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);//glBlendFunc 特殊的像素算法

    // Add named Panel and bind to variables beginning 'menu‘'
    // A Panel is just a View with a default layout and input handling
    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));//设置菜单的位置
    //设置按钮
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);
    pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",false,true);
    pangolin::Var<bool> menuReset("menu.Reset",false,false);

    // Define Camera Render Object (for view / scene browsing)
    //ModelViewLookAt矩阵是模型矩阵（Model Matrix）和视觉矩阵（View Matrix）的组合，Model 变换指的是将Object  Space转换到World Space
    //（译注：World Space值得是OpenGL中的三维空间），而View 变换是将World space变换到eye space；
    //视觉坐标系和ProjectionMatrix矩阵相乘，得到剪切面坐标系。
    /*pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
                );*/
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(640,480,420,420,320,240,0.2,100),
        pangolin::ModelViewLookAt(-0,0.5,-3, 0,0,0, pangolin::AxisY)
      );


    // Add named OpenGL viewport to window and provide 3D Handler
    ////setBounds 跟opengl的viewport 有关
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    cv::namedWindow("ORB-SLAM2: Current Frame");

    bool bFollow = true;
    bool bLocalizationMode = false;

    while(1)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);//glClear 用当前值清除缓冲区

        mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);//地图可视类，得到当前相机位姿，储存在Twc

        if(menuFollowCamera && bFollow)//按钮menuFollowCamera为真, 视角对相机进行跟踪
        {
            s_cam.Follow(Twc);//栈中加入当前帧的位姿(视角跟踪相机运动)
        }
        else if(menuFollowCamera && !bFollow)//重新跟踪,再一次回复视角跟踪信息
        {
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
            s_cam.Follow(Twc);
            bFollow = true;
        }
        else if(!menuFollowCamera && bFollow)//如果menuFollowCamera为flase,则认为不允许视角跟踪相机
        {
            bFollow = false;
        }

        //menuLocalizationMode, bLocalizationMode初始均为false, 局部建图线程开启(默认), 按下则关闭局部建图线程
        if(menuLocalizationMode && !bLocalizationMode)
        {
            mpSystem->ActivateLocalizationMode();//关闭局部建图线程
            bLocalizationMode = true;
        }
        else if(!menuLocalizationMode && bLocalizationMode)//按两次后开启, 重新开启局部建图线程
        {
            mpSystem->DeactivateLocalizationMode();//开启局部建图线程
            bLocalizationMode = false;
        }

        d_cam.Activate(s_cam);//相机位姿可视化
        glClearColor(1.0f,1.0f,1.0f,1.0f);//颜色处理(白色)
        mpMapDrawer->DrawCurrentCamera(Twc);//画相机
        if(menuShowKeyFrames || menuShowGraph)//都为true
            mpMapDrawer->DrawKeyFrames(menuShowKeyFrames,menuShowGraph);
        if(menuShowPoints)//需要画地标点
            mpMapDrawer->DrawMapPoints();

        pangolin::FinishFrame();//结束(帧的绘制)

        cv::Mat im = mpFrameDrawer->DrawFrame();
        cv::imshow("ORB-SLAM2: Current Frame",im);
        cv::waitKey(mT);//按帧率等待现实

        if(menuReset)//如果按下了重置按钮
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

//停止状态判定
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
