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

#include "FrameDrawer.h"
#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<mutex>

namespace ORB_SLAM2
{
FrameDrawer::FrameDrawer(Map* pMap, MapDrawer* pMapDrawer, const string &strSettingPath):
        mpMap(pMap),
        mpMapDrawer(pMapDrawer)
{
    mState=Tracking::SYSTEM_NOT_READY;

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];
    int cam_width = fSettings["Camera.width"];
    int cam_height = fSettings["Camera.height"];

    mRGB = cv::Mat(cam_height,cam_width,CV_8UC3, cv::Scalar(0,0,0));// 初始化一个空的三通道图像

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);// 相机内参数======
}

void FrameDrawer::FillImage(cv::Mat &im, const cv::Mat &mask, const cv::Mat &Semlab, cv::Scalar scalar_mask, cv::Scalar scalar_people, cv::Scalar scalar_chair)
{
    unsigned char* color = (unsigned char*)im.data; //3 通道
    unsigned char* dyna = (unsigned char*)mask.data;//1 通道 动静 mask
    unsigned char* label = (unsigned char*)Semlab.data;//1 通道 语义分割 Label

    for(int r=0; r<im.rows; r++ ) // y
    {
        for(int c=0; c<im.cols; c++) // x
        {
            int index = r*im.cols + c;// 总索引
            if(!dyna[index])
            {
                color[index*3+0] = scalar_mask.val[0];
                color[index*3+1] = scalar_mask.val[1];
                color[index*3+2] = scalar_mask.val[2];
            }
            if(label[index]==31)//people label添加mask
            {
                color[index*3+0] = scalar_people.val[0];
                color[index*3+1] = scalar_people.val[1];
                color[index*3+2] = scalar_people.val[2];
            }
            if(label[index]==5)//chair label添加mask
            {
                for(int k=0;k<dy_chair.size();k++)
                {
                    if(sqrt(pow((dy_chair[k].x-c), 2)+pow((dy_chair[k].y-r), 2))>150.0)
                    {
                        continue;
                    }
                    color[index*3+0] = scalar_chair.val[0];
                    color[index*3+1] = scalar_chair.val[1];
                    color[index*3+2] = scalar_chair.val[2];
                }
            }

        }
    }
}


cv::Mat FrameDrawer::DrawFrame()
{
    cv::Mat im,mask,label;
    vector<cv::KeyPoint> vIniKeys; // 初始化参考帧关键点 Initialization: KeyPoints in reference frame
    vector<int> vMatches;          // 匹配点 Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // 当前帧关键点 KeyPoints in current frame
    vector<bool> vbVO, vbMap;          // 跟踪的关键点 Tracked MapPoints in current frame
    // vbMap 匹配到地图上一个点
    // vbVO    匹配到 无观察帧的地图点
    int state; // Tracking state

    //Copy variables within scoped mutex   大括号，表示数据上锁区域
    {
        unique_lock<mutex> lock(mMutex);// 对数据上锁====
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;// 切换成 没有图像==

        mRGB.copyTo(im);                    // 由update函数从 tracer内拷贝过来======
        mDynMask.copyTo(mask);
        mask.cv::Mat::convertTo(mask,CV_8U);// 0～1,1处动，0处静

        mLabel.copyTo(label);
        label.cv::Mat::convertTo(label,CV_8U);// 单通道Label


        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeys;// 类对象 复制过来
            vIniKeys = mvIniKeys;        // 初始关键帧 关键点
            vMatches = mvIniMatches;     // 初始关键帧 关键帧匹配点
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeys;// 当前关键帧 关键点
            vbVO = mvbVO;   // 跟踪到的
            vbMap = mvbMap;
        }
        else if(mState==Tracking::LOST)// 跟丢了，关键点就没有匹配上===
        {
            vCurrentKeys = mvCurrentKeys;// 只有 当前帧 检测到的关键点
        }
    } // destroy scoped mutex -> release mutex

    if(im.channels()<3) //this should be always true
        cvtColor(im,im,CV_GRAY2BGR);// 变成三通道 可显示彩色===

    //Draw
    if(state==Tracking::NOT_INITIALIZED) //INITIALIZING=====初始化====
    {
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0)
            {
                cv::line(im,vIniKeys[i].pt,vCurrentKeys[vMatches[i]].pt,
                         cv::Scalar(0,255,0));
            }
        }
    }
    else if(state==Tracking::OK) //TRACKING  跟踪=====
    {
        mnTracked=0;
        mnTrackedVO=0;
        const float r = 5;
        //const int n = vCurrentKeys.size();
        for(int i=0;i<N;i++)// Update()函数中已经获取到===
        {
            if(vbVO[i] || vbMap[i]) // 跟踪到 的关键点=====
            {
                cv::Point2f pt1,pt2;
                pt1.x=vCurrentKeys[i].pt.x-r;// 左上方点
                pt1.y=vCurrentKeys[i].pt.y-r;
                pt2.x=vCurrentKeys[i].pt.x+r;// 右下方点
                pt2.y=vCurrentKeys[i].pt.y+r;

                // This is a match to a MapPoint in the map  匹配到地图上一个点=====
                if(vbMap[i])
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));// bgr 绿色  正方形
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(0,255,0),-1);// 内画圆
                    mnTracked++; // 跟踪到的地图点数量====
                }
                else // 跟踪到的上一帧创建的 视觉里程记点 (部分会被删除掉)
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,0));// bgr  蓝色===
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(255,0,0),-1);
                    mnTrackedVO++;
                }
// 显示==========语义分割=====光流信息==动静信息===========
            }
        }
        // cvCopy(im,im,mask);
        // im.copyTo(im, mask);
        //cv::floodFill(im, mask,  cv::Point(200,200), cv::Scalar(255 , 0, 0));
        // #mask不为0的区域不会被填充，mask为0的区域才会被填充
        cv::Scalar s = sum(mask);//各通道求和
        if(!mask.empty()&&(s.val[0] > mask.rows * mask.cols * 0.65))
            FillImage(im, mask, label, cv::Scalar(255 , 0, 0), cv::Scalar(155, 211, 255), cv::Scalar(161 , 66, 179));
    }

    cv::Mat imWithInfo;
    DrawTextInfo(im,state, imWithInfo);// 显示文字

    return imWithInfo;
}


void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
{
    stringstream s;
    if(nState==Tracking::NO_IMAGES_YET)
        s << " WAITING FOR IMAGES";
    else if(nState==Tracking::NOT_INITIALIZED)
        s << " TRYING TO INITIALIZE ";
    else if(nState==Tracking::OK)
    {
        if(!mbOnlyTracking)
            s << "SLAM MODE |  ";
        else
            s << "LOCALIZATION | ";
        int nKFs = mpMap->KeyFramesInMap();
        int nMPs = mpMap->MapPointsInMap();
        s << "KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mnTracked;
        if(mnTrackedVO>0)
            s << ", + VO matches: " << mnTrackedVO;
    }
    else if(nState==Tracking::LOST)
    {
        s << " TRACK LOST. TRYING TO RELOCALIZE ";
    }
    else if(nState==Tracking::SYSTEM_NOT_READY)
    {
        s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }

    int baseline=0;
    cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);

    imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());
    im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
    imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
    cv::putText(imText,s.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);

}

void FrameDrawer::Update(Tracking *pTracker)
{
    unique_lock<mutex> lock(mMutex);
    pTracker->mImGray.copyTo(mGray);

    pTracker->mimLabel.copyTo(mLabel);// 语义分割标签
    pTracker->mImMask.copyTo(mDynMask);// 动态点mask
    pTracker->mImDepth.copyTo(mImDep);// 深度图
    pTracker->mImRGB.copyTo(mRGB);  // 彩色图
    dy_chair.assign(pTracker->dynamic_chair.begin(),pTracker->dynamic_chair.end());  //从dynamic_chair拷贝到dy_chair
    // pTracker->mK.copyTo(mK);// 相机内参数===
    pTracker->mCurrentFrame.mTcw.copyTo(mTcw);

    mvCurrentKeys=pTracker->mCurrentFrame.mvKeys;
    N = mvCurrentKeys.size();
    mvbVO = vector<bool>(N,false);
    mvbMap = vector<bool>(N,false);
    mbOnlyTracking = pTracker->mbOnlyTracking;


    if(pTracker->mLastProcessedState==Tracking::NOT_INITIALIZED)
    {
        mvIniKeys=pTracker->mInitialFrame.mvKeys;
        mvIniMatches=pTracker->mvIniMatches;
    }
    else if(pTracker->mLastProcessedState==Tracking::OK)
    {
        for(int i=0;i<N;i++)
        {
            MapPoint* pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
            if(pMP)
            {
                if(!pTracker->mCurrentFrame.mvbOutlier[i])
                {
                    if(pMP->Observations()>0)
                        mvbMap[i]=true;
                    else
                        mvbVO[i]=true;
                }
            }
        }
    }
    mState=static_cast<int>(pTracker->mLastProcessedState);
}

} //namespace ORB_SLAM
