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

#ifndef FRAMEDRAWER_H
#define FRAMEDRAWER_H

#include "Tracking.h"
#include "MapPoint.h"
#include "Map.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include<mutex>


namespace ORB_SLAM2
{

class Tracking;
class Viewer;
class MapDrawer;

class FrameDrawer
{
public:
    FrameDrawer(Map* pMap);
    FrameDrawer(Map* pMap, MapDrawer* pMapDrawer, const string &strSettingPath);
    // Update info from the last processed frame.
    void Update(Tracking *pTracker);

    // Draw last processed frame.
    cv::Mat DrawFrame();
protected:

    void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);

    void FillImage(cv::Mat &im, const cv::Mat &mask, const cv::Mat &Semlab, cv::Scalar color_mask, cv::Scalar color_people, cv::Scalar color_chair);

    // Info of the frame to be drawn
    cv::Mat mLabel;// 语义分割标签
    cv::Mat mDynMask;// 动态点mask
    cv::Mat mImDep;// 深度图
    cv::Mat mRGB;// 深度图
    cv::Mat mK;// 相机内参数
    cv::Mat mTcw;// 相机位姿
    vector<cv::Point2f> dy_chair; //存储潜在的移动物体中心点（这里认为是椅子）
    // Info of the frame to be drawn
    cv::Mat mGray;
    int N;
    vector<cv::KeyPoint> mvCurrentKeys;
    vector<bool> mvbMap, mvbVO;
    bool mbOnlyTracking;
    int mnTracked, mnTrackedVO;
    vector<cv::KeyPoint> mvIniKeys;
    vector<int> mvIniMatches;
    int mState;

    Map* mpMap;

    MapDrawer* mpMapDrawer;// 地图显示绘制

    std::mutex mMutex;
};

} //namespace ORB_SLAM

#endif // FRAMEDRAWER_H
