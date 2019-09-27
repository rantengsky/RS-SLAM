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

#ifndef MAPDRAWER_H
#define MAPDRAWER_H
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>

#include"Map.h"
#include"Viewer.h"
#include"MapPoint.h"
#include"KeyFrame.h"
#include"Tracking.h"
#include"Converter.h"
#include"LocalMapping.h"
#include<pangolin/pangolin.h>
#include<condition_variable>
#include<mutex>
#include<thread>
// octomap
#include <octomap/ColorOcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
// pcl
#include <pcl/io/pcd_io.h>// 读写
#include <pcl/common/transforms.h>// 点云坐标变换
#include <pcl/point_types.h>      // 点类型
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>// 体素格滤波
#include <pcl/filters/passthrough.h>//  直通滤波
#include <pcl/sample_consensus/method_types.h>// 采样一致性，采样方法
#include <pcl/sample_consensus/model_types.h>// 模型
#include <pcl/segmentation/sac_segmentation.h>// 采样一致性分割
#include <pcl/filters/extract_indices.h>// 提取点晕索引
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>

namespace ORB_SLAM2
{

class Viewer;
class MapDrawer
{
public:
    MapDrawer(Map* pMap, const string &strSettingPath);
    ~MapDrawer();

    Map* mpMap;
    octomap::ColorOcTree *m_octree;
    octomap_msgs::Octomap map_msg;

    typedef pcl::PointXYZRGBL PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    pcl::VoxelGrid<PointT>  voxel;

    void DrawMapPoints();
    void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph);
    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
    void SetCurrentCameraPose(const cv::Mat &Tcw);
    void SetReferenceKeyFrame(KeyFrame *pKF);
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);

    void DrawGrid();// 通过画线 显示格子

    // 保存 octomap地图====
    void SaveOctoMap(const char*filename);

    // 发布octomap图====
    void PubOctoMap();

    void InsertFrame(KeyFrame *kf);
    bool CheckNewFrame();
    KeyFrame* GetNewKeyFrame();

protected:
    // 生成当前帧的点云
    void GeneratePointCloud(KeyFrame* kf);
    void InsertScan(octomap::point3d origin, PointCloud &cloud);

    std::vector<KeyFrame*>       keyframes;
    uint16_t                     lastKeyframeSize =0;
    std::mutex mMutexNewKFs;
private:

    ros::NodeHandle nn;
    // OctoMap.
    ros::Publisher puber_octomap;
    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mGraphLineWidth;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;
    bool bIsLocalization;
    float cam_fx, cam_fy, cam_cx, cam_cy;
    cv::Mat mCameraPose;

    std::list<KeyFrame*> mlNewKeyFrames;

    std::mutex mMutexCamera;
private:
    octomap::ColorOcTreeNode::Color its; // 获取八叉树方格的颜色

    double m_maxRange;
    bool m_useHeightMap;
    // octomap param
    double m_colorFactor;
    float m_res;
    float m_occupy;
    float m_depth;
    float max_range;
    float raycast_range;
    float clamping_thres_min;
    float clamping_thres_max;
    float prob_hit;
    float prob_miss;
    unsigned m_treeDepth;
    unsigned m_maxTreeDepth;
    int cam_width;
    int cam_height;
    int expansion;
    float punishment;
};

} //namespace ORB_SLAM

#endif // MAPDRAWER_H
