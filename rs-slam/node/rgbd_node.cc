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



#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include"../include/Converter.h"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <unistd.h>

#include <opencv2/core/core.hpp>

#include"../include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);
    void GrabSemtic(const sensor_msgs::ImageConstPtr& msgSemL,const sensor_msgs::ImageConstPtr& msgSemC, const sensor_msgs::ImageConstPtr& msgSemConf);

    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;
//订阅语义标签和颜色并同步
    message_filters::Subscriber<sensor_msgs::Image> seml_sub(nh, "/semantic_image/semantic_label", 1);
    cout << "ROS-ImageGrabber->订阅/semantic_image/semantic_label话题--" << endl;
    message_filters::Subscriber<sensor_msgs::Image> semc_sub(nh, "/semantic_image/semantic_color", 1);
    cout << "ROS-ImageGrabber->订阅/semantic_image/semantic_color话题--" << endl;
//订阅confidence
    message_filters::Subscriber<sensor_msgs::Image> semconf_sub(nh, "/semantic_image/semantic_confidence", 1);
    cout << "ROS-ImageGrabber->订阅/semantic_image/semantic_confidence话题--" << endl;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> sync_pol2;
    message_filters::Synchronizer<sync_pol2> sync2(sync_pol2(10), seml_sub,semc_sub,semconf_sub);
    cout << "ROS-ImageGrabber->话题同步处理--" << endl;
    sync2.registerCallback(boost::bind(&ImageGrabber::GrabSemtic,&igb,_1,_2,_3));
    cout << "ROS-ImageGrabber->调用callback函数GrabSemtic--" << endl;
///订阅彩色图和深度图并同步
///kinect2 话题：/kinect2/qhd/image_color  /kinect2/qhd/image_depth_rect
///TUM 话题：/camera/rgb/image_color  /camera/depth/image
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_color", 1);
    cout << "ROS-ImageGrabber->订阅/camera/rgb/image_color话题--" << endl;
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth/image", 1);
    cout << "ROS-ImageGrabber->订阅/camera/depth/image话题--" << endl;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol1;
    message_filters::Synchronizer<sync_pol1> sync1(sync_pol1(10), rgb_sub,depth_sub);
    cout << "ROS-ImageGrabber->话题同步处理--" << endl;
    sync1.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));
    cout << "ROS-ImageGrabber->调用callback函数GrabRGBD--" << endl;

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

//语义分割图像的回调函数//订阅的结果进入回调函数，也就进入了这个工程//这是图像分割的回调
void ImageGrabber::GrabSemtic(const sensor_msgs::ImageConstPtr& msgSemL, const sensor_msgs::ImageConstPtr& msgSemC, const sensor_msgs::ImageConstPtr& msgSemConf)
{
    cout << "ROS-ImageGrabber->进入GrabSemtic()回调函数" << endl;
    cv_bridge::CvImageConstPtr cv_ptrSemL;
    try
    {
        cv_ptrSemL = cv_bridge::toCvShare(msgSemL);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrSemC;
    try
    {
        cv_ptrSemC = cv_bridge::toCvShare(msgSemC);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrSemConf;
    try
    {
        cv_ptrSemConf = cv_bridge::toCvShare(msgSemConf);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cout << "ROS-ImageGrabber->执行--TrackSemtic--" << endl;
    mpSLAM->TrackSemtic(cv_ptrSemL->image, cv_ptrSemC->image, cv_ptrSemConf->image);
}
//这是彩色图和深度图的回调
void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    cout << "ROS-ImageGrabber->进入GrabRGBD()回调函数" << endl;
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cout << "ROS-ImageGrabber->执行--TrackRGBD--" << endl;
    cv::Mat Tcw = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());

    if(Tcw.empty())
        return;
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);

    // Publish tf transform
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = cv_ptrRGB->header.stamp;
    transformStamped.header.frame_id = "camera_rgb_optical_frame";
    transformStamped.child_frame_id = "world";
    transformStamped.transform.translation.x = tcw.at<float>(0);
    transformStamped.transform.translation.y = tcw.at<float>(1);
    transformStamped.transform.translation.z = tcw.at<float>(2);
    vector<float> q = ORB_SLAM2::Converter::toQuaternion(Tcw.rowRange(0,3).colRange(0,3));
    transformStamped.transform.rotation.x = q[0];
    transformStamped.transform.rotation.y = q[1];
    transformStamped.transform.rotation.z = q[2];
    transformStamped.transform.rotation.w = q[3];

    br.sendTransform(transformStamped);
}


