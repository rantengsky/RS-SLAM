/* This file is part of Dynamic Robust SLAM.
利用图像中心矩进一步判断潜在的动态目标
*/

#ifndef ORB_SLAM2_CENTRALMOMENT_H
#define ORB_SLAM2_CENTRALMOMENT_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

namespace CenMoment
{
class CentralMoment
{
    cv::Mat semcolor_gray;

public:
    CentralMoment();
    vector<Point2f> ComputeCentralMoment(const cv::Mat& SemColor, const cv::Mat& SemLabel);
};


}



#endif //ORB_SLAM2_CENTRALMOMENT_H
