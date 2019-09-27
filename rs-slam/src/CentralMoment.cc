/* This file is part of  RS-SLAM.
利用图像中心矩进一步判断潜在的动态目标
*/
#include "CentralMoment.h"
int thresh = 100;
float segthreshold = 150.0;
namespace CenMoment
{
	CentralMoment::CentralMoment()
	{

	}

    vector<Point2f> CentralMoment::ComputeCentralMoment(const cv::Mat& SemColor, const cv::Mat& SemLabel){
		/// 把原图像转化成灰度图像并进行平滑
        cvtColor( SemColor, semcolor_gray, CV_BGR2GRAY );
        blur( semcolor_gray, semcolor_gray, Size(3,3) );

		Mat canny_output;
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		/// 使用Canny检测边缘
		Canny( semcolor_gray, canny_output, thresh, thresh*2, 3 );
		findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
		/// 计算矩
        cout<<"计算矩 "<<endl;
		vector<Moments> mu(contours.size() );
		for( int i = 0; i < contours.size(); i++ )
		{
		    mu[i] = moments( contours[i], false );
		}
		///  计算中心矩:
        vector<Point2f> mc_people;
        vector<Point2f> mc_chair;
		vector<Point2f> mc( contours.size() );
		for( int i = 0; i < contours.size(); i++ )
		{
		    mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
            if(isnan(mu[i].m10/mu[i].m00) || isnan(mu[i].m01/mu[i].m00))
            {
                continue;
            }
            if (SemLabel.ptr<uchar>(int(mc[i].y))[int(mc[i].x)] == 31)
            {
                mc_people.push_back(mc[i]);
            }
            if (SemLabel.ptr<uchar>(int(mc[i].y))[int(mc[i].x)] == 5)
            {
                mc_chair.push_back(mc[i]);
            }
		}
        vector<Point2f> dynamic_chair;
        for(auto it : mc_people)
        {
            for(auto its : mc_chair)
            {
                double Distance = sqrt(pow((it.x-its.x),2)+pow((it.y-its.y),2));//计算人和椅子的轮廓中心欧式距离
                if (Distance<segthreshold)
                {
                    dynamic_chair.emplace_back(Point2f(its.x,its.y));
                }
            }
        }
        return dynamic_chair;
	}
}


