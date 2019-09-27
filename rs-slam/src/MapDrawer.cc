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

#include "MapDrawer.h"
#include "MapPoint.h"
#include <pangolin/pangolin.h>
#include <typeinfo> //查看数据类型
#define PEOPLE_LABEL 31
namespace ORB_SLAM2
{
MapDrawer::MapDrawer(Map* pMap, const string &strSettingPath):
        mpMap(pMap),
        m_octree(NULL),
        m_maxRange(-1.0),
        m_useHeightMap(true),
        m_res(0.05),
        m_colorFactor(0.8),
        m_treeDepth(0),
        m_maxTreeDepth(0)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
    mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
    mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
    mPointSize = fSettings["Viewer.PointSize"];
    mCameraSize = fSettings["Viewer.CameraSize"];
    mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];

    m_res = fSettings["octoMap.res"];// octomap图精度
    m_occupy = fSettings["octoMap.occupy"];// octomap概率阈值
    m_depth = fSettings["octoMap.depth"];
    max_range = fSettings["octoMap.max_range"];
    raycast_range = fSettings["octoMap.raycast_range"];
    clamping_thres_min = fSettings["octoMap.clamping_thres_min"];
    clamping_thres_max = fSettings["octoMap.clamping_thres_max"];
    prob_hit = fSettings["octoMap.prob_hit"];
    prob_miss = fSettings["octoMap.prob_miss"];
    expansion = fSettings["octoMap.expansion"];
    cam_fx = fSettings["Camera.fx"];
    cam_fy = fSettings["Camera.fy"];
    cam_cx = fSettings["Camera.cx"];
    cam_cy = fSettings["Camera.cy"];
    cam_width = fSettings["Camera.width"];
    cam_height = fSettings["Camera.height"];

    punishment = fSettings["confidence.punish"];

    // initialize octomap
    m_octree = new octomap::ColorOcTree(m_res);// octomap图精度
    m_octree->setOccupancyThres(m_occupy);
    m_octree->setClampingThresMin(clamping_thres_min); // 这些参数都可以传进来===
    m_octree->setClampingThresMax(clamping_thres_max);
    m_octree->setProbHit(prob_hit);
    m_octree->setProbMiss(prob_miss);

    m_treeDepth = m_octree->getTreeDepth();
    m_maxTreeDepth = m_treeDepth;
    bIsLocalization = false;

    puber_octomap = nn.advertise<octomap_msgs::Octomap> ( "rs_slam/octomap", 1, true );
}

MapDrawer::~MapDrawer()
{
    delete m_octree;
}

void MapDrawer::DrawMapPoints()
{
    const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();

    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
    }
    glEnd();

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);

    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));

    }

    glEnd();
}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
{
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();

    if(bDrawKF)
    {
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            cv::Mat Twc = pKF->GetPoseInverse().t();

            glPushMatrix();

            glMultMatrixf(Twc.ptr<GLfloat>(0));

            glLineWidth(mKeyFrameLineWidth);
            glColor3f(0.0f,0.0f,1.0f);
            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();
        }
    }

    if(bDrawGraph)
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(0.0f,1.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        for(size_t i=0; i<vpKFs.size(); i++)
        {
            // Covisibility Graph
            const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            cv::Mat Ow = vpKFs[i]->GetCameraCenter();
            if(!vCovKFs.empty())
            {
                for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                {
                    if((*vit)->mnId<vpKFs[i]->mnId)
                        continue;
                    cv::Mat Ow2 = (*vit)->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                    glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
                }
            }

            // Spanning tree
            KeyFrame* pParent = vpKFs[i]->GetParent();
            if(pParent)
            {
                cv::Mat Owp = pParent->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
            }

            // Loops
            set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
            for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
            {
                if((*sit)->mnId<vpKFs[i]->mnId)
                    continue;
                cv::Mat Owl = (*sit)->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
            }
        }

        glEnd();
    }
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}

void MapDrawer::DrawGrid()
{
    glBegin(GL_LINES);// 画线 ======
    glLineWidth(1);       // 线宽======

    glColor3f(0.5,0.5,0.5); //gray  灰色线条
    int size =10;
    for(int i = -size; i <= size ; i++){
// xz 平面 垂直x轴直线  x轴上-10,-9,...,0,1,2,...,10 21条线，z方向范围， -10～10
        glVertex3f(i,0.6,  size);
        glVertex3f(i, 0.6, -size);

// xz 平面 垂直z轴直线z轴上 -10,-9,...,0,1,2,...,10  21条线，x方向范围， -10～10
        glVertex3f( size, 0.6, i);
        glVertex3f(-size, 0.6, i);
    }
    glEnd();
}

// Publish octomap, 在rviz中显示=========
void MapDrawer::PubOctoMap() {
    while (true)
    {
        if (CheckNewFrame())
        {
            std::chrono::steady_clock::time_point start_Octomap = std::chrono::steady_clock::now();
            KeyFrame *kf = GetNewKeyFrame();
            GeneratePointCloud(kf);

            // Publish octomap
            map_msg.header.frame_id = "world";
            map_msg.header.stamp = ros::Time::now();
            if (octomap_msgs::fullMapToMsg( *m_octree, map_msg ))
            {cout<<"发布 ROS-Octomap !"<<endl;
                puber_octomap.publish ( map_msg );}
            else
                ROS_ERROR("Error serializing OctoMap");

            std::chrono::steady_clock::time_point end_Octomap = std::chrono::steady_clock::now();
            double Octomap_consume= std::chrono::duration_cast<std::chrono::duration<double> >(end_Octomap - start_Octomap).count();
            cout << "wait for new Octomap time  =" << Octomap_consume*1000 << endl;
        }
    }

}

void MapDrawer::InsertFrame(KeyFrame *kf)
{
    std::unique_lock<mutex> lock ( mMutexNewKFs );
    if(kf->mnId!=0)
        mlNewKeyFrames.push_back(kf);
} // InsertKeyFrame

bool MapDrawer::CheckNewFrame()
{
    std::unique_lock<mutex> lock ( mMutexNewKFs );
    return ( !mlNewKeyFrames.empty() );
} // CheckNewFrame

KeyFrame* MapDrawer::GetNewKeyFrame()
{
    std::unique_lock<mutex> lock ( mMutexNewKFs );
    KeyFrame* kf = mlNewKeyFrames.front();
    mlNewKeyFrames.pop_front();
    return  kf;
} // GetNewKeyFrame


// 生成当前帧的点云
void MapDrawer::GeneratePointCloud(KeyFrame *kf)
{
    PointCloud::Ptr tmp( new PointCloud() );
    // Point cloud is null ptr
    for ( int m=0; m<kf->mImDep.rows; m+=1)
    {
        for ( int n=0; n<kf->mImDep.cols; n+=1)
        {
            float d = kf->mImDep.ptr<float>(m)[n];
            if (d < 0.01 || d > m_depth)
                continue;

            if((int)kf->mImLabel.ptr<uchar>(m)[n] != PEOPLE_LABEL && (int)kf->mImFmask.ptr<uchar>(m)[n] != 0)
            {
                    //语义分割膨胀，剔除Label边缘的特征点，进一步减小动态目标的影响
                if((int)kf->mImLabel.ptr<uchar>(m+expansion)[n+expansion] != PEOPLE_LABEL &&
                   (int)kf->mImLabel.ptr<uchar>(m+expansion)[n-expansion] != PEOPLE_LABEL &&
                   (int)kf->mImLabel.ptr<uchar>(m+expansion)[n] != PEOPLE_LABEL &&
                   (int)kf->mImLabel.ptr<uchar>(m-expansion)[n] != PEOPLE_LABEL &&
                   (int)kf->mImLabel.ptr<uchar>(m-expansion)[n+expansion] != PEOPLE_LABEL &&
                   (int)kf->mImLabel.ptr<uchar>(m-expansion)[n-expansion] != PEOPLE_LABEL &&
                        (int)kf->mImLabel.ptr<uchar>(m+expansion/2)[n+expansion/2] != PEOPLE_LABEL &&
                        (int)kf->mImLabel.ptr<uchar>(m+expansion/2)[n-expansion/2] != PEOPLE_LABEL &&
                        (int)kf->mImLabel.ptr<uchar>(m+expansion/2)[n] != PEOPLE_LABEL &&
                        (int)kf->mImLabel.ptr<uchar>(m-expansion/2)[n] != PEOPLE_LABEL &&
                        (int)kf->mImLabel.ptr<uchar>(m-expansion/2)[n+expansion/2] != PEOPLE_LABEL &&
                        (int)kf->mImLabel.ptr<uchar>(m-expansion/2)[n-expansion/2] != PEOPLE_LABEL &&
                    //光流检测膨胀，剔除Label边缘的特征点，进一步减小动态目标的影响
                   (int)kf->mImFmask.ptr<uchar>(m+expansion)[n+expansion] != 0 &&
                   (int)kf->mImFmask.ptr<uchar>(m+expansion)[n-expansion] != 0 &&
                   (int)kf->mImFmask.ptr<uchar>(m+expansion)[n] != 0 &&
                   (int)kf->mImFmask.ptr<uchar>(m-expansion)[n] != 0 &&
                   (int)kf->mImFmask.ptr<uchar>(m-expansion)[n+expansion] != 0 &&
                   (int)kf->mImFmask.ptr<uchar>(m-expansion)[n-expansion] != 0 &&
                        (int)kf->mImFmask.ptr<uchar>(m+expansion/2)[n+expansion/2] != 0 &&
                        (int)kf->mImFmask.ptr<uchar>(m+expansion/2)[n-expansion/2] != 0 &&
                        (int)kf->mImFmask.ptr<uchar>(m+expansion/2)[n] != 0 &&
                        (int)kf->mImFmask.ptr<uchar>(m-expansion/2)[n] != 0 &&
                        (int)kf->mImFmask.ptr<uchar>(m-expansion/2)[n+expansion/2] != 0 &&
                        (int)kf->mImFmask.ptr<uchar>(m-expansion/2)[n-expansion/2] != 0)
                {
                    PointT p;
                    p.z = d;
                    p.x = ( n - cam_cx) * p.z / cam_fx;
                    p.y = ( m - cam_cy) * p.z / cam_fy;

                    p.b = kf->mImColor.ptr<uchar>(m)[n*3];
                    p.g = kf->mImColor.ptr<uchar>(m)[n*3+1];
                    p.r = kf->mImColor.ptr<uchar>(m)[n*3+2];

                    p.label = kf->mImConf.ptr<uchar>(m)[n];
                    tmp->points.push_back(p);
                }

            }

        }
    }
    // 转换到世界坐标下
    Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat(kf->GetPose());
    PointCloud::Ptr cloud(new PointCloud);
    pcl::transformPointCloud( *tmp, *cloud, T.inverse().matrix());
    cloud->is_dense = false;
    octomap::point3d origin = octomap::point3d( T(0,3), T(1,3), T(2,3));// 点云原点
    // 过滤点云
    PointCloud::Ptr filtered(new PointCloud);
    voxel.setInputCloud( cloud );
    voxel.setLeafSize(0.2*m_res, 0.2*m_res, 0.2*m_res);
    voxel.filter( *filtered );
    //将新点云 插入到 octomap地图中
    if(!cloud->empty()){
        InsertScan(origin, *filtered);
    }
}

void MapDrawer::InsertScan(octomap::point3d origin,  // 点云原点
                           PointCloud &cloud)//点云
{
    //// The method of ZhangXuan=======================
    octomap::Pointcloud raycast_cloud; // Point cloud to be inserted with ray casting
    int endpoint_count = 0; // total number of endpoints inserted
    for(typename PointCloud::const_iterator it = cloud.begin(); it != cloud.end(); ++it)
    {
        // Check if the point is invalid
        if (!std::isnan(it->x) && !std::isnan(it->y) && !std::isnan(it->z))
        {
            float dist = sqrt((it->x - origin.x())*(it->x - origin.x()) + (it->y - origin.y())*(it->y - origin.y()) + (it->z - origin.z())*(it->z - origin.z()));
            // Check if the point is in max_range
            if(dist <= max_range)
            {
                // Check if the point is in the ray casting range
                if(dist <= raycast_range) // Add to a point cloud and do ray casting later all together
                {
                    raycast_cloud.push_back(it->x, it->y, it->z);
                }
                else // otherwise update the occupancy of node and transfer the point to the raycast range
                {
//                    cout<<"it: "<<" "<<it->x<<" "<<it->y<<" "<<it->z<<endl;
                    octomap::point3d direction = (octomap::point3d(it->x, it->y, it->z) - origin).normalized ();
//                    cout<<"direction: "<<direction<<endl;
                    octomap::point3d new_end = origin + direction * (raycast_range + m_octree->getResolution()*2);
//                    cout<<"new end: : "<<new_end<<endl;
                    raycast_cloud.push_back(new_end);
                    m_octree->updateNode(it->x, it->y, it->z, true, false); // use lazy_eval, run updateInnerOccupancy() when done
                }
                endpoint_count++;
            }
        }
    }
    // Do ray casting for points in raycast_range
    if(raycast_cloud.size() > 0)
//         use lazy_eval, run updateInnerOccupancy() when done, use discretize to downsample cloud
        m_octree->insertPointCloud(raycast_cloud, origin, raycast_range, false, true);
    // Update colors and semantics, differs between templates
    for(PointCloud::const_iterator it = cloud.begin(); it < cloud.end(); it++)
    {
        if (!std::isnan(it->x) && !std::isnan(it->y) && !std::isnan(it->z))
        {
            m_octree->setNodeColor(it->x, it->y, it->z, it->r, it->g, it->b);
            octomap::OcTreeKey key;
            octomap::point3d point(it->x, it->y, it->z);
            if (!m_octree->coordToKeyChecked(point, key)) continue;
            octomap::ColorOcTreeNode* n = m_octree->search(key);//能否在已经建立的八叉树中找到当前节点，若找到则视为旧节点，用来和当前语义值对比
            if (n != 0) {
                int r; int g; int b; //局部变量存储颜色
                if (n->getColor().b == it->b && n->getColor().g == it->g &&
                                n->getColor().r == it->r) {
                    b = it->b;
                    g = it->g;
                    r = it->r;
                    n->setValue((n->getValue()+it->label)/2.0);
                }
                else
                    {
                    if(n->getValue() > it->label){
                        b = n->getColor().b;
                        g = n->getColor().g;
                        r = n->getColor().r;
                        n->setValue(n->getValue());
                    } else{
                        b = it->b;
                        g = it->g;
                        r = it->r;
                        n->setValue(it->label * punishment);
                    }
                    }
                    n->setColor(r, g, b);
            }
        }
    }
    // updates inner node occupancy and colors
    if(endpoint_count > 0)
        m_octree->updateInnerOccupancy();
    //// The method of ZhangXuan=======================
}

// 保存地图为octomap=====
void MapDrawer::SaveOctoMap(const char *filename)
{
    std::ofstream outfile(filename, std::ios_base::out | std::ios_base::binary);
    if (outfile.is_open())
    {
        m_octree->write(outfile);
        outfile.close();
    }
}

void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    if(!mCameraPose.empty())
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
        }

        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;
    }
    else
        M.SetIdentity();
}


} //namespace ORB_SLAM