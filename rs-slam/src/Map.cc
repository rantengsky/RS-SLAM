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

#include "Map.h"

#include<mutex>
#include <sys/stat.h>
namespace ORB_SLAM2
{

Map::Map():mnMaxKFid(0),mnBigChangeIdx(0)
{
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
}

void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

void Map::clear()
{
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

// 保存 二进制 地图===================
bool Map::Save(const string &filename)
{
    cerr << "Map: Saving to " << filename << endl;
    ofstream f;
    f.open(filename.c_str(), ios_base::out|ios::binary);

    cerr << "  writing " << mspMapPoints.size() << " mappoints" << endl;
    unsigned long int nbMapPoints = mspMapPoints.size();// 地图点数量
    f.write((char*)&nbMapPoints, sizeof(nbMapPoints));  // 写地图点总数
// 写入 每一个地图点=====
    for(auto mp: mspMapPoints)
        _WriteMapPoint(f, mp);

    map<MapPoint*, unsigned long int> idx_of_mp; // 地图点:id 映射====
    unsigned long int i = 0;
    for(auto mp: mspMapPoints)
    {
        idx_of_mp[mp] = i; // 映射===
        i += 1;
    }

// 写每一个关键帧 =======
    cerr << "  writing " << mspKeyFrames.size() << " keyframes" << endl;
    unsigned long int nbKeyFrames = mspKeyFrames.size();
    f.write((char*)&nbKeyFrames, sizeof(nbKeyFrames)); // 写关键帧数量
    for(auto kf: mspKeyFrames)
        _WriteKeyFrame(f, kf, idx_of_mp);// 写入一个关键帧=====

// 保存 父关键帧id 共视关系 和 权重，最小生成树======
    // store tree and graph
    for(auto kf: mspKeyFrames) {
        KeyFrame* parent = kf->GetParent();// 父关键帧id
        unsigned long int parent_id = ULONG_MAX;
        if (parent) parent_id = parent->mnId;
        f.write((char*)&parent_id, sizeof(parent_id));

        unsigned long int nb_con = kf->GetConnectedKeyFrames().size();// 相关连的 关键帧
        f.write((char*)&nb_con, sizeof(nb_con));
        for (auto ckf: kf->GetConnectedKeyFrames())
        {
            int weight = kf->GetWeight(ckf); // 相关关系权重(共同看到的地图点数量)
            f.write((char*)&ckf->mnId, sizeof(ckf->mnId));
            f.write((char*)&weight, sizeof(weight));
        }
    }

    f.close();
    cerr << "Map: finished saving" << endl;
    struct stat st;
    stat(filename.c_str(), &st);// 获取文件大小
    cerr << "Map: saved " << st.st_size << " bytes" << endl;

#if 0
    for(auto mp: mspMapPoints)
if (!(mp->mnId%100))
  cerr << "mp " << mp->mnId << " " << mp->Observations() << " " << mp->isBad() << endl;
#endif

    return true;
}

// 写地图点=====================================
void Map::_WriteMapPoint(ofstream &f, MapPoint* mp)
{
    f.write((char*)&mp->mnId, sizeof(mp->mnId));               // id: long unsigned int
    cv::Mat wp = mp->GetWorldPos();// 位置 x,y,z
    f.write((char*)&wp.at<float>(0), sizeof(float));           // pos x: float
    f.write((char*)&wp.at<float>(1), sizeof(float));           // pos y: float
    f.write((char*)&wp.at<float>(2), sizeof(float));           // pos z: float
}

// 写关键帧
void Map::_WriteKeyFrame(ofstream &f, KeyFrame* kf, map<MapPoint*, unsigned long int>& idx_of_mp)
{
    f.write((char*)&kf->mnId, sizeof(kf->mnId));                 // 关键帧 id: long unsigned int
    f.write((char*)&kf->mTimeStamp, sizeof(kf->mTimeStamp));     // 时间戳 ts: TimeStamp, double

#if 0
    cerr << "writting keyframe id " << kf->mnId << " ts " << kf->mTimeStamp << " frameid " << kf->mnFrameId << " TrackReferenceForFrame " << kf->mnTrackReferenceForFrame << endl;
cerr << " parent " << kf->GetParent() << endl;
cerr << "children: ";
for(auto ch: kf->GetChilds())
cerr << " " << ch->mnId;
cerr <<endl;
cerr << kf->mnId << " connected: (" << kf->GetConnectedKeyFrames().size() << ") ";
for (auto ckf: kf->GetConnectedKeyFrames())
cerr << ckf->mnId << "," << kf->GetWeight(ckf) << " ";
cerr << endl;
#endif



    cv::Mat Tcw = kf->GetPose();// 关键帧位姿===
    f.write((char*)&Tcw.at<float>(0,3), sizeof(float));          // px: float
    f.write((char*)&Tcw.at<float>(1,3), sizeof(float));          // py: float
    f.write((char*)&Tcw.at<float>(2,3), sizeof(float));          // pz: float
    vector<float> Qcw = Converter::toQuaternion(Tcw.rowRange(0,3).colRange(0,3));
    f.write((char*)&Qcw[0], sizeof(float));                      // qx: float
    f.write((char*)&Qcw[1], sizeof(float));                      // qy: float
    f.write((char*)&Qcw[2], sizeof(float));                      // qz: float
    f.write((char*)&Qcw[3], sizeof(float));                      // qw: float

    f.write((char*)&kf->N, sizeof(kf->N));                       // 特征点数量 nb_features: int
    for (int i=0; i<kf->N; i++)
    {
        // 特征点==============
        cv::KeyPoint kp = kf->mvKeys[i];
        f.write((char*)&kp.pt.x,     sizeof(kp.pt.x));               // float
        f.write((char*)&kp.pt.y,     sizeof(kp.pt.y));               // float
        f.write((char*)&kp.size,     sizeof(kp.size));               // float
        f.write((char*)&kp.angle,    sizeof(kp.angle));              // float
        f.write((char*)&kp.response, sizeof(kp.response));           // float
        f.write((char*)&kp.octave,   sizeof(kp.octave));             // int
        // 描述子
        for (int j=0; j<32; j++)
            f.write((char*)&kf->mDescriptors.at<unsigned char>(i,j), sizeof(char));

        unsigned long int mpidx;
        MapPoint* mp = kf->GetMapPoint(i); // 地图点
        if (mp == NULL) mpidx = ULONG_MAX;
        else mpidx = idx_of_mp[mp]; // 所在所有地图点 map的 id
        f.write((char*)&mpidx,   sizeof(mpidx));  // long int
    }

}

} //namespace ORB_SLAM
