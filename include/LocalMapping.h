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

#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "KeyFrame.h"
#include "Map.h"
#include "LoopClosing.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"

#include <mutex>


namespace ORB_SLAM2
{

class Tracking;
class LoopClosing;
class Map;

class LocalMapping
{
public:
    //! @brief 构造函数
    //!
    //! @param [in] pMap 地图对象
    //! @param [in] bMonocular 是否使用单目相机
    LocalMapping(Map* pMap, const float bMonocular);

    //! @brief 设置闭环探测器
    //! @param [in] pLoopCloser 闭环探测器
    void SetLoopCloser(LoopClosing* pLoopCloser);

    //! @brief 设置轨迹跟踪器
    //! @param [in] pTracker 轨迹跟踪器
    void SetTracker(Tracking* pTracker);

    //! @brief LocalMapping 的线程函数
    void Run();

    //! @brief 插入关键帧
    //!
    //! @details 一般由轨迹跟踪器根据一定规则生成关键帧，
    //! 调用该接口将新建的关键帧放到等待队列中。
    //! 局部地图管理器将在其线程函数 Run 中进一步对关键帧进行筛选，
    //! 插入到地图中，进行 Local BA 优化。
    //!
    //! @param [in] pKF 新的关键帧
    void InsertKeyFrame(KeyFrame* pKF);

    //////////////////////////////////////////////
    //
    // 一些线程同步的接口
    //
    //////////////////////////////////////////////
 
    //! @brief 请求暂停更新地图
    void RequestStop();
    //! @brief 请求重置局部地图管理器
    void RequestReset();
    //! @brief 请求终止局部地图管理线程的工作
    void RequestFinish();

    //! @brief 设置 mbNotStop 的状态，屏蔽暂停线程的功能
    bool SetNotStop(bool flag);

    //! @brief 停止当前正在进行的 Local BA 优化
    void InterruptBA();

    //! @brief 线程函数 Run 调用，检查是否需要停止局部建图
    bool Stop();

    //! @brief 释放等待队列中的关键帧
    void Release();

    //! @brief 检查 mbStopped 是否置位
    bool isStopped();

    //! @brief 检查 mbStopRequested 是否置位
    bool stopRequested();

    //! @brief 检查 mbAcceptKeyFrames 是否置位，既是否接收新的关键帧
    bool AcceptKeyFrames();

    //! @brief 设置 mbAcceptKeyFrames 的状态，控制关键帧的接收
    void SetAcceptKeyFrames(bool flag);

    //! @brief 检查 mbFinished 是否置位，既管理线程是否终止运行
    bool isFinished();

    //! @brief 查询等待队列中新的关键帧数量
    int KeyframesInQueue(){
        unique_lock<std::mutex> lock(mMutexNewKFs);
        return mlNewKeyFrames.size();
    }

protected:

    //! @brief 等待队列中是否有新的关键帧
    bool CheckNewKeyFrames();

    //! @brief 处理新的关键帧，插入新的关键帧，更新共视图
    void ProcessNewKeyFrame();

    //! @brief 根据共视关系三角化，生成地图点
    void CreateNewMapPoints();

    //! @brief 剔除质量较差的地图点
    void MapPointCulling();

    //! @brief 根据关键帧的两级邻接关系，更新共视的地图点坐标
    void SearchInNeighbors();

    //! @brief 剔除质量较差的关键帧
    void KeyFrameCulling();

    //! @brief 计算两个关键帧之间的基础矩阵
    //!
    //! @param [in] pKF1 关键帧1
    //! @param [in] pKF2 关键帧2
    //! @return 3x3的基础矩阵
    cv::Mat ComputeF12(KeyFrame* &pKF1, KeyFrame* &pKF2);

    //! @brief 三维向量的斜对称矩阵形式
    //!
    //! @param [in] v 三维向量
    //! @return 3x3的斜对称矩阵
    cv::Mat SkewSymmetricMatrix(const cv::Mat &v);

    //////////////////////////////////////////////
    //
    // 重置管理器
    //
    //////////////////////////////////////////////

    //! @brief 检查复位标识 mbResetRequested，若置位，则重置管理器
    void ResetIfRequested();
    //! @brief 当前系统是否有请求重置的信号
    bool mbResetRequested;
    //! @brief 保护重置信号的互斥量
    std::mutex mMutexReset;

    //////////////////////////////////////////////
    //
    // 终止建图
    //
    //////////////////////////////////////////////
    
    //! @brief 检查结束标识 mbFinishRequested，若置位，则停止局部建图的线程
    bool CheckFinish();
    //! @brief 置位标识 mbFinished，终止线程
    void SetFinish();
    //! @brief 接收到结束信号的标识
    bool mbFinishRequested;
    //! @brief 局部建图线程退出的标识
    bool mbFinished;
    //! @brief 保护终止相关信号的互斥量
    std::mutex mMutexFinish;

    //////////////////////////////////////////////
    //
    // 系统相关的成员变量
    //
    //////////////////////////////////////////////
 
    //! @brief 当前系统是否使用单目相机建图
    bool mbMonocular;
    //! @brief 地图对象
    Map* mpMap;
    //! @brief 轨迹跟踪器
    Tracking* mpTracker;
    //! @brief 闭环探测器
    LoopClosing* mpLoopCloser;

    //////////////////////////////////////////////
    //
    // 新关键帧
    //
    //////////////////////////////////////////////
    
    //! @brief 新关键帧的等待队列
    std::list<KeyFrame*> mlNewKeyFrames;
    //! @brief 当前正在处理的关键帧
    KeyFrame* mpCurrentKeyFrame;
    //! @brief 记录新生成的地图点，待筛查
    std::list<MapPoint*> mlpRecentAddedMapPoints;
    //! @brief 保护 mlNewKeyFrames 的互斥量
    std::mutex mMutexNewKFs;


    //////////////////////////////////////////////
    //
    // 暂停建图
    //
    //////////////////////////////////////////////
    
    //! @brief 暂停更新局部地图的标识
    bool mbStopped;
    //! @brief 接收到暂停更新的标识
    bool mbStopRequested;
    //! @brief mbStopRequested 是否有效的标识
    bool mbNotStop;
    //! @brief 保护暂停相关标识的互斥量
    std::mutex mMutexStop;

    //////////////////////////////////////////////

    //! @brief 放弃当前 BA 优化的标识
    bool mbAbortBA;
    //! @brief 是否可以接收新的关键帧的标识
    bool mbAcceptKeyFrames;
    //! @brief 保护 mbAcceptKeyFrames 的互斥量
    std::mutex mMutexAccept;
};

} //namespace ORB_SLAM

#endif // LOCALMAPPING_H
