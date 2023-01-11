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

#ifndef LOOPCLOSING_H
#define LOOPCLOSING_H

#include "KeyFrame.h"
#include "LocalMapping.h"
#include "Map.h"
#include "ORBVocabulary.h"
#include "Tracking.h"

#include "KeyFrameDatabase.h"

#include <thread>
#include <mutex>
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM2
{

class Tracking;
class LocalMapping;
class KeyFrameDatabase;


class LoopClosing
{
public:

    //! @brief 一组具有一致共视关系的关键帧
    //!
    //! 第一个元素，ConsistentGroup.first 关键帧集合，称之为组
    //! 第二个元素，ConsistentGroup.second 与其他组具有连续关系的数量
    typedef pair<set<KeyFrame*>,int> ConsistentGroup;    

    //! @brief 关键帧和位姿映射关系
    //!
    //! @detail std::map 的模板有四个参数，分别是
    //! Key(map::key_type) 对应 KeyFrame*；
    //! T(map::mapped_type) 对应 g2o::Sim3；
    //! Compare(map::key_compare) 对应 std::less<KeyFrame*> 用于比较键值
    //! Alloc(map::allocator_type) 对应 Eigen::aligned_allocator<std::pair<KeyFrame *const, g2o::Sim3> >
    //! Eigen库做了一些加速操作，需要内存按照一定的字节对齐，所以专门提供了申请内存的方法
    typedef map<KeyFrame*,g2o::Sim3,std::less<KeyFrame*>,
        Eigen::aligned_allocator<std::pair<KeyFrame *const, g2o::Sim3> > > KeyFrameAndPose;

public:
    //! @brief 构造函数
    //!
    //! @param [in] pMap 地图对象
    //! @param [in] pDB  关键帧数据库
    //! @param [in] pVoc 特征字典对象
    //! @param [in] bFixScale 是否需要固定尺度
    //!                       对于双目和深度相机，因为尺度可以直接测量得到，所以是固定的。
    //!                       对于单目相机，由于存在尺度确实的问题，所以该参数为 false，需要在g2o优化时考虑尺度。
    LoopClosing(Map* pMap, KeyFrameDatabase* pDB, ORBVocabulary* pVoc,const bool bFixScale);

    //! @brief 设置轨迹跟踪器
    //! @param [in] pTracker 轨迹跟踪器
    void SetTracker(Tracking* pTracker);

    //! @brief 设置局部地图管理器
    //! @param [in] pLocalMapper 局部地图管理器
    void SetLocalMapper(LocalMapping* pLocalMapper);

    //! @brief LoopClosing 的线程函数
    void Run();

    //! @brief 插入关键帧
    //!
    //! @details 当局部地图规划器接受了某个关键帧之后，
    //! 调用该接口将新建的关键帧放到等待队列中。
    //! 闭环探测器将在其线程函数 Run 中消费等待队列，检测闭环
    //!
    //! @param [in] pKF 新的关键帧
    void InsertKeyFrame(KeyFrame *pKF);

    //! @brief 请求重置闭环探测器
    void RequestReset();

    //! @brief 进行全局的 BA 优化，该函数将独立开辟一个线程完成优化
    //!
    //! @param [in] nLoopKF 触发全局 BA 优化的关键帧 ID
    void RunGlobalBundleAdjustment(unsigned long nLoopKF);

    //! @brief 判定全局 BA 优化线程是否正在运行
    bool isRunningGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbRunningGBA;
    }

    //! @brief 判定全局 BA 优化是否已经结束
    bool isFinishedGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbFinishedGBA;
    }   

    //! @brief 请求终止闭环探测线程的工作
    void RequestFinish();

    //! @brief 判定闭环探测线程是否已经终止
    bool isFinished();

    // 因为 Eigen 库的加速优化，需要内存按照一定形式对齐，所以增加了该宏
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:

    //! @brief 等待队列中是否有新的关键帧
    bool CheckNewKeyFrames();

    //! @brief 探测回环
    bool DetectLoop();

    //! @brief 计算闭环的两帧之间的 Sim3 变换
    bool ComputeSim3();

    //! @brief 根据闭环和共视关系搜索地图点并融合
    void SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap);

    //! @brief 检测到闭环之后修正关键帧的位姿
    void CorrectLoop();

    //////////////////////////////////////////////
    //
    // 重置探测器
    //
    //////////////////////////////////////////////

    //! @brief 检查复位标识 mbResetRequested，若置位，则重置探测器
    void ResetIfRequested();
    //! @brief 当前系统是否有请求重置的信号
    bool mbResetRequested;
    //! @brief 保护重置信号的互斥量
    std::mutex mMutexReset;

    //////////////////////////////////////////////
    //
    // 终止闭环探测线程
    //
    //////////////////////////////////////////////

    //! @brief 检查结束标识 mbFinishRequested，若置位，则停止闭环探测线程
    bool CheckFinish();
    //! @brief 置位标识 mbFinished，终止线程
    void SetFinish();
    //! @brief 接收到结束信号的标识
    bool mbFinishRequested;
    //! @brief 闭环探测线程退出的标识
    bool mbFinished;
    //! @brief 保护终止相关信号的互斥量
    std::mutex mMutexFinish;

    //////////////////////////////////////////////
    //
    // 系统相关的成员变量
    //
    //////////////////////////////////////////////
 
    //! @brief 地图对象
    Map* mpMap;
    //! @brief 轨迹跟踪器
    Tracking* mpTracker;
    //! @brief 局部地图管理器
    LocalMapping *mpLocalMapper;

    //! @brief 关键帧数据库
    KeyFrameDatabase* mpKeyFrameDB;
    //! @brief 特征字典
    ORBVocabulary* mpORBVocabulary;


    //////////////////////////////////////////////
    //
    // 新关键帧
    //
    //////////////////////////////////////////////
 
    //! @brief 新关键帧的等待队列
    std::list<KeyFrame*> mlpLoopKeyFrameQueue;
    //! @brief 保护 mlpLoopKeyFrameQueue 的互斥量
    std::mutex mMutexLoopQueue;
    //! @brief 当前的关键帧
    KeyFrame* mpCurrentKF;
    //! @brief 检测到与当前帧闭环的关键帧
    KeyFrame* mpMatchedKF;

    //! @brief 闭环检测共视关系一致性判定阈值
    float mnCovisibilityConsistencyTh;
    //! @brief 上一个关键帧的一致共视组集合
    std::vector<ConsistentGroup> mvConsistentGroups;
    //! @brief 当前关键帧的闭环候选集合
    std::vector<KeyFrame*> mvpEnoughConsistentCandidates;

    //! @brief 当前关键帧的邻接关键帧
    std::vector<KeyFrame*> mvpCurrentConnectedKFs;
    //! @brief 当前关键帧中成功定位的地图点集合
    std::vector<MapPoint*> mvpCurrentMatchedPoints;
    //! @brief 闭环关键帧上的地图点集合
    std::vector<MapPoint*> mvpLoopMapPoints;
    //! @brief OpenCV 形式的 sim3 变换
    cv::Mat mScw;
    //! @brief 优化后当前帧的 sim3 变换
    g2o::Sim3 mg2oScw;

    //////////////////////////////////////////////
    //
    // 全局 BA 
    //
    //////////////////////////////////////////////
 
    //! @brief 上一次检测到闭环的关键帧ID
    long unsigned int mLastLoopKFid;

    //! @brief 全局 BA 优化正在进行中
    bool mbRunningGBA;
    //! @brief 全局 BA 优化已经结束
    bool mbFinishedGBA;
    //! @brief 暂停全局 BA ?
    bool mbStopGBA;
    //! @brief 保护全局 BA 的互斥量
    std::mutex mMutexGBA;
    //! @brief 进行全局 BA 的线程对象
    std::thread* mpThreadGBA;

    //! @brief 是否固定尺度，双目和深度相机需要固定
    bool mbFixScale;
    //! @brief 进行 Full BA 优化的次数
    bool mnFullBAIdx;
};

} //namespace ORB_SLAM

#endif // LOOPCLOSING_H
