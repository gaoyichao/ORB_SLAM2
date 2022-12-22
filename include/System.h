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


#ifndef SYSTEM_H
#define SYSTEM_H

#include<string>
#include<thread>
#include<opencv2/core/core.hpp>

#include "Tracking.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "Viewer.h"

#include <unistd.h>

namespace ORB_SLAM2
{

class Viewer;
class FrameDrawer;
class Map;
class Tracking;
class LocalMapping;
class LoopClosing;

class System
{
public:
    // Input sensor
    enum eSensor{
        MONOCULAR=0,
        STEREO=1,
        RGBD=2
    };

public:

    //! @brief 构造函数
    //!
    //! 初始化 SLAM 系统，开启 Local Mapping, Loop Closing 和可视化线程.
    //!
    //! @param [in] strVocFile 词袋模型的字典文件路径
    //! @param [in] strSettingsFile 系统配置文件的路径
    //! @param [in] sensor 建图时使用的相机类型
    //! @param [in] bUseViewer 是否开启可视化程序
    System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer = true);

    //! @brief 跟踪双目相机. 左右目的图像必须是同步的.
    //!
    //! @param [in] imLeft 左目图像，可以是RGB彩色图(CV_8UC3)或者灰度图(CV_8U)，彩色图将被转换为灰度图后使用
    //! @param [in] imRight 右目图像，可以是RGB彩色图(CV_8UC3)或者灰度图(CV_8U)，彩色图将被转换为灰度图后使用
    //! @param [in] timestamp 时间戳
    //! @return 相机位姿，如果跟丢了将返回一个空的 cv::Mat.
    cv::Mat TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp);

    //! @brief 跟踪RGB-D相机. 深度图的像素必须与RGB图像对应.
    //!
    //! @param [in] im RGB图像可以是彩色图(CV_8UC3)或者灰度图(CV_8U)，彩色图将被转换为灰度图后使用
    //! @param [in] depthmap 深度图是32位的浮点数据(CV_32F).
    //! @param [in] timestamp 时间戳
    //! @return 相机位姿，如果跟丢了将返回一个空的 cv::Mat.
    cv::Mat TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp);

    //! @brief 跟踪单目相机
    //! @param [in] im RGB图像可以是彩色图(CV_8UC3)或者灰度图(CV_8U)，彩色图将被转换为灰度图后使用
    //! @param [in] timestamp 时间戳
    //! @return 相机位姿，如果跟丢了将返回一个空的 cv::Mat.
    cv::Mat TrackMonocular(const cv::Mat &im, const double &timestamp);

    //! @brief 停止更新地图只进行相机位姿跟踪
    void ActivateLocalizationMode();
    //! @brief 恢复地图更新进行完整的 SLAM
    void DeactivateLocalizationMode();

    //! @brief 距离上次调用该接口，地图是否因为闭环、全局BA发生了大的改变
    bool MapChanged();

    //! @brief 重置 SLAM 系统，清空地图
    void Reset();

    //! @brief 关闭 SLAM 系统，等待所有线程退出。
    //!
    //! 保存轨迹之前必须先调用该函数
    void Shutdown();

    //! @brief 按照 TUM RGB-D 数据集的格式保存相机轨迹。
    //!
    //! Only for stereo and RGB-D. This method does not work for monocular.
    //! Call first Shutdown()
    //! See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    //!
    //! @param [in] filename 轨迹文件名称
    void SaveTrajectoryTUM(const string &filename);

    //! @brief 按照 TUM RGB-D 数据集的格式保存关键帧的相机位姿
    //!
    //! This method works for all sensor input.
    //! Call first Shutdown()
    //! See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    //!
    //! @param [in] filename 轨迹文件名称
    void SaveKeyFrameTrajectoryTUM(const string &filename);

    //! @brief 按照 KITTI 数据集的格式保存相机轨迹
    //!
    //! Only for stereo and RGB-D. This method does not work for monocular.
    //! Call first Shutdown()
    //! See format details at: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
    //!
    //! @param [in] filename 轨迹文件名称
    void SaveTrajectoryKITTI(const string &filename);

    // TODO: Save/Load functions
    // SaveMap(const string &filename);
    // LoadMap(const string &filename);

    //! @brief 获取跟踪状态
    int GetTrackingState();
    //! @brief 获取跟踪的地图点
    std::vector<MapPoint*> GetTrackedMapPoints();
    //! @brief 获取校畸变后的特征点
    std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

private:
    //! @brief 传感器类型，内部嵌套定义的枚举类型，MONOCULAR=0, STEREO=1, RGBD=2
    eSensor mSensor;
    //! @brief DBoW2离线字典，通过词袋模型进行重定位和闭环检测
    ORBVocabulary* mpVocabulary;
    //! @brief 用于重定位和闭环检测的关键帧数据库
    KeyFrameDatabase* mpKeyFrameDatabase;
    //! @brief 系统的地图数据对象，保存所有关键帧和地图点
    Map* mpMap;

    //! @brief 轨迹跟踪器，通过输入的图像估计相机位姿，生成关键帧并创建新的地图点。
    //! 如果相机位姿跟丢了，将触发重定位。
    Tracking* mpTracker;
    //! @brief 局部地图管理器，管理一段时间内的局部地图并进行局部的BA(local bundle adjustment)。
    LocalMapping* mpLocalMapper;
    //! @brief 闭环检测器，新增关键帧时搜索闭环.如果检测到闭环就触发一次位姿图优化，
    //! 然后创建一个新的线程进行完整的BA(full bundle adjustment)。
    LoopClosing* mpLoopCloser;

    //! @brief 基于 Pangolin 的可视化对象，用于绘制地图和当前的相机位姿
    Viewer* mpViewer;
    //! @brief 关键帧渲染器，用于绘制关键帧(或者说是相机位姿)
    FrameDrawer* mpFrameDrawer;
    //! @brief 地图渲染器，用于绘制地图点
    MapDrawer* mpMapDrawer;

    //! @brief 进行 LOCAL MAPPING 的线程
    std::thread* mptLocalMapping;
    //! @brief 进行 LOOP CLOSING 的线程
    std::thread* mptLoopClosing;
    //! @brief 用于可视化的线程
    std::thread* mptViewer;

    //! @brief 保护成员变量 mbReset 的互斥信号量
    std::mutex mMutexReset;
    //! @brief 重置系统标志
    bool mbReset;

    //! @brief 保护工作模式标志的互斥信号量
    std::mutex mMutexMode;
    //! @brief 是否以纯定位模式运行，不理解为啥要弄两个这样的标志
    bool mbActivateLocalizationMode;
    bool mbDeactivateLocalizationMode;

    //! @brief 跟踪状态 
    int mTrackingState;
    //! @brief 当前帧参与估计相机位姿的地图点
    std::vector<MapPoint*> mTrackedMapPoints;
    //! @brief 当前帧中矫畸变(Undistorted)后的特征点
    std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
    //! @brief 保护跟踪状态的互斥信号量
    std::mutex mMutexState;
};

}// namespace ORB_SLAM

#endif // SYSTEM_H
