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


#ifndef TRACKING_H
#define TRACKING_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"Viewer.h"
#include"FrameDrawer.h"
#include"Map.h"
#include"LocalMapping.h"
#include"LoopClosing.h"
#include"Frame.h"
#include "ORBVocabulary.h"
#include"KeyFrameDatabase.h"
#include"ORBextractor.h"
#include "Initializer.h"
#include "MapDrawer.h"
#include "System.h"

#include <mutex>

namespace ORB_SLAM2
{

class Viewer;
class FrameDrawer;
class Map;
class LocalMapping;
class LoopClosing;
class System;

class Tracking
{  

public:

    //! @brief 构造函数
    //!
    //! @param [in] pSys           SLAM 系统对象
    //! @param [in] pVoc           字典对象
    //! @param [in] pFrameDrawer   帧渲染器
    //! @param [in] pMapDrawer     地图渲染器
    //! @param [in] pKFDB          关键帧数据库对象
    //! @param [in] strSettingPath 配置文件路径
    //! @param [in] sensor         相机类型 MONOCULAR=0, STEREO=1, RGBD=2
    Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor);

    //! @brief 预处理输入图像数据，调用 Track 函数完成相机位姿跟踪
    //!
    //! 提取左右目图像特征，完成双目匹配，实现双目相机的跟踪
    //!
    //! @param [in] imRectLeft  校畸变之后的左目图像
    //! @param [in] imRectRight 校畸变之后的右目图像
    //! @param [in] timestamp   时间戳
    //! @return 相机位姿
    cv::Mat GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp);

    //! @brief 预处理输入图像数据，调用 Track 函数完成相机位姿跟踪
    //!
    //! 提取RGB图像特征，结合深度图，实现深度相机的跟踪
    //!
    //! @param [in] imRGB     彩色图或灰度图
    //! @param [in] imD       深度图
    //! @param [in] timestamp 时间戳
    //! @return 相机位姿
    cv::Mat GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp);

    //! @brief 预处理输入图像数据，调用 Track 函数完成相机位姿跟踪
    //!
    //! 提取RGB图像特征，实现单目相机的跟踪
    //!
    //! @param [in] imRGB     彩色图或灰度图
    //! @param [in] timestamp 时间戳
    //! @return 相机位姿
    cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);

    //! @brief 设置局部地图管理器
    //! @param [in] pLocalMapper 局部地图管理器
    void SetLocalMapper(LocalMapping* pLocalMapper);

    //! @brief 设置闭环探测器
    //! @param [in] pLoopClosing 闭环探测器
    void SetLoopClosing(LoopClosing* pLoopClosing);

    //! @brief 设置可视化对象
    //! @param [in] pViewer 可视化对象
    void SetViewer(Viewer* pViewer);

    //! @brief 加载新的配置
    //!
    //! The focal lenght should be similar or scale prediction will fail when projecting points
    //! @todo: Modify MapPoint::PredictScale to take into account focal lenght
    //!
    //! @param [in] strSettingPath 配置文件路径
    void ChangeCalibration(const string &strSettingPath);

    //! @brief 进入纯定位模式
    //!
    //! @param [in] flag 纯定位标志
    void InformOnlyTracking(const bool &flag);

    //! @brief 复位系统
    void Reset();

public:

    //! 跟踪状态
    enum eTrackingState{
        SYSTEM_NOT_READY = -1,  //! 系统尚未准备好
        NO_IMAGES_YET    =  0,  //! 还没有收到图片
        NOT_INITIALIZED  =  1,  //! 尚未初始化
        OK               =  2,  //! 一切正常
        LOST             =  3   //! 跟丢了
    };

    //! @brief 当前跟踪状态
    eTrackingState mState;
    //! @brief 上次的跟踪状态
    eTrackingState mLastProcessedState;

    //! @brief 传感器类型，MONOCULAR=0, STEREO=1, RGBD=2
    int mSensor;

    //! @brief 当前帧 
    Frame mCurrentFrame;
    //! @brief 灰度图，如果是双目相机，则对应左目的图像
    cv::Mat mImGray;

    //////////////////////////////////////////////
    //
    // 用于单目初始化的相关成员变量
    //
    //////////////////////////////////////////////
    
    //! @brief 用于初始化的参考帧，简称初始帧
    Frame mInitialFrame;
    //! @brief 记录初始帧中各个特征点与当前帧匹配的特征点索引
    std::vector<int> mvIniMatches;
    //! @brief 记录初始帧与上一帧的匹配特征点关系
    std::vector<int> mvIniLastMatches;
    //! @brief 记录初始化过程中，上一次匹配的特征点坐标
    //! 其实并不知道它到底有什么用，只在调用matcher.SearchForInitialization时有用到。
    std::vector<cv::Point2f> mvbPrevMatched;
    //! @brief 记录初始化过程中，成功三角化的特征点的3D坐标
    std::vector<cv::Point3f> mvIniP3D;

    //////////////////////////////////////////////
    //
    // 用于结束程序的时候恢复完整的相机轨迹。
    // 为每一帧记录其参考关键帧和相对变换。
    //
    //////////////////////////////////////////////
    
    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation

    //! @brief 参考关键帧的位姿列表
    list<cv::Mat> mlRelativeFramePoses;
    //! @brief 参考关键帧列表
    list<KeyFrame*> mlpReferences;
    //! @brief 关键帧的时间戳
    list<double> mlFrameTimes;
    //! @brief 对应关键帧是否跟丢了
    list<bool> mlbLost;

    //! 只用于定位的标志，不进行局部建图(local mapping)
    bool mbOnlyTracking;

protected:

    //! @brief 具体执行相机位姿跟踪任务的函数
    //! 它的运行过程只用到了特征点，与输入的传感器类型没有关系。
    void Track();

    //! @brief 双目相机和深度相机的初始化函数
    void StereoInitialization();
    //! @brief 单目相机的初始化函数
    void MonocularInitialization();
    //! @brief 生成单目相机所用的初始地图
    void CreateInitialMapMonocular();

    //! @brief 检查上一帧中是否有地图点被替换掉
    void CheckReplacedInLastFrame();

    //! @brief 根据参考关键帧估计相机位姿
    bool TrackReferenceKeyFrame();
    //! @brief 主要用于纯定位模式
    void UpdateLastFrame();
    //! @brief 根据匀速模型估计相机位姿
    bool TrackWithMotionModel();
    //! @brief 根据词袋模型进行重定位
    bool Relocalization();

    //! @brief 更新局部地图
    void UpdateLocalMap();
    //! @brief 更新局部地图点
    void UpdateLocalPoints();
    //! @brief 更新局部的关键帧
    void UpdateLocalKeyFrames();

    //! @brief 更新局部地图并优化相机位姿
    bool TrackLocalMap();
    //! @brief 筛选局部地图点
    void SearchLocalPoints();

    //! @brief 判定是否需要新建关键帧
    bool NeedNewKeyFrame();
    //! @brief 新建关键帧
    void CreateNewKeyFrame();


    //! @brief 标志着地图中没有找到相应的匹配点(只适用于纯定位的模式)
    //!
    //! 如果该标志为 false，说明还有足够多的匹配点支持视觉里程计运行。
    //! 如果该标志为 true, 基本上就跟丢了，系统将尝试进行重定位，来消除定位偏差(zero-drift)。
    //!
    //! In case of performing only localization, this flag is true when there are no matches to
    //! points in the map. Still tracking will continue if there are enough matches with temporal points.
    //! In that case we are doing visual odometry. The system will try to do relocalization to recover
    //! "zero-drift" localization to the map.
    //! 
    bool mbVO;

    //////////////////////////////////////////////
    //
    // ORB-SLAM2 系统中其它子系统对象。
    //
    //////////////////////////////////////////////
    
    //! @brief 局部地图管理器指针
    LocalMapping* mpLocalMapper;
    //! @brief 闭环检测器指针
    LoopClosing* mpLoopClosing;
    //! @brief ORB-SLAM2 系统对象指针
    System* mpSystem;
    //! @brief 可视化对象
    Viewer* mpViewer;
    //! @brief 帧渲染器
    FrameDrawer* mpFrameDrawer;
    //! @brief 地图渲染器
    MapDrawer* mpMapDrawer;
    //! @brief 地图初始化器，只用于单目相机
    Initializer* mpInitializer;

    //////////////////////////////////////////////
    //
    // ORB 特征提取器
    //
    //////////////////////////////////////////////
    
    //! @brief 单目、深度相机的特征提取器
    ORBextractor* mpORBextractorLeft;
    //! @brief 对于双目相机还需要再提供一个特征提取器
    ORBextractor* mpORBextractorRight;
    //! @brief 专门用于单目初始化的特征提取器
    ORBextractor* mpIniORBextractor;

    //! @brief 词袋模型的特征字典
    ORBVocabulary* mpORBVocabulary;
    //! @brief 关键帧数据库
    KeyFrameDatabase* mpKeyFrameDB;
    //! @brief 地图对象
    Map* mpMap;

    //////////////////////////////////////////////
    //
    // 一些系统配置
    //
    //////////////////////////////////////////////
 
    //! @brief 相机的内参矩阵
    cv::Mat mK;
    //! @brief 相机的畸变系数
    cv::Mat mDistCoef;
    //! @brief 相机基线
    //! 对于双目就是两个相机之间的距离。
    //! 对于RGBD相机，ORB SLAM2给定了一个假想的基线，模拟双目的数据。
    float mbf;

    //! 创建关键帧所需经历的最小帧数量
    int mMinFrames;
    //! 创建关键帧所需经历的最大帧数量
    int mMaxFrames;

    //! @brief 远近点判定阈值 
    //!
    //! 对于双目或者深度相机，近点的深度是比较可信的，可以直接获取定位；
    //! 而远处的点则需要在两个关键帧的匹配三角化得到具体的 3D 坐标。
    //!
    //! Points seen as close by the stereo/RGBD sensor are considered reliable
    //! and inserted from just one frame. Far points requiere a match in two keyframes.
    //!
    float mThDepth;

    //! @brief 深度图比例系数，只适用于深度相机
    float mDepthMapFactor;

    //////////////////////////////////////////////
    //
    // 相机轨迹跟踪的一些计算过程变量
    //
    //////////////////////////////////////////////
 
    //! @brief 当前帧中匹配点的数量
    int mnMatchesInliers;

    //! @brief 上一个关键帧
    KeyFrame* mpLastKeyFrame;
    //! @brief 上一帧数据
    Frame mLastFrame;
    //! @brief 上一个关键帧的ID
    unsigned int mnLastKeyFrameId;
    //! @brief 上次发生重定位的帧ID
    unsigned int mnLastRelocFrameId;

    //! @brief 相机的运动速度估计
    //! ORB-SLAM2 通过匀速模型估计相机位姿，提供位姿估计迭代计算的初值
    cv::Mat mVelocity;

    //! @brief RGB图像的颜色通道顺序，true: RGB, false: BGR
    bool mbRGB;

    //! @brief 当前参考关键帧
    KeyFrame* mpReferenceKF;
    //! @brief 局部地图中关键帧列表
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    //! @brief 局部地图中地图点列表
    std::vector<MapPoint*> mvpLocalMapPoints;
    //! @brief 一个临时的地图点列表
    list<MapPoint*> mlpTemporalPoints;
};

} //namespace ORB_SLAM

#endif // TRACKING_H
