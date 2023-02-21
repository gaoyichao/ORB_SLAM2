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
#ifndef INITIALIZER_H
#define INITIALIZER_H

#include<opencv2/opencv.hpp>
#include "Frame.h"


namespace ORB_SLAM2
{


    //! @brief 用于单目初始化的辅助类, 双目和深度相机是不要它的
    class Initializer
    {
        typedef pair<int,int> Match;
    
    public:
    
        //! @brief 构造函数
        //! @param [in] ReferenceFrame 参考帧引用
        //! @param [in] sigma 测量标准差
        //! @param [in] iterations 最大迭代次数
        Initializer(const Frame & ReferenceFrame, float sigma = 1.0, int iterations = 200);
    
        //! @brief 单目相机初始化
        //!
        //! 在两个线程中同时计算基础矩阵和单应矩阵，
        //! 根据 R_H = \frac{S_H}{S_H + S_F} 选择其中一个模型，恢复相机的位姿，并对匹配点对进行三角测量
        //!
        //! @param [in] CurrentFrame 当前帧
        //! @param [in] vMatches12 当前帧与参考帧之间特征点匹配关系, 与当前帧的特征点一一对应，记录匹配点在参考帧中的索引
        //! @param [out] R21 当前帧相对于参考帧的旋转矩阵
        //! @param [out] t21 当前帧相对于参考帧的平移向量
        //! @param [out] vP3D 三角测量后的地图点坐标
        //! @param [out] vbTriangulated 当前帧中特征点是否成功三角测量
        //! @return true 成功完成初始化
        //! @return false 不成功
        bool Initialize(const Frame &CurrentFrame, const vector<int> &vMatches12,
                        cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated);
    private:
        //! @brief 计算单应矩阵
        //!
        //! @param [inout] vbMatchesInliers 内点标记
        //! @param [out] score 重投影误差评分
        //! @param [out] H21 单应矩阵
        void FindHomography(std::vector<bool> &vbMatchesInliers, float &score, cv::Mat &H21);

        //! @brief DLT 算法求解单应矩阵
        //!
        //! @param [in] vP1 参考帧中归一化后的 8 个特征点
        //! @param [in] vP2 当前帧中归一化后的 8 个特征点
        cv::Mat ComputeH21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2);


        //! @brief 计算基础矩阵
        //!
        //! @param [inout] vbInliers 内点标记
        //! @param [out] score 重投影误差评分
        //! @param [out] F21 基础矩阵
        void FindFundamental(vector<bool> &vbInliers, float &score, cv::Mat &F21);

        cv::Mat ComputeF21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2);
    
        /** @brief 归一化特征点
           
            对原始特征点 \f$ \boldsymbol{x} = \begin{bmatrix} u & v & 1 \end{bmatrix}^T \f$ 进行平移和缩放，
            使得 \f$ \boldsymbol{x'} = \begin{bmatrix} u' & v' & 1 \end{bmatrix}^T \f$ 的质心为 \f$ \begin{bmatrix} 0 & 0 & 1 \end{bmatrix} \f$，
            各点到质心的平均距离为\f$ \sqrt{2} \f$。
            \f[
                \begin{bmatrix} u' \\ v' \\ 1 \end{bmatrix} = T \begin{bmatrix} u \\ v \\ 1 \end{bmatrix}
            \f]
           
            @param [in]  vKeys             原始特征点集合
            @param [out] vNormalizedPoints 归一化后的特征点坐标
            @param [out] T                 归一化操作的 3x3 变换矩阵
         */
        void Normalize(const vector<cv::KeyPoint> &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T);

        float CheckHomography(const cv::Mat &H21, const cv::Mat &H12, vector<bool> &vbMatchesInliers, float sigma);
    
        float CheckFundamental(const cv::Mat &F21, vector<bool> &vbMatchesInliers, float sigma);
    
        bool ReconstructF(vector<bool> &vbMatchesInliers, cv::Mat &F21, cv::Mat &K,
                          cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated);
    
        /** @brief 分解单应矩阵，计算相机位姿，三角测量特征点
          
            参考文章 Faugeras et al. Motion and structure from motion in a piecewise planar environment.
            International Journal of Pattern Recognition and Artificial Intelligence, 1988
          
            @param [in]  vbMatchesInliers 匹配点对为内点的标记
            @param [in]  H21              函数 FindHomography() 输出的单应矩阵
            @param [in]  K                相机的内参矩阵
            @param [out] R21              输出旋转矩阵
            @param [out] t21              输出平移向量
            @param [out] vP3D             对特征点进行三角测量后输出的世界坐标
            @param [out] vbTriangulated   特征点是否成功三角测量了
            @param [in]  minParallax      最小视差角
            @param [in]  minTriangulated  成功完成三角测量的最少特征点数
         */
        bool ReconstructH(vector<bool> &vbMatchesInliers,
                          cv::Mat &H21, cv::Mat &K,
                          cv::Mat &R21, cv::Mat &t21,
                          vector<cv::Point3f> &vP3D,
                          vector<bool> &vbTriangulated,
                          float minParallax, int minTriangulated);
    
        void Triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D);
    
    
        int CheckRT(const cv::Mat &R, const cv::Mat &t, const vector<cv::KeyPoint> &vKeys1, const vector<cv::KeyPoint> &vKeys2,
                           const vector<Match> &vMatches12, vector<bool> &vbInliers,
                           const cv::Mat &K, vector<cv::Point3f> &vP3D, float th2, vector<bool> &vbGood, float &parallax);
    
        void DecomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t);

        void SampleForRansec();
    
    
        //! @brief 参考帧的特征点 (Frame 1)
        vector<cv::KeyPoint> mvKeys1;
    
        //! @brief 当前帧的特征点 (Frame 2)
        vector<cv::KeyPoint> mvKeys2;
    
        //! @brief 匹配点对 first 参考帧特征点索引，second 当前帧特征点索引
        vector<Match> mvMatches12;
        //! @brief 对应参考帧中每个特征点，在当前帧中是否有匹配点
        vector<bool> mvbMatched1;
    
        //! @brief 相机内参矩阵 3x3
        cv::Mat mK;
    
        //! @brief 标准差
        float mSigma;
        //! @brief 方差
        float mSigma2;
    
        //! @brief RANSAC 最大迭代次数
        int mMaxIterations;
    
        //! @brief RANSAC 样本组集合
        vector<vector<size_t> > mvSets;   
    
    };

} //namespace ORB_SLAM

#endif // INITIALIZER_H
