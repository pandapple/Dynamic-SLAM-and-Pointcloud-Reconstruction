/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef POINTCLOUDMAPPING_H
#define POINTCLOUDMAPPING_H

// Note: PCL headers should be included in .cc file BEFORE this header
// to avoid 'detail' namespace conflict between OpenCV and VTK
// Only include PCL types that are needed in the header
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>
#include <memory>
#include <string>
#include <opencv2/core/core.hpp>
#include "Map.h"

// Forward declarations to avoid including System.h (which includes OpenCV) in header
namespace ORB_SLAM2 {
    class System;
    class KeyFrame;
}

class PointCloudMapping {
public:
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    PointCloudMapping(float resolution_, ORB_SLAM2::Map *map);

    // 启动点云建图线程（在YOLO初始化完成后调用）
    void start();

    // 插入一个keyframe，会更新一次地图
    void insertKeyFrame(ORB_SLAM2::KeyFrame *kf, cv::Mat &color, cv::Mat &depth);
    void insertKeyFrame(ORB_SLAM2::KeyFrame *kf);

    void shutdown();

    void viewer();

    void savePointCloud(const std::string &name);

    void generateMap();

protected:
    PointCloud::Ptr generatePointCloud(ORB_SLAM2::KeyFrame *kf, cv::Mat &color, cv::Mat &depth, cv::Mat& mask);
    
    // 基于体积的聚类过滤，去除小的条/块状物体（残留的人体特征）
    PointCloud::Ptr filterSmallClustersByVolume(PointCloud::Ptr cloud, double minVolume = 0.001, 
                                                 double clusterTolerance = 0.05, int minClusterSize = 20);

    PointCloud::Ptr globalMap;
    std::shared_ptr<std::thread> viewerThread;

    bool shutDownFlag = false;
    bool started = false;  // Flag to delay thread start until YOLO is initialized
    std::mutex shutDownMutex;

    std::condition_variable keyFrameUpdated;
    std::mutex keyFrameUpdateMutex;

    // data to generate point clouds
    std::vector<ORB_SLAM2::KeyFrame *> keyframes;
    std::vector<cv::Mat> colorImgs;
    std::vector<cv::Mat> depthImgs;
    std::mutex keyframeMutex;
    uint16_t lastKeyframeSize = 0;

    double resolution = 0.04;
    pcl::VoxelGrid<PointT> voxel;
    pcl::PassThrough<PointT> pass;

    ORB_SLAM2::Map *mpMap;
};

#endif // POINTCLOUDMAPPING_H
