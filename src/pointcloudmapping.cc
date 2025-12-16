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

// CRITICAL: Include ALL PCL headers FIRST, before any OpenCV headers
// This prevents 'detail' namespace conflict between OpenCV (cv::detail) and VTK (detail)
// VTK is used by PCL visualization, and both define a 'detail' namespace
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>

// Standard library headers
#include <iostream>
#include <limits>
#include <algorithm>

// Now include our headers (which may include OpenCV)
#include "pointcloudmapping.h"
#include <KeyFrame.h>
#include "Converter.h"
#include "System.h"

#include <boost/make_shared.hpp>

PointCloudMapping::PointCloudMapping(float resolution_, ORB_SLAM2::Map *map) : mpMap(map)
{
    this->resolution = resolution_;
    voxel.setLeafSize(resolution, resolution, resolution);
    //pass.setFilterFieldName("z");
    //pass.setFilterLimits(0.0, 3.0);
    globalMap = boost::make_shared<PointCloud>();
    
    // Don't start viewer thread immediately - wait for start() to be called
    // after YOLO initialization is complete
    started = false;
    viewerThread = nullptr;
}

void PointCloudMapping::start()
{
    if (!started)
    {
        started = true;
        viewerThread = std::make_shared<std::thread>(std::bind(&PointCloudMapping::viewer, this));
        std::cout << "Point cloud mapping thread started." << std::endl;
    }
}

void PointCloudMapping::shutdown()
{
    {
        std::unique_lock<std::mutex> lck(shutDownMutex);
        shutDownFlag = true;
        keyFrameUpdated.notify_one();
    }
    if (viewerThread && viewerThread->joinable())
    {
        viewerThread->join();
    }
}

void PointCloudMapping::insertKeyFrame(ORB_SLAM2::KeyFrame *kf, cv::Mat &color, cv::Mat &depth)
{
    std::cout << kf->mTimeStamp << " receive a keyframe, id = " << kf->mnId << std::endl;
    std::unique_lock<std::mutex> lck(keyframeMutex);
    keyframes.push_back(kf);
    colorImgs.push_back(color.clone());
    depthImgs.push_back(depth.clone());

    keyFrameUpdated.notify_one();
}

void PointCloudMapping::insertKeyFrame(ORB_SLAM2::KeyFrame *kf)
{
    std::cout << kf->mTimeStamp << " receive a keyframe, id = " << kf->mnId << std::endl;
    std::unique_lock<std::mutex> lck(keyframeMutex);

    keyFrameUpdated.notify_one();
}

pcl::PointCloud<PointCloudMapping::PointT>::Ptr
PointCloudMapping::generatePointCloud(ORB_SLAM2::KeyFrame *kf, cv::Mat &color, cv::Mat &depth, cv::Mat &mask)
{
    PointCloud::Ptr tmp(new PointCloud());
    
    // Check if mask is valid and resize if needed
    cv::Mat validMask;
    if(mask.empty())
    {
        // If mask is empty, create a mask with all ones
        validMask = cv::Mat::ones(depth.size(), CV_8UC1) * 255;
    }
    else
    {
        validMask = mask.clone();
        // Resize mask to match depth image size if needed
        if(validMask.size() != depth.size())
        {
            cv::resize(validMask, validMask, depth.size(), 0, 0, cv::INTER_NEAREST);
        }
        // Ensure mask is single channel
        if(validMask.channels() > 1)
        {
            cv::cvtColor(validMask, validMask, cv::COLOR_BGR2GRAY);
        }
    }
    
    // point cloud is null ptr
    for (int m = 0; m < depth.rows; m += 3)
    {
        for (int n = 0; n < depth.cols; n += 3)
        {
            // Check mask bounds and value
            if(m >= 0 && m < validMask.rows && n >= 0 && n < validMask.cols)
            {
                if(validMask.at<uchar>(m, n) == 0)
                {
                    continue; // Skip points in person detection boxes
                }
            }
            
            float d = depth.ptr<float>(m)[n];
            if (d < 0.01 || d > 10)
                continue;
            PointT p;
            p.z = d;
            p.x = (n - kf->cx) * p.z / kf->fx;
            p.y = (m - kf->cy) * p.z / kf->fy;

            // Extract color from color image with proper bounds checking
            // Color image should be BGR format with 3 channels
            if(color.channels() == 3 && m >= 0 && m < color.rows && n >= 0 && n < color.cols)
            {
                // Direct pixel access for BGR image
                cv::Vec3b pixel = color.at<cv::Vec3b>(m, n);
                p.b = pixel[0];  // Blue
                p.g = pixel[1];  // Green
                p.r = pixel[2];  // Red
            }
            else if(color.channels() == 1 && m >= 0 && m < color.rows && n >= 0 && n < color.cols)
            {
                // Grayscale image - convert to RGB
                uchar gray = color.at<uchar>(m, n);
                p.b = gray;
                p.g = gray;
                p.r = gray;
            }
            else
            {
                // Skip point if color cannot be extracted (don't add gray points)
                continue;
            }

            tmp->points.push_back(p);
        }
    }

    Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat(kf->GetPose());
    PointCloud::Ptr cloud(new PointCloud);
    //pass.setInputCloud(tmp);
    //pass.filter(*cloud);
    //tmp->swap(*cloud);

    pcl::transformPointCloud(*tmp, *cloud, T.inverse().matrix());
    cloud->is_dense = false;

    std::cout << "generate point cloud for kf " << kf->mnId << ", size=" << cloud->points.size() << std::endl;
    return cloud;
}

PointCloudMapping::PointCloud::Ptr 
PointCloudMapping::filterSmallClustersByVolume(PointCloud::Ptr cloud, double minVolume, 
                                                double clusterTolerance, int minClusterSize)
{
    if (cloud->points.empty() || cloud->points.size() < 50)
    {
        return cloud;
    }

    PointCloud::Ptr filtered_cloud(new PointCloud());
    
    // Create KdTree for nearest neighbor search
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    // Extract clusters using Euclidean clustering
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);  // Configurable cluster tolerance
    ec.setMinClusterSize(minClusterSize);       // Configurable minimum cluster size
    ec.setMaxClusterSize(25000000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    std::cout << "Found " << cluster_indices.size() << " clusters before volume filtering" << std::endl;

    int removed_clusters = 0;
    int kept_clusters = 0;
    
    // Calculate volume for each cluster and filter by volume
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
         it != cluster_indices.end(); ++it)
    {
        if (it->indices.empty())
            continue;

        // Calculate axis-aligned bounding box (AABB) for the cluster
        double min_x = std::numeric_limits<double>::max();
        double max_x = std::numeric_limits<double>::lowest();
        double min_y = std::numeric_limits<double>::max();
        double max_y = std::numeric_limits<double>::lowest();
        double min_z = std::numeric_limits<double>::max();
        double max_z = std::numeric_limits<double>::lowest();

        for (std::vector<int>::const_iterator pit = it->indices.begin();
             pit != it->indices.end(); ++pit)
        {
            const PointT& pt = cloud->points[*pit];
            min_x = std::min(min_x, static_cast<double>(pt.x));
            max_x = std::max(max_x, static_cast<double>(pt.x));
            min_y = std::min(min_y, static_cast<double>(pt.y));
            max_y = std::max(max_y, static_cast<double>(pt.y));
            min_z = std::min(min_z, static_cast<double>(pt.z));
            max_z = std::max(max_z, static_cast<double>(pt.z));
        }

        // Calculate volume of the bounding box
        double width = max_x - min_x;
        double height = max_y - min_y;
        double depth = max_z - min_z;
        double volume = width * height * depth;

        // Keep cluster if volume is above threshold
        if (volume >= minVolume)
        {
            for (std::vector<int>::const_iterator pit = it->indices.begin();
                 pit != it->indices.end(); ++pit)
            {
                filtered_cloud->points.push_back(cloud->points[*pit]);
            }
            kept_clusters++;
        }
        else
        {
            removed_clusters++;
            std::cout << "Removed cluster with volume " << volume << " m³ (size: " 
                      << width << " x " << height << " x " << depth 
                      << " m, points: " << it->indices.size() << ")" << std::endl;
        }
    }

    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = false;

    std::cout << "Volume-based filtering: kept " << kept_clusters << " clusters, removed " 
              << removed_clusters << " small clusters" << std::endl;
    std::cout << "Points before: " << cloud->points.size() 
              << ", after: " << filtered_cloud->points.size() << std::endl;

    return filtered_cloud;
}

void PointCloudMapping::viewer()
{
    pcl::visualization::CloudViewer viewer("PointCloud Viewer");

    while (true)
    {
        {
            std::unique_lock<std::mutex> lck_shutdown(shutDownMutex);
            if (shutDownFlag)
            {
                break;
            }
        }
        {
            std::unique_lock<std::mutex> lck_keyframeUpdated(keyFrameUpdateMutex);
            keyFrameUpdated.wait(lck_keyframeUpdated);
        }


        // keyframe is updated
//        keyframes = mpMap->GetAllKeyFrames();
//        size_t N = 0;
//        {
//            unique_lock<mutex> lck(keyframeMutex);
//            N = keyframes.size();
//        }
//
//        for (size_t i = lastKeyframeSize; i < N; i++)
//        {
//            PointCloud::Ptr p = generatePointCloud(keyframes[i], keyframes[i]->img0, keyframes[i]->img1);
//            *globalMap += *p;
//        }
//
//        PointCloud::Ptr tmp(new PointCloud());
//        voxel.setInputCloud(globalMap);
//        voxel.filter(*tmp);
//
//        globalMap->swap(*tmp);

        generateMap();
        viewer.showCloud(globalMap);

        std::cout << "show global map, size=" << globalMap->points.size() << std::endl;
//        lastKeyframeSize = N;
    }
    savePointCloud("result.pcd");
    std::cout << "pcl_viewer shutdown!" << std::endl;
}

void PointCloudMapping::savePointCloud(const std::string &name)
{
//    generateMap();
    if (!globalMap->empty())
    {
        pcl::io::savePCDFile(name, *globalMap, true);
        std::cout << "save map to file : " << name << std::endl;
    }
}

void PointCloudMapping::generateMap()
{
    keyframes = mpMap->GetAllKeyFrames();
    globalMap->clear();
    size_t N = 0;
    {
        std::unique_lock<std::mutex> lck(keyframeMutex);
        N = keyframes.size();
    }

    for (size_t i = 0; i < N; i++)
    {
        PointCloud::Ptr p = generatePointCloud(keyframes[i], keyframes[i]->img0, keyframes[i]->img1, keyframes[i]->mask);
        *globalMap += *p;
    }

    PointCloud::Ptr tmp(new PointCloud());
    voxel.setInputCloud(globalMap);
    voxel.filter(*tmp);

    globalMap->swap(*tmp);

    // Apply local volume-based clustering filter to remove small clusters (residual human features)
    // Local filtering: smaller threshold (0.05 m³) for fine-grained removal
    if (globalMap->points.size() > 100)
    {
        PointCloud::Ptr filtered_cloud = filterSmallClustersByVolume(globalMap, 0.05, 0.05, 20); // Local: 0.05m³, 5cm tolerance, 20 points min
        globalMap->swap(*filtered_cloud);
        std::cout << "After local volume-based filtering: " << globalMap->points.size() << " points remaining" << std::endl;
    }

    // Apply global volume-based clustering filter with larger threshold for final cleanup
    // Global filtering: larger threshold (0.2 m³) and tolerance for removing larger residual objects
    if (globalMap->points.size() > 100)
    {
        PointCloud::Ptr global_filtered_cloud = filterSmallClustersByVolume(globalMap, 0.2, 0.1, 50); // Global: 0.2m³, 10cm tolerance, 50 points min
        globalMap->swap(*global_filtered_cloud);
        std::cout << "After global volume-based filtering: " << globalMap->points.size() << " points remaining" << std::endl;
    }

    std::cout << "show global map, size=" << globalMap->points.size() << std::endl;

}
