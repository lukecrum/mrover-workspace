#pragma once
#include "perception.hpp"

#if OBSTACLE_DETECTION
	#include <pcl/common/common_headers.h>
#endif

class PCL {
    public:
        PCL();
        ~PCL();

    #if OBSTACLE_DETECTION
        private:
            void PassThroughFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pt_cloud_ptr);
            void DownsampleVoxelFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pt_cloud_ptr);
            void RANSACSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pt_cloud_ptr, string type);
            void CPUEuclidianClusterExtraction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pt_cloud_ptr, 
                                            std::vector<pcl::PointIndices> &cluster_indices);
            void FindInterestPoints(std::vector<pcl::PointIndices> &cluster_indices, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pt_cloud_ptr, 
                                    std::vector<std::vector<int>> &interest_points);
            bool FindClearPath(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pt_cloud_ptr, 
                                std::vector<std::vector<int>> interest_points,
                                shared_ptr<pcl::visualization::PCLVisualizer> viewer, 
                                obstacle_return & result);
        public:
            obstacle_return pcl_obstacle_detection(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pt_cloud_ptr, 
                                                shared_ptr<pcl::visualization::PCLVisualizer> viewer);
            shared_ptr<pcl::visualization::PCLVisualizer> createRGBVisualizer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
    #endif

};