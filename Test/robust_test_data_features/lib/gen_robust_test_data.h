#ifndef GEN_ROBUST_TEST_DATA_H
#define GEN_ROBUST_TEST_DATA_H

#include "commonHeadFiles.h"
#include "sampleCloud.h"


// occlusion with different percentages
void 
getOccludedCloud (
    const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
    const std::string & label_name, 
    const float & occlusion_percentage,
    pcl::PointCloud<pcl::PointXYZI>::Ptr & occluded_cloud
    );



// random downsample with different percentage
void
getSparseCloud (
    const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,      // input cloud
    const float & sparse_percentage,                         // percentage of points after downsampling
    pcl::PointCloud<pcl::PointXYZI>::Ptr & sparse_cloud      // output sparse cloud
    );



// add gaussian noise to cloud using Box-Muller algorithm
void
addNoise (
    const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,      // input cloud
    const float & sigma,                                     // param standard variance
    pcl::PointCloud<pcl::PointXYZI>::Ptr & noise_cloud       // output cloud with noise  
    );

#endif