#include "getFeatureVector.h"

/*
***********************************************
********** build feature vector ***************
***********************************************
*/

void
getFeatureVector (
                    const float & searchRadius,                     // param search radius
                    pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,   // input cloud
                    const int & i,                                  // param label name index
                    std::vector<float> & featVec                    // output feature vector
    )
{
    if (!featVec.empty())
        featVec.clear();
    /*** global features have nothing to do with normals and search radius  ***/

    // geometry size: length, width, height
    std::vector<float> geoSize {0.0, 0.0, 0.0};
    geometrySize(cloud, geoSize[0], geoSize[1], geoSize[2]);
    for (auto & gs : geoSize)
        featVec.push_back(gs);
    // intensity: Imax, Imean, Ivar
    std::vector<float> intenFeat {0.0, 0.0, 0.0};
    intensity(cloud, intenFeat[0], intenFeat[1], intenFeat[2]);
    for (auto & i : intenFeat)
        featVec.push_back(i);
    // moment invariants (results should be divided by point number in cloud)
    std::vector<float> jvec;
    momentInvariants(cloud, jvec);
    featVec.push_back(jvec[0]/cloud->points.size());
    // global eigen values of covariance matrix
    std::vector<float> eigvals;
    geoCovEigen(cloud, eigvals, true);
    for (auto & e : eigvals)
        featVec.push_back(e);
    

    /****** local features, which depend on search radius ******/

    // upsample
    while (cloud->points.size() < size_low_limit[i])
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr 
        cloud_dense (new pcl::PointCloud<pcl::PointXYZI>);
        upSample(cloud, cloud_dense);
        cloud->points = cloud_dense->points;
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    // downsample
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_sparse (new pcl::PointCloud<pcl::PointXYZI>);
    downSample_rand(cloud, cloud_sparse, size_high_limit[i], false);

    // calculate lalonde feature histogram
    std::vector<float> lalondeHisto;
    lalondeFeat(cloud_sparse, cloud, searchRadius, lalondeHisto);
    if (!lalondeHisto.empty())
        for (auto & lh : lalondeHisto)
            featVec.push_back(lh);

    // estimate normals
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    estimateNormals(cloud_sparse, cloud, searchRadius, normals);
    
    // remove NaN normals
    pcl::PointCloud<pcl::Normal>::Ptr normals_valid (new pcl::PointCloud<pcl::Normal>);
    std::vector<int> mapping;
    removeNanNormals(normals, normals_valid, mapping);
    if (mapping.empty())
        return;
    
    // remove point with NaN normal
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_valid (new pcl::PointCloud<pcl::PointXYZI>);                
    removePointWithNanNormal(cloud_sparse, cloud_valid, mapping);

    // estimate mean intensity gradient
    float meanIG = 0.0;
    meanIntensityGradient(cloud_valid, normals_valid, searchRadius, meanIG);
    featVec.push_back(meanIG);

    // estimate FPFH histogram
    std::vector<float> FPFHHisto;
    estimateFPFH(cloud_valid, normals_valid, searchRadius, FPFHHisto);
    if (!FPFHHisto.empty())
        for (auto & fh : FPFHHisto)
            featVec.push_back(fh);
}