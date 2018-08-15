/* ******** test every module ********** */

#include "accessFile.h"
#include "sampleCloud.h"
#include "extractFeature.h"
#include "getFeatureVector.h"

#define FILE_PATH "/home/shao/文档/VSCodeWS/Masterarbeit_Code/PointCloud_FeatureEngineering/dataset_example/car742.pcd"

/*
* influence of random downsample on eigen values of covariance matrix *
*/
void
influence_downsample_rand_on_covmat_eig (
    const unsigned int & init_size_after_downsampling,
    const unsigned int & num_downsample_size,
    const unsigned int & runningtime_for_average
    )
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZI>);
    // before downsampling
    readPCD(FILE_PATH, cloud);
    std::vector<float> eigvals;
    geoCovEigen(cloud, eigvals, true);
    std::cout << "eigen values of global covariance matrix before downsampling: " << std::endl;
    std::cout << eigvals[0] << ">=" << eigvals[1] << ">=" << eigvals[2] << std::endl;
    // downSample_rand
    unsigned int size_after_downsampling = init_size_after_downsampling;
    for (unsigned int n = 0; n != num_downsample_size; ++n)
    {
        if (n)
            size_after_downsampling /= 2;
        std::vector<float> mean_eigvals {0.0, 0.0, 0.0};
        for (unsigned int i = 0; i != runningtime_for_average; ++i)
        {
            downSample_rand(cloud, cloud_downsampled, size_after_downsampling, false);
            geoCovEigen(cloud_downsampled, eigvals, true);
            for (size_t j = 0; j != eigvals.size(); ++j)
                mean_eigvals[j] += eigvals[j];
        }
        for (auto & e : mean_eigvals)
            e /= runningtime_for_average;

        std::cout << "mean eigen values of global covariance matrix after downsampling with cloud size: "
                    << cloud_downsampled->points.size()
                    << " over " 
                    << runningtime_for_average
                    << " running: " 
                    << std::endl;
        std::cout << mean_eigvals[0] << ">=" << mean_eigvals[1] << ">=" << mean_eigvals[2] << std::endl;
    } 
}


/*
* influence of voxel grid downsample on eigen values of covariance matrix *
*/
void
influence_downsample_vg_on_covmat_eig (
    const float & init_cube_leaf_side_length,     
    const unsigned int & runningtime
)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZI>);
    float leaf_size = init_cube_leaf_side_length;
    readPCD(FILE_PATH, cloud);
    std::vector<float> eigvals;
    geoCovEigen(cloud, eigvals, true);
    std::cout << "eigen values of global covariance matrix before downsampling: " << std::endl;
    std::cout << eigvals[0] << ">=" << eigvals[1] << ">=" << eigvals[2] << std::endl;
    for (unsigned int i = 1; i != runningtime+1; ++i)
    {
        if (i > 1)
            leaf_size *= 2; 
        downSample_vg(cloud, cloud_downsampled, leaf_size, true);
        geoCovEigen(cloud_downsampled, eigvals, true);
        std::cout << "eigen values after downsampling with leaf size "
                    << leaf_size << "m "
                    << "are: "
                    << "\n"
                    << eigvals[0] << ">=" << eigvals[1] << ">=" << eigvals[2] 
                    << std::endl;
    }
}


/*
* influence of upsample on eigen values of covariance matrix *
*/
void
influence_upsample_on_covmat_eig (
    const unsigned int & runningtime
    )
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_upsampled (new pcl::PointCloud<pcl::PointXYZI>);
    readPCD(FILE_PATH, cloud);
    std::vector<float> eigvals;
    geoCovEigen(cloud, eigvals, true);
    std::cout << "eigen values of global covariance matrix before upsampling: " << std::endl;
    for (auto & e : eigvals)
        std::cout << e << " ";
    std::cout << std::endl;
    // upsample
    for (unsigned int i = 0; i != runningtime; ++i)
    {
        upSample(cloud, cloud_upsampled);
        geoCovEigen(cloud_upsampled, eigvals, true);
        std::cout << "eigen values of global covariance matrix after upsampling with cloud size: " 
                    << cloud_upsampled->points.size()
                    << std::endl;
        for (auto & e : eigvals)
            std::cout << e << " ";
        std::cout << std::endl;
        // cloud->points.clear();
        cloud->points = cloud_upsampled->points;
        // cloud_upsampled->points.clear();
    }
}


/*
* influence of random downsample on moment invariants *
*/
void
influence_downsample_rand_on_moment_invariants (
    const unsigned int & init_size_after_downsampling,
    const unsigned int & num_downsample_size,
    const unsigned int & runningtime_for_average
)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZI>);
    // before downsampling
    readPCD(FILE_PATH, cloud);
    std::vector<float> jvec;
    momentInvariants(cloud, jvec);
    std::cout << "moment invariants before downsampling: " << std::endl;
    for (auto & mi : jvec)
        std::cout << mi/cloud->points.size() << " ";       // divided by cloud->points.size()
    std::cout << std::endl;
    // downSample_rand
    unsigned int size_after_downsampling = init_size_after_downsampling;
    for (unsigned int n = 0; n != num_downsample_size; ++n)
    {
        if (n)
            size_after_downsampling /= 2;
        std::vector<float> mean_jvec {0.0, 0.0, 0.0};
        for (unsigned int i = 0; i != runningtime_for_average; ++i)
        {
            downSample_rand(cloud, cloud_downsampled, size_after_downsampling, false);
            momentInvariants(cloud_downsampled, jvec);
            for (size_t j = 0; j != jvec.size(); ++j)
                mean_jvec[j] += jvec[j];
        }
        std::cout << "mean moment invariants after downsampling with cloud size: "
            << cloud_downsampled->points.size()
            << " over " 
            << runningtime_for_average
            << " running: " 
            << std::endl;
        for (auto & e : mean_jvec)
        {
            e /= runningtime_for_average;
            e /= cloud_downsampled->points.size();       // divided by cloud->points.size()
            std::cout << e << " ";
        }
        std::cout << std::endl;
    }
}


/*
* influence of voxel grid downsample on moment invariants *
*/
void
influence_downsample_vg_on_moment_invariants (
    const float & init_cube_leaf_side_length,     
    const unsigned int & runningtime
)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZI>);
    float leaf_size = init_cube_leaf_side_length;
    readPCD(FILE_PATH, cloud);
    std::vector<float> jvec;
    momentInvariants(cloud, jvec);
    std::cout << "moment invariants before downsampling: " << std::endl;
    for (auto & mi : jvec)
        std::cout << mi/cloud->points.size() << " ";       // divided by cloud->points.size()
    std::cout << std::endl;
    for (unsigned int i = 0; i != runningtime; ++i)
    {
        if (i)
            leaf_size *= 2; 
        downSample_vg(cloud, cloud_downsampled, leaf_size, true);
        momentInvariants(cloud_downsampled, jvec);
        std::cout << "moment invariants after downsampling with leaf size "
                    << leaf_size 
                    << "m are: "
                    << std::endl;
        for (auto & e : jvec)
        {
            e /= cloud_downsampled->points.size();       // divided by cloud->points.size()
            std::cout << e << " ";
        }
        std::cout << std::endl;
    }
}


/*
* influence of upsample on moment invariants *
*/
void
influence_upsample_on_moment_invariants (
    const unsigned int & runningtime
    )
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_upsampled (new pcl::PointCloud<pcl::PointXYZI>);
    readPCD(FILE_PATH, cloud);
    std::vector<float> jvec;
    momentInvariants(cloud, jvec);
    std::cout << "moment invariants before upsampling: " << std::endl;
    for (auto & e : jvec)
        std::cout << e/cloud->points.size() << " ";    // divided by cloud->points.size()
    std::cout << std::endl;
    // upsample
    for (unsigned int i = 0; i != runningtime; ++i)
    {
        upSample(cloud, cloud_upsampled);
        momentInvariants(cloud_upsampled, jvec);
        std::cout << "moment invariants after upsampling with cloud size: " 
                    << cloud_upsampled->points.size()
                    << std::endl;
        for (auto & e : jvec)
        {
            e /= cloud_upsampled->points.size();     // divided by cloud->points.size()
            std::cout << e << " ";              
        }
        std::cout << std::endl;
        cloud->points = cloud_upsampled->points;
    }
}



int 
main(void)
{
    // /**** test the influence of random downsample on eigen values of covaraince matrix ****/
    // influence_downsample_rand_on_covmat_eig(4000, 4, 1000);


    // /**** test the influence of voxel grid downsample on eigen values of covaraince matrix ****/
    // influence_downsample_vg_on_covmat_eig(0.01, 5);


    // /**** test the influence of upsample on eigen values of covaraince matrix ****/
    // influence_upsample_on_covmat_eig(4);


    // /**** test the influence of random downsample on moment invariants ****/
    // // influence_downsample_rand_on_moment_invariants(4000, 4, 1000);


    // /**** test the influence of voxel grid downsample on moment invariants ****/
    // influence_downsample_vg_on_moment_invariants(0.01, 5);


    // /**** test the influence of upsample on moment invariants ****/
    // influence_upsample_on_moment_invariants(4);


    // /**** build training feature vector with different search radius ****/
    // const std::vector<float> searchRadius { 0.08, 0.10, 0.12, 0.14, 0.16, 0.18, 0.20, 0.22};
    // for (auto & r : searchRadius)
    // {
    //     getFeatureVector(r, true);
    // }


    /**** build test feature vector with search radius = 0.16m ****/
    getFeatureVector(0.16, false);

    return 0;
}
