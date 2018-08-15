#ifndef FEATUREEXTRACTION_H
#define FEATUREEXTRACTION_H

#include "commonHeadFiles.h"

#include "searchKdtree.h"
#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/impl/filter.hpp>
#include <pcl/common/eigen.h>
#include <pcl/features/moment_invariants.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/intensity_gradient.h>

/*
****************************************
********** Normals estimation **********
****************************************
*/

void
estimateNormals(
                    const pcl::PointCloud<pcl::PointXYZI>::Ptr & inputCloud,     // input cloud
                    const pcl::PointCloud<pcl::PointXYZI>::Ptr & searchSurface, // search surface
                    const float & searchRadius,                               // radius for searching [m]
                    pcl::PointCloud<pcl::Normal>::Ptr & normals,    // output normals for every point
                    bool debug = false                              // debug mode, default false
                    // bool removeNaN = true,
                    // std::vector<int> & mapping,
                    // bool visualize = false
                );



/*
****************************************
********** Remove NaN Normals **********
****************************************
*/

void
removeNanNormals(
                const pcl::PointCloud<pcl::Normal>::Ptr & inputNormals, // input normals inklusive NaN
                pcl::PointCloud<pcl::Normal>::Ptr & outputNormals,      // output normals without NaN
                std::vector<int> & mapping                            // output index in inputNormals with
                                                                      // inputNormals[index[i]] != NaN
                );



/*
****************************************************
********** Remove Points with NaN Normals **********
****************************************************
*/

void 
removePointWithNanNormal(
                            const pcl::PointCloud<pcl::PointXYZI>::Ptr & inputCloud,  // input cloud
                            pcl::PointCloud<pcl::PointXYZI>::Ptr & outputCloud,   // output cloud, which has no NaN normal
                            const std::vector<int> & mapping,              // index in inputCloud, inputCloud[index[i]] has no NaN normal
                            bool debug = false                    // debug mode, default false
                        );



/*
***************************************
********** Visualize normals **********
***************************************
*/

void
visualizeNormals(
                const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,      // cloud with XYZI
                const pcl::PointCloud<pcl::Normal>::Ptr & normals        // normals
                );     




/*
***************************
****** Estimate FPFH ******
***************************
*/

void
estimateFPFH(
                const pcl::PointCloud<pcl::PointXYZI>::Ptr & inputCloud,  // input cloud
                const pcl::PointCloud<pcl::Normal>::Ptr & inputNormals,   // input normals
                const float & searchRadius,                               // param search radius
                std::vector<float> & FPFHHisto,                           // output FPFH histogram
                bool debug = false                                        // debug mode, defualt false
            );




/*
*****************************************************
******* Transform point cloud to Eigen Matrix *******
*****************************************************
*/

void
cloud2Matrix(
                const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,   // input cloud
                Eigen::MatrixXf & PointMat,                         // output point matrix
                const std::string & mode                            // transform mode: "xy" or "xyz"
            );


/*
*************************************************
********** Calculate covariance matrix **********
*************************************************
*/

void
covMatrix(
            const Eigen::MatrixXf & M,
            Eigen::MatrixXf & CovMat
        );



/*
*****************************************
****** Calculate 3D Size using PCA ******
*****************************************
*/

void 
geometrySize(
                const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,  // input cloud
                float & length,                                      // output length
                float & width,                                       // output width
                float & height                                       // output height
            );      



/*
****************************************************************************
****** Calculate eigen values of covaraince matrix in geometric domain******
****************************************************************************
*/

void
geoCovEigen(
                const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,     // input cloud
                std::vector<float> & eigvals,        // output eigen values
                bool sort = true                     // sort mode, if true (default), sort eigvals from large to small
                                                     // 1 >= eigvals[0] >= eigvals[1] >= eigvals[2] >= 0
            );



/*
****************************************
****** Calculate Lalonde features ******
****************************************
*/

void
lalondeFeat(
                const pcl::PointCloud<pcl::PointXYZI>::Ptr & inputCloud,     // input cloud
                const pcl::PointCloud<pcl::PointXYZI>::Ptr & searchSurface,  // input search surface
                const float & searchRadius,                                // param search radius
                std::vector<float> & lalondeHisto                          // output lalonde histogram
            );         



/*
************************************************
****** Calculate Features about intensity ******
************************************************
*/

void
intensity(
            const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,   // input cloud
            float & Imax,                                         // output max intensity
            float & Imean,                                        // output mean intensity
            float & Ivar                                          // output intensity variance
            );



/*
****************************************
****** Estimate moment invariants ******
****************************************
*/

void
momentInvariants(
                    const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud, // input cloud
                    std::vector<float> & jvec     // output moment invariants vector jvec[0] jvec[1] jvec[2]
                    );



/*
***********************************************
****** Calculate mean intensity gradient ******
***********************************************
*/

void
meanIntensityGradient(
                        const pcl::PointCloud<pcl::PointXYZI>::Ptr & inputCloud,  // input cloud
                        const pcl::PointCloud<pcl::Normal>::Ptr & inputNormals,   // input normals
                        const float & searchRadius,                      // param search radius
                        float & meanIG,                                  // output meanIG
                        bool debug = false                               // debug mode, default false
                        );

#endif
