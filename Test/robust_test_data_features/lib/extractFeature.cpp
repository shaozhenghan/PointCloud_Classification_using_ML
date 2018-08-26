#include "extractFeature.h"


/*
****************************************
********** Normals estimation **********
****************************************
*/

void
estimateNormals(
                    const pcl::PointCloud<pcl::PointXYZI>::Ptr & inputCloud,    // input cloud
                    const pcl::PointCloud<pcl::PointXYZI>::Ptr & searchSurface, // search surface
                    const float & searchRadius,                               // radius for searching [m]
                    pcl::PointCloud<pcl::Normal>::Ptr & normals,         // output normals for every point
                    bool debug                                           // debug mode, default false
                    // bool removeNaN,
                    // std::vector<int> & mapping,
                    // bool visualize
                )
{
    pcl::NormalEstimationOMP<pcl::PointXYZI, pcl::Normal> ne;
    ne.setInputCloud (inputCloud);
    ne.setSearchSurface (searchSurface);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (searchRadius);
    ne.compute (*normals);

    // if (removeNaN)
    // {

    // }
    // debug mode
    if (debug)
    {
        std::vector<int> mapping;
        pcl::PointCloud<pcl::Normal>::Ptr noNanNormals (new pcl::PointCloud<pcl::Normal>);
        removeNanNormals (normals, noNanNormals, mapping);
        std::cout << "normalEstimation(): debug: " << std::endl;
        std::cout << "total number of normals: " 
                    << normals->points.size() 
                    << std::endl;
        std::cout << "number of normals without NaN: " 
                    << noNanNormals->points.size()
                    << std::endl;
    }

    // // optional: visualization for debug
    // if (visualize)
    // {
    //     pcl::visualization::PCLVisualizer viewer ("PCL Viewer");
    //     viewer.setBackgroundColor (0.0, 0.0, 0.0);
    //     viewer.addPointCloudNormals<pcl::PointXYZI,pcl::Normal>(inputCloud, normals);
    //     while (!viewer.wasStopped ())
    //     {
    //         viewer.spinOnce ();
    //     }
    // }
}


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
                )
{
    if (!mapping.empty())
        mapping.clear();
    pcl::removeNaNNormalsFromPointCloud(*inputNormals, *outputNormals, mapping);
}


/*
****************************************************
********** Remove Points with NaN Normals **********
****************************************************
*/

void 
removePointWithNanNormal(
                            const pcl::PointCloud<pcl::PointXYZI>::Ptr & inputCloud,  // input cloud
                            pcl::PointCloud<pcl::PointXYZI>::Ptr & outputCloud,    // output cloud, which has no NaN normal
                            const std::vector<int> & mapping,              // index in inputCloud, inputCloud[index[i]] has no NaN normal
                            bool debug                            // debug mode, default false
                        )
{
    if (mapping.empty())
    {
        std::cout << "removePointWithNanNormal(): The input vector is empty!" << std::endl;
        return; 
    }
    // clear the possible invalid residual values in outputCloud
    if (!outputCloud->points.empty())
        outputCloud->points.clear();
    
    for (auto & idx : mapping)
    {
        outputCloud->points.push_back(inputCloud->points[idx]);
    }
    outputCloud->width = outputCloud->points.size();
    outputCloud->height = 1;
    outputCloud->is_dense = true;
    // debug mode
    if (debug)
    {
        std::cout << "removePointWithNanNormal(): debug: " << std::endl;
        std::cout << "total number of points in input cloud: " 
            << inputCloud->points.size() 
            << std::endl;
        std::cout << "number of points in output cloud, which has no points with Nan normals: " 
            << outputCloud->points.size() 
            << std::endl;         
    }
}


/*
***************************************
********** Visualize normals **********
***************************************
*/

void
visualizeNormals(
                const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,      // cloud with XYZI
                const pcl::PointCloud<pcl::Normal>::Ptr & normals        // normals
                )
{
    if (cloud->points.size() == normals->points.size() && !normals->empty())
    {
        pcl::visualization::PCLVisualizer viewer ("Pointcloud with normals Viewer");
        viewer.setBackgroundColor (0.0, 0.0, 0.0);
        viewer.addPointCloudNormals<pcl::PointXYZI,pcl::Normal>(cloud, normals);
        while (!viewer.wasStopped ())
        {
            viewer.spinOnce ();
        }
    }
    else
    {
        std::cerr << "visualizeNormals(): Wrong size of inputs!" << std::endl;
        return;
    }
}



/*
***************************
****** Estimate FPFH ******
***************************
*/

void
estimateFPFH(
                const pcl::PointCloud<pcl::PointXYZI>::Ptr & inputCloud,   // input cloud
                const pcl::PointCloud<pcl::Normal>::Ptr & inputNormals,    // input normals
                const float & searchRadius,                                // param search radius
                std::vector<float> & FPFHHisto,                            // output FPFH histogram
                bool debug                                                 // debug mode, default false
            )
{
    // clear the possible invalid residual values in FPFHHisto
    if (!FPFHHisto.empty())
        FPFHHisto.clear();
    
    pcl::FPFHEstimationOMP<pcl::PointXYZI, pcl::Normal, pcl::FPFHSignature33> fpfhe;
    // set input
    fpfhe.setInputCloud(inputCloud);
    fpfhe.setInputNormals(inputNormals);
    // set search method
    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZI>);
    fpfhe.setSearchMethod(kdtree);
    // Radius search, r = searchRadius
    fpfhe.setRadiusSearch(searchRadius);
    // compute fpfh_
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_ (new pcl::PointCloud<pcl::FPFHSignature33>);
    fpfhe.compute(*fpfh_);
    
    // remove invalid values in fpfh_
    // if fpfh_->points[i].histogram[for all index] == 0 => point[i]is invalid 
    // reason: the search radius too small
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh (new pcl::PointCloud<pcl::FPFHSignature33>);
    for (auto & p: fpfh_->points)
    {
        bool isValid = false;
        for (auto & h : p.histogram)
            if (h != 0.0)
            {
                isValid = true;
                break;
            }
        if (isValid)
            fpfh->points.push_back(p);
    }
    // debug mode
    if (debug)
    {
        std::cout << "estimateFPFH(): debug: " << std::endl;
        std::cout << "total number of elements in calculated FPFH: " 
                    << fpfh_->points.size() 
                    << std::endl;
        std::cout << "number of valid elements in calculated FPFH: " 
                    << fpfh->points.size() 
                    << std::endl;
    }
    // check, wether the size of fpfh is 0
    size_t size = fpfh->points.size();
    if (size == 0)
    {
        std::cerr << "estimateFPFH(): the valid size of fpfh is 0! return without doing anything! \n" << std::endl;
        return;
    }
    // build FPFH histogram
    float mean = 0.0;
    for (unsigned int i = 0; i != 33; ++i)
    {
        mean = 0.0;
        for (auto & r : fpfh->points)
            mean += r.histogram[i];
        mean /= size;
        FPFHHisto.push_back(mean);
    }
    assert(FPFHHisto.size() == 33);
}

/*
*****************************************************
******* Transform point cloud to Eigen Matrix *******
*****************************************************
*/

void
cloud2Matrix(
                const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,   // input cloud
                Eigen::MatrixXf & PointMat,                    // output point matrix
                const std::string & mode                       // transform mode: "xy" or "xyz" or "i"
            )
{
    if (mode == "xy")
    {
        PointMat.resize(cloud->points.size(), 2);
        for (size_t i = 0; i != cloud->points.size(); ++i)
        {
            PointMat(i,0) = cloud->points[i].x;
            PointMat(i,1) = cloud->points[i].y;  
        }
    }
    else if (mode == "xyz")
    {
        PointMat.resize(cloud->points.size(), 3);
        for (size_t i = 0; i != cloud->points.size(); ++i)
        {
            PointMat(i,0) = cloud->points[i].x;
            PointMat(i,1) = cloud->points[i].y;  
            PointMat(i,2) = cloud->points[i].z;
        }   
    }
    else if (mode == "i")
    {
        PointMat.resize(cloud->points.size(), 1);
        for (size_t i = 0; i != cloud->points.size(); ++i)
        {
            PointMat(i, 0) = cloud->points[i].intensity;
        }
    }
    else
    {
        std::cerr << "cloud2Matrix(): invalid mode!" << std::endl;
        return;
    }
}



/*
*************************************************
********** Calculate covariance matrix **********
*************************************************
*/

void
covMatrix(
            const Eigen::MatrixXf & M,
            Eigen::MatrixXf & CovMat
        )
{
    unsigned int col = M.cols();
    unsigned int row = M.rows();
    Eigen::MatrixXf meanVector(1, col);
    Eigen::MatrixXf M_centered(row, col);

    meanVector = M.colwise().mean();

    for (unsigned int i = 0; i != row; ++i)
    {
        M_centered.row(i) = M.row(i) - meanVector;
    }

    CovMat.resize(col, col);
    CovMat = 1.0 / row * (M_centered.transpose() * M_centered); 
}

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
            ) 
{
    /***** PCA starts *****/
    // transform cloud to matrix in XOY
    Eigen::MatrixXf PointXYZMat;
    cloud2Matrix(cloud, PointXYZMat, "xyz");
    Eigen::MatrixXf PointXYMat = PointXYZMat.leftCols(2);
    // std::cout << PointXYMat.cols() << " " << PointXYMat.rows() << std::endl; // debug

    // calculate height
    height = PointXYZMat.col(2).maxCoeff();
    
    // calculate covariance matrix
    Eigen::MatrixXf CovMat;
    covMatrix(PointXYMat, CovMat);

    // svd-decomposition for convariance matrix
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(CovMat, Eigen::ComputeFullU);
    Eigen::MatrixXf U = svd.matrixU();
    
    // projection
    Eigen::MatrixXf PointXYMat_pca = PointXYMat * U;
    /***** PCA ends *****/

    // calculate length and width
    Eigen::Vector2f maxValues = PointXYMat_pca.colwise().maxCoeff();
    Eigen::Vector2f minValues = PointXYMat_pca.colwise().minCoeff();
    Eigen::Vector2f length_width = maxValues - minValues;
    length = std::max(length_width(0), length_width(1));
    width = std::min(length_width(0), length_width(1));
}



/*
****************************************************************************
****** Calculate eigen values of covaraince matrix in geometric domain******
****************************************************************************
*/

void
geoCovEigen(
                const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,     // input cloud
                std::vector<float> & eigvals,        // output eigen values
                bool sort                            // sort mode, if true (default), sort eigvals from large to small
                                                     // 1 >= eigvals[0] >= eigvals[1] >= eigvals[2] >= 0
            )
{
    Eigen::MatrixXf PointMat;
    cloud2Matrix(cloud, PointMat, "xyz");
    Eigen::MatrixXf CovMat;
    covMatrix(PointMat, CovMat);
    Eigen::EigenSolver<Eigen::Matrix3f> ev(CovMat);
    // eigen values are not sorted!!
    Eigen::Matrix3f EigValMat = ev.pseudoEigenvalueMatrix();

    // //debug
    // std::cout << "EigValMat: " << "\n" << EigValMat << std::endl;

    // ensure that e1, e2, e3 belong to [0, 1]
    float sumEigVal = EigValMat.sum();
    if (!eigvals.empty())
        eigvals.clear();
    for (unsigned int i = 0; i != EigValMat.rows(); ++i)
    {
        eigvals.push_back(EigValMat(i, i) / sumEigVal);
    }
    // sort the eigen values in vector with bubble sort: from large to small
    if (sort)
        for (size_t i = 0; i != eigvals.size(); ++i)
            for (auto e = eigvals.begin(); e != eigvals.end(); ++e)
                if (*e < *(e+1))
                {
                    float tmp = *e;
                    *e = *(e+1);
                    *(e+1) = tmp;
                }
}


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
            )
{
    lalondeHisto = {0.0, 0.0, 0.0};
    unsigned int valid_size = 0;
    for (auto & point : inputCloud->points)
    {
        // mode: RadiusSearch, r = searchRadius
        std::vector<int> searchedPointIdx;
        kdtreeSearch(searchSurface, point, searchedPointIdx, "r", 0, searchRadius);

        // if too few points searched, don't calculate histogram.
        if (searchedPointIdx.size() < 5)
            continue;

        ++valid_size;
        // cloud in neighborhood of the point
        pcl::PointCloud<pcl::PointXYZI>::Ptr neighborCloud (new pcl::PointCloud<pcl::PointXYZI>);
        for (auto & idx : searchedPointIdx)
        {
            neighborCloud->points.push_back(searchSurface->points[idx]);
        }
        neighborCloud->width = neighborCloud->size();
        neighborCloud->height = 1;
        neighborCloud->is_dense = true;
        // // debug
        // std::cout << neighborCloud->points.size() << std::endl;

        // calculate eigen values of the covariance matrix
        std::vector<float> eigvals;        
        geoCovEigen(neighborCloud, eigvals, true); // 1 >= eigvals[0] >= eigvals[1] >= eigvals[2] >= 0
        
        // // debug
        // for(auto &r : eigvals)
        //     std::cout << "eigvals: " << r << std::endl;

        // lalonde features
        std::vector<float> lalondeFeats = {eigvals[0], eigvals[0] - eigvals[1], eigvals[1] - eigvals[2]};
        
        // accumulate histogram   
        for (unsigned int i = 0; i != 3; ++i)
        {
            lalondeHisto[i] += lalondeFeats[i];
            // // debug
            // std::cout << "lalondeFeats" << i << ": " << lalondeFeats[i] << std::endl; 
            // std::cout << "lalondeHisto" << i << ": " << lalondeHisto[i] << std::endl; 
        }
    }
    
    // average the accumulated lalonde histogram
    if (valid_size == 0)
    {
        std::cerr << "lalondeFeat(): the valid size of the lalonde histogram is 0! return without doing anything! \n"
                    << std::endl;
        lalondeHisto.clear();
        return;
    }

    for (auto &r : lalondeHisto)
    {
        r /= valid_size;
    }
    // // debug
    // std::cout << "lalondeFeat(): valid_size: " << valid_size << std::endl; 
}



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
            )
{
    // transform cloud to matrix
    Eigen::MatrixXf IMat;
    cloud2Matrix(cloud, IMat, "i");

    // calculate max intensity, mean intensity
    Imax = IMat.maxCoeff();
    Imean = IMat.mean();

    // calculate intensity variance
    Eigen::MatrixXf ImeanMat;
    Eigen::MatrixXf::Ones(IMat.rows(), IMat.cols());
    ImeanMat.setOnes(IMat.rows(), IMat.cols());
    ImeanMat *= Imean;
    Ivar = (IMat - ImeanMat).squaredNorm() / IMat.rows();
}



/*
****************************************
****** Estimate moment invariants ******
****************************************
*/

void
momentInvariants(
                    const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud, // input cloud
                    std::vector<float> & jvec   // output moment invariants vector jvec[0] jvec[1] jvec[2]
                    )
{
    jvec = {0.0, 0.0, 0.0};
    pcl::MomentInvariantsEstimation<pcl::PointXYZI, pcl::MomentInvariants> mie;
    mie.computePointMomentInvariants(*cloud, jvec[0], jvec[1], jvec[2]);
    // // divided by point number in cloud
    // for (auto & j : jvec)
    //     j /= cloud->points.size();
}



/*
***********************************************
****** Calculate mean intensity gradient ******
***********************************************
*/

void
meanIntensityGradient(
                        const pcl::PointCloud<pcl::PointXYZI>::Ptr & inputCloud,  // input cloud
                        const pcl::PointCloud<pcl::Normal>::Ptr & inputNormals,   // input normals
                        const float & searchRadius,                    // param search radius
                        float & meanIG,                                // output meanIG
                        bool debug                                     // debug mode, default false
                        )
{
    pcl::IntensityGradientEstimation<   pcl::PointXYZI, 
                                        pcl::Normal, 
                                        pcl::IntensityGradient, 
                                        pcl::common::IntensityFieldAccessor<pcl::PointXYZI> >::Ptr 
    ige (new pcl::IntensityGradientEstimation<  pcl::PointXYZI, 
                                                pcl::Normal, 
                                                pcl::IntensityGradient, 
                                                pcl::common::IntensityFieldAccessor<pcl::PointXYZI> >);
    ige->setInputCloud(inputCloud);
    ige->setInputNormals(inputNormals);
    // ige->setNumberOfThreads(4U);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZI>);
    ige->setSearchMethod(kdtree);
    ige->setRadiusSearch(searchRadius);
    pcl::PointCloud<pcl::IntensityGradient>:: Ptr ig (new pcl::PointCloud<pcl::IntensityGradient>);
    ige->compute(*ig);
    // calculate the mean [valid] value of ig 
    // invalid value: NaN in the calculated intensity gradient ig
    unsigned int valid_size = 0;
    meanIG = 0.0;
    for (auto & igp : ig->points)
    {
        if (std::isnan(igp.gradient_x) || std::isnan(igp.gradient_y) || std::isnan(igp.gradient_z))
            continue;

        ++valid_size;
        // actually mean [squared] intensity gradient is calculated
        meanIG +=   igp.gradient_x * igp.gradient_x +
                    igp.gradient_y * igp.gradient_y +
                    igp.gradient_z * igp.gradient_z;  
    }
    // debug mode
    if (debug)
    {
        std::cout << "meanIntensityGradient(): debug: " << std::endl;
        std::cout << "total size of the calculated intensity gradient: "
                    << ig->points.size()
                    << std::endl;
        std::cout << "valid size of the calculated intensity gradient (without NaN): "
                    << valid_size
                    << std::endl;                    
    }
    // check, wether the valid_size is 0
    if (valid_size == 0)
    {
        std::cerr << "meanIntensityGradient(): the valid size of the intensity gradient is 0! return without doing anything! \n"
                    << std::endl;
        return;
    }

    meanIG /= valid_size;
}
