#include "getFeatureVector.h"

/*
***********************************************
********** build feature vector ***************
***********************************************
*/

void
getFeatureVector (
                    const float & searchRadius,
                    bool is_training
    )
{
    // table_drive
    const std::string label_name[5] = {"car", "van", "pedestrian", "truck", "cyclist"};
    const unsigned int size_low_limit[5] = {1000, 1000, 150, 1000, 500};
    const unsigned int size_high_limit[5] = {2000, 2000, 1000, 3000, 1500};
    const unsigned int label[5] = {0, 1, 2, 3, 4};
    // path
    std::string read_file_base_path = "/media/shao/TOSHIBA EXT/data_object_velodyne/Daten";
    if (is_training)
        read_file_base_path += "/train/data_augmented";
    else
        read_file_base_path += "/test/data_augmented";
    
    for (int i = 0; i != 5; ++i)
    {
        // path
        std::string read_file_path = read_file_base_path + "/" + label_name[i];
        std::vector<std::string> file_name_vec;
        // get all pcd-files' name under the folder
        getFileName(read_file_path, file_name_vec);
        // traverse every pcd-file to build feature vector
        for (auto & f : file_name_vec)
        {
            // feature vector
            std::vector<float> featVec;
            // pcd file path
            std::string pcd_path = read_file_path + "/" + f;
            // read pcd data file
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
            readPCD(pcd_path, cloud);
            if (cloud->points.empty())
                continue;

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
            pcl::PointCloud<pcl::PointXYZI>::Ptr 
            cloud_sparse (new pcl::PointCloud<pcl::PointXYZI>);
            downSample_rand(cloud, cloud_sparse, size_high_limit[i], false);

            // calculate lalonde feature histogram
            std::vector<float> lalondeHisto;
            lalondeFeat(cloud_sparse, cloud, searchRadius, lalondeHisto);
            if (lalondeHisto.empty())
                continue;
            for (auto & lh : lalondeHisto)
                featVec.push_back(lh);

            // estimate normals
            pcl::PointCloud<pcl::Normal>::Ptr 
            normals (new pcl::PointCloud<pcl::Normal>);
            estimateNormals(cloud_sparse, cloud, searchRadius, normals);
            
            // remove NaN normals
            pcl::PointCloud<pcl::Normal>::Ptr 
            normals_valid (new pcl::PointCloud<pcl::Normal>);
            std::vector<int> mapping;
            removeNanNormals(normals, normals_valid, mapping);
            if (mapping.empty())
                continue;
            
            // remove point with NaN normal
            pcl::PointCloud<pcl::PointXYZI>::Ptr 
            cloud_valid (new pcl::PointCloud<pcl::PointXYZI>);                
            removePointWithNanNormal(cloud_sparse, cloud_valid, mapping);

            // estimate mean intensity gradient
            float meanIG = 0.0;
            meanIntensityGradient(cloud_valid, normals_valid, searchRadius, meanIG);
            featVec.push_back(meanIG);

            // estimate FPFH histogram
            std::vector<float> FPFHHisto;
            estimateFPFH(cloud_valid, normals_valid, searchRadius, FPFHHisto);
            if (FPFHHisto.empty())
                continue;
            for (auto & fh : FPFHHisto)
                featVec.push_back(fh);

            // check the size of the feature vector featVec
            assert(featVec.size() == 47);

            /****** save the calculated feature vector into disk ******/
            std::string write_file_base_path = 
            "/media/shao/TOSHIBA EXT/data_object_velodyne/feature_matrix_with_label";
            if (is_training)
                write_file_base_path += "/train";
            else
                write_file_base_path += "/test";
            std::stringstream rs;
            rs << searchRadius;
            std::string write_file_path = write_file_base_path + "/" + "r_" + rs.str() + ".txt";
            writeTXT(featVec, label[i], write_file_path);
        }
    } 
}