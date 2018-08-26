#include "getFeatureVector.h"
#include "gen_robust_test_data.h"

int main (void)
{
    // standard variance of gaussian noise with mean value = 0
    const std::vector<float> sigma_vec {0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09};
    // path
    std::string read_file_base_path = "/media/shao/TOSHIBA EXT/data_object_velodyne/Daten/test/data_augmented";

    for (auto & sigma : sigma_vec)
    {
        // file path to write
        std::string write_file_base_path = 
            "/media/shao/TOSHIBA EXT/data_object_velodyne/Daten/test/robust_test/noise";
        std::stringstream ss;
        ss << sigma;
        std::string write_file_path =   write_file_base_path + 
                                        "/" + 
                                        "noi_" + 
                                        ss.str() + 
                                        ".txt";
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
                // pcd file path
                std::string pcd_path = read_file_path + "/" + f;
                // read pcd data file
                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_original (new pcl::PointCloud<pcl::PointXYZI>);
                readPCD(pcd_path, cloud_original);
                if (cloud_original->points.empty())
                    continue;
                // add gaussian noise to point cloud
                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
                addNoise(cloud_original, sigma, cloud);
                if (cloud->points.empty())
                    continue;
                // calculate or estimate feature vector
                std::vector<float> featVec;
                getFeatureVector(0.16, cloud, i, featVec);

                // save the calculated feature vector into disk, when featVec is valid
                if (featVec.size() == 47)
                    writeTXT(featVec, label[i], write_file_path); 
            }
        }
    }
}