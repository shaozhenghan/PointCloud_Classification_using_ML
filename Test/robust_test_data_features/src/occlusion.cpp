#include "getFeatureVector.h"
#include "gen_robust_test_data.h"

int main (void)
{
    // occlusion percentage vector
    const std::vector<float> occ_percen_vec {10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0}; // %
    // path
    std::string read_file_base_path = "/media/shao/TOSHIBA EXT/data_object_velodyne/Daten/test/data_augmented";

    for (auto & op : occ_percen_vec)
    {
        // file path to write
        std::string write_file_base_path = 
            "/media/shao/TOSHIBA EXT/data_object_velodyne/Daten/test/robust_test/occlusion";
        std::stringstream occper;
        occper << op;
        std::string write_file_path =   write_file_base_path + 
                                        "/" + 
                                        "op_" + 
                                        occper.str() + 
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
                // generate occluded point cloud
                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
                getOccludedCloud(cloud_original, label_name[i], op, cloud);
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