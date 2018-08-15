/*
 *****************************************************
 ************ data augmentation for pointcloud *******
 *****************************************************
 */
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common_headers.h>
#include <fstream>
#include <Python.h>
#include "data_aug.h"

typedef pcl::PointXYZI PointT;

int 
main(int argc, char** argv)
{
    pcl::PCDReader reader;
    pcl::PCDWriter writer;   

    // argv[1]: one of {"car", "cyclist", "pedestrian", "truck", "van"}
    // number of the pcd-files in every folder("car", "cyclist", "pedestrian", "truck", "van")
    unsigned int num_files = 0;
    unsigned int augment_multiple = 1;
    float sigma_ = 0.0;
    float clip_ = 0.0;
    if(strcmp(argv[1], "car") == 0)
    {
        // num_files = 128;
        num_files = 56;
        augment_multiple = 6;
        sigma_ = 0.04;
        clip_ = 0.06;
    }
    
    else if (strcmp(argv[1], "cyclist") == 0)
    {
        // num_files = 28;
        num_files = 17;
        augment_multiple = 12;
        sigma_ = 0.02;
        clip_ = 0.05;
    }
    else if (strcmp(argv[1], "pedestrian") == 0)
    {
        // num_files = 81;
        num_files = 29;
        augment_multiple = 6;
        sigma_ = 0.01;
        clip_ = 0.05;
    }
    else if (strcmp(argv[1], "truck") == 0)
    {
        // num_files = 19;
        num_files = 9;
        augment_multiple = 20;
        sigma_ = 0.05;
        clip_ = 0.1;
    }
    else if (strcmp(argv[1], "van") == 0)
    {
        // num_files = 46;
        num_files = 23;
        augment_multiple = 12;
        sigma_ = 0.04;
        clip_ = 0.06;
    }
    else
    {
        // Exception
        // code here
        std::cerr << "Invalid input for argv[1]!" << std::endl;
        return(1);
    }

    std::string read_pcd_file_base_path = "/media/shao/TOSHIBA EXT/data_object_velodyne/Daten/test/test_data_original/";
    // pointer for the cloud in original pcd-file
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    // initialize Python module
    Py_Initialize();
    initdata_aug();
    // for-loop to augment every original pcd-file in the folder
    for(unsigned int file_index = 0; file_index < num_files; ++file_index)
    {
        // path of the file to be read
        std::stringstream read_pcd_file_path;
        read_pcd_file_path << read_pcd_file_base_path
                            << argv[1] 
                            << "/" 
                            << argv[1] 
                            << file_index 
                            << ".pcd";
        // read the pcd-file
        reader.read (read_pcd_file_path.str(), *cloud);
        // print the size of cloud
        unsigned int point_size = cloud->points.size();
        std::cout << "PointCloud in " 
                    << argv[1]
                    << file_index
                    << ".pcd "
                    <<"has: " 
                    << point_size
                    << " data points." 
                    << std::endl; 

        // for-loop for the Data Augmentation
        // every original pcd-file is augmented in [augment_multiple] new files.
        static unsigned int augment_file_index = num_files;
        for(unsigned int i = 1; i < augment_multiple; ++i, ++augment_file_index)
        {
            // randomly rotation with angle
            PyObject *angle = Py_BuildValue("f", 6.28*i/augment_multiple); // theta = 2*pi*i/10, i = 1,2,...,9
            // jitter the data with sigma, clip
            PyObject *sigma = Py_BuildValue("f", sigma_);
            PyObject *clip = Py_BuildValue("f", clip_);
            // augmented new cloud       
            pcl::PointCloud<PointT>::Ptr augmented_cloud (new pcl::PointCloud<PointT>);
            // augmentation for every point in original pcd-file
            for(unsigned int point_index = 0; point_index < point_size; ++point_index)
            {
                //std::cout << point_index << std::endl; // debug
                float x = cloud->points[point_index].x;
                float y = cloud->points[point_index].y;
                float z = cloud->points[point_index].z;
                PyObject *point = Py_BuildValue("[f,f,f]", x, y, z);
                
                PyObject *augmented_point = augment_data(point, angle, sigma, clip);
                // assert(PyList_Check(augmented_point));
                
                PyObject *pValue = PyList_GetItem(augmented_point, 0);
                PyObject *pValue_0 = PyList_GET_ITEM(pValue, 0);
                PyObject *pValue_1 = PyList_GET_ITEM(pValue, 1);
                PyObject *pValue_2 = PyList_GET_ITEM(pValue, 2);

                // augmented new point to be push back to the augmented_cloud.
                PointT point_new;
                // float x_a = PyFloat_AsDouble(pValue_0);
                // float y_a = PyFloat_AsDouble(pValue_1);
                // float z_a = PyFloat_AsDouble(pValue_2); 
                point_new.x = PyFloat_AsDouble(pValue_0);
                point_new.y = PyFloat_AsDouble(pValue_1);
                point_new.z = PyFloat_AsDouble(pValue_2);
                point_new.intensity = cloud->points[point_index].intensity;
                augmented_cloud->points.push_back(point_new);
            }
            // set the height and width for the augmented cloud. Important!!
            augmented_cloud->height = 1;
            augmented_cloud->width = augmented_cloud->points.size();
            // path of the file to be written
            std::string write_pcd_file_base_path = "/media/shao/TOSHIBA EXT/data_object_velodyne/Daten/test/test_data_augmented/";
            std::stringstream write_pcd_file_path;
            write_pcd_file_path << write_pcd_file_base_path
                                << argv[1]
                                << "/"
                                << argv[1]
                                << augment_file_index
                                << ".pcd";
            // write the file
            writer.write<PointT> (write_pcd_file_path.str(), *augmented_cloud, false);
            //std::cout << "bis hier" << i << std::endl; // debug
        }
    }  
    // finalize the python module 
    Py_Finalize();
    return(0);
}