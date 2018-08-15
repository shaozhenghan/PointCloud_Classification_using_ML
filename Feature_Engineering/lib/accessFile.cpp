#include "accessFile.h"

/*
****************************************
********** Read pcd file ***************
****************************************
*/
void 
readPCD(
        const std::string & filename,
        pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud
        )
{
    pcl::PCDReader reader;
    reader.read (filename, *cloud);
    std::cout << "readPCD(): "
                << cloud->points.size() 
                << " points in " 
                << filename
                << std::endl;
}


/*
****************************************
********** Write pcd file **************
****************************************
*/

// pcl::PointCloud<pcl::PointXYZI>
void
writePCD(
        const std::string & filename,
        const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud
        )
{
    if (cloud->points.empty())
    {
        std::cout << "writePCD(): There is no points in this cloud!" << std::endl;
        return;
    }
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZI> (filename, *cloud, false);
}

// pcl::PointCloud<pcl::Normal>
void
writePCD(
        const std::string & filename,
        const pcl::PointCloud<pcl::Normal>::Ptr & normals
        )
{
    if (normals->points.empty())
    {
        std::cout << "writePCD(): There is no points in this cloud!" << std::endl;
        return;
    }
    pcl::PCDWriter writer;
    writer.write<pcl::Normal> (filename, *normals, false);
}


/*
*******************************************************
********** Write txt file with "Append Mode" **********
*******************************************************
*/
void 
writeTXT(
        const std::vector<float> & globalFeatureVector,
        const unsigned int & label,
        const std::string & txt_path
        )
{
    std::ofstream writeTXT;
    writeTXT.open(txt_path.c_str(), std::ios::app);
    for (auto & e : globalFeatureVector)
    {
        writeTXT << e << ' ';
    }
    writeTXT << label << "\n";
}


/*
****************************************
********** Get file name ***************
****************************************
*/
void 
getFileName(
            const std::string & path,
            std::vector<std::string> & file_name_vec
            )  
{
    // use Cython so that C++ can call Python function

    // initialize Python module
    Py_Initialize();
    initget_file_name();

    // get the parameter and returned value in python format
    PyObject * path_ = Py_BuildValue("s", path.c_str());
    PyObject * file_name_list = get_file_name(path_);
    assert(PyList_Check(file_name_list));

    // build the file name vector
    Py_ssize_t size = PyList_Size(file_name_list);
    for (unsigned int file_index = 0; file_index != size; ++file_index)
    {
        PyObject * file_name_ = PyList_GetItem(file_name_list, file_index);
        std::string file_name = PyString_AsString(file_name_);
        file_name_vec.push_back(file_name);
    }
    
    // finalize the python module 
    Py_Finalize();
}
