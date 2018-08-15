#ifndef ACCESSFILE_H
#define ACCESSFILE_H

#include "commonHeadFiles.h"

#include <pcl/io/pcd_io.h>
#include <fstream>
#include <Python.h>
#include "get_file_name.h"



/*
****************************************
********** Read pcd file ***************
****************************************
*/
void 
readPCD(
        const std::string & filename,
        pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud
        );


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
        );

// pcl::PointCloud<pcl::Normal>
void
writePCD(
        const std::string & filename,
        const pcl::PointCloud<pcl::Normal>::Ptr & normals
        );


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
        );



/*
****************************************
********** Get file name ***************
****************************************
*/
void 
getFileName(
            const std::string & path,
            std::vector<std::string> & file_name_vec
            );

#endif
