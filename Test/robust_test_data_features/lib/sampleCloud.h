#ifndef PREPROCESSING_H
#define PREPROCESSING_H

#include "commonHeadFiles.h"

#include <pcl/filters/voxel_grid.h>
#include "searchKdtree.h"
#include <ctime>


/*
*************************************************
********** Down sample with Voxel Grid **********
*************************************************
*/

void
downSample_vg(
                const pcl::PointCloud<pcl::PointXYZI>::Ptr & inputCloud, // input inputCloud before downsampling
                pcl::PointCloud<pcl::PointXYZI>::Ptr & outputCloud,     // output inputCloud after downsampling
                const float & cube_leaf_side_length, // param side length of cube leaf of voxel grid [m] 
                bool debug = true                    // debug mode, default true
            );



/*
******************************************************
********** Down sample with random sampling **********
******************************************************
*/

void
downSample_rand(
                    const pcl::PointCloud<pcl::PointXYZI>::Ptr & inputCloud, // input inputCloud before downsampling
                    pcl::PointCloud<pcl::PointXYZI>::Ptr & outputCloud,     // output inputCloud after downsampling
                    const unsigned int & highThreshold,    // number of points in outputCloud after downsampling
                    bool debug = true                         // debug mode, default true
                );




// struct //
struct NeighborPointIdxPair
{
    int index1;
    int index2;
    bool operator==(const NeighborPointIdxPair & pair) const
    {
        return (pair.index1 == this->index1 &&
                pair.index2 == this->index2)||
               (pair.index1 == this->index2 &&
                pair.index2 == this->index1);
    }
    bool operator!=(const NeighborPointIdxPair & pair) const 
    {
        // return (pair.index1 != this->index1 ||
        //         pair.index2 != this->index2)&&
        //        (pair.index1 != this->index2 ||
        //         pair.index2 != this->index1);
        return !(this->operator==(pair));
    }
};


/*
****************************************************************
********** Check the existence in vector of a element **********
****************************************************************
*/

// true: element exists in vector; false: not in vector
bool inVector(
                const NeighborPointIdxPair & PIdxPair,
                const std::vector<NeighborPointIdxPair> & PIdxPairVec
                );



/*
************************************************************
********** Upsampling for sparse point inputCloud **********
************************************************************
*/

void
upSample(
            const pcl::PointCloud<pcl::PointXYZI>::Ptr & inputCloud,    // original cloud as input
            pcl::PointCloud<pcl::PointXYZI>::Ptr & outputCloud          // upsampled cloud as output
        );

#endif
