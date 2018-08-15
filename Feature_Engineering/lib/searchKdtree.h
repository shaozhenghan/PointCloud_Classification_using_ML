#ifndef KDTREESEARCH_H
#define KDTREESEARCH_H

#include "commonHeadFiles.h"
#include <pcl/kdtree/kdtree_flann.h>

/*
************************************************************************
********** kdtree search for searching points in neighborhood **********
************************************************************************
*/

// if points are succussfully searched, return ture; if not, return false
bool
kdtreeSearch (
                const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud, // input cloud for searching
                const pcl::PointXYZI & searchPoint,               // search point
                std::vector<int> & searchedPointIdx,        // output: point index of searched neighbot points
                const std::string & mode,                   // serch mode: k: nearestKSearch; r: radiusSearch
                const unsigned int & k = 1,                 // k nearest points for mode=k, default 1
                const float & r = 0.02                      // search radius for mode=r, default 0.02m
            );


#endif
