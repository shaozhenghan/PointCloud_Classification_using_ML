#include "searchKdtree.h"

/*
************************************************************************
********** kdtree search for searching points in neighborhood **********
************************************************************************
*/

bool
kdtreeSearch (
                const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud, // input cloud for searching
                const pcl::PointXYZI & searchPoint,               // search point
                std::vector<int> & searchedPointIdx,     // output: point index of searched neighbot points
                const std::string & mode,                // serch mode: k: nearestKSearch; r: radiusSearch
                const unsigned int & k,                  // k nearest points for mode=k, defualt 1
                const float & r                          // search radius for mode=r, default 0.02m
            )
{
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud (cloud);
    std::vector<float>pointSqrDistance;
    // clear the possible invalid residual values in vector
    if (!searchedPointIdx.empty())
        searchedPointIdx.clear();
    
    if (mode == "r")
    {
        if (kdtree.radiusSearch(searchPoint,r, searchedPointIdx, pointSqrDistance) > 0)
            return (true);
        else
        {
            std::cerr << "kdtreeSearch(): no points are successfully searched!" << std::endl;
            return (false);
        }
    }
    
    else if (mode == "k")
    {
        if (kdtree.nearestKSearch(searchPoint, k, searchedPointIdx, pointSqrDistance) > 0)
            return (true);
        else
        {
            std::cerr << "kdtreeSearch(): no points are successfully searched!" << std::endl;
            return (false);
        }
    }

    else
    {
        std::cerr << "kdtreeSearch(): invalid mode!" << std::endl;
        std::cerr << "valid modes are:" << "\n"
                    << "k: nearestKSearch" << "\n" 
                    << "r: radiusSearch" << std::endl;
        return (false);
    }
    
}
