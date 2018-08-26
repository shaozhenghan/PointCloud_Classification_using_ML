#include "sampleCloud.h"

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
                bool debug                           // debug mode, default true
            )
{
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud (inputCloud);
    vg.setLeafSize (cube_leaf_side_length, cube_leaf_side_length, cube_leaf_side_length);
    // clear the possible invalid residual values in outputCloud
    if (!outputCloud->points.empty())
        outputCloud->points.clear();
    // compute downsampled cloud
    vg.filter (*outputCloud);

    // debug mode
    if (debug)
    {
        std::cout << "downSample_vg(): points size before downsampling: " 
                    << inputCloud->points.size() 
                    << std::endl;
        std::cout << "downSample_vg(): points size after downsampling: " 
                    << outputCloud->points.size() 
                    << std::endl;
    }
}



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
                    bool debug                          // debug mode, default true
                )
{
    size_t size = inputCloud->points.size();
    // clear the possible invalid residual values in outputCloud
    if (!outputCloud->points.empty())
        outputCloud->points.clear();
    // exception: size of the inputCloud <= highThreshold, no need to downsample
    if (size <= highThreshold)
        outputCloud = inputCloud; // that's why & outputCloud must have "&"
    else
    {
        // generate sequential index
        std::vector<unsigned int> idxSeq;
        for (unsigned int i = 0; i != size; ++i)
            idxSeq.push_back(i);
        // randomly shuffle index
        srand(time(NULL));
        std::random_shuffle(idxSeq.begin(), idxSeq.end());
        // get randomly downsampled outputCloud
        for (auto idx = idxSeq.cbegin(); idx != idxSeq.cbegin()+highThreshold; ++idx)
            outputCloud->points.push_back(inputCloud->points[*idx]);
        outputCloud->width = outputCloud->size();
        outputCloud->height = 1;
        outputCloud->is_dense = true;
    }
    // debug mode
    if (debug)
    {
        std::cout << "downSample_rand(): points size before downsampling: " 
            << inputCloud->points.size() 
            << std::endl;
        std::cout << "downSample_rand(): points size after downsampling: " 
            << outputCloud->points.size() 
            << std::endl;       
    }
}


/*
****************************************************************
********** Check the existence in vector of a element **********
****************************************************************
*/

// true: element exists in vector; false: not in vector
bool inVector(
                const NeighborPointIdxPair & PIdxPair,
                const std::vector<NeighborPointIdxPair> & PIdxPairVec
                )
{
    // if (PIdxPairVec.empty())
    // {
    //     std::cout << "inVector(): The input vector is empty!" << std::endl;
    //     return (false);
    // }
    auto it = std::find(PIdxPairVec.cbegin(), PIdxPairVec.cend(), PIdxPair);
    return (!(it == PIdxPairVec.cend()));
}



/*
*******************************************************
********** Upsampling for sparse point inputCloud **********
*******************************************************
*/

void
upSample(
            const pcl::PointCloud<pcl::PointXYZI>::Ptr & inputCloud,    // original cloud as input
            pcl::PointCloud<pcl::PointXYZI>::Ptr & outputCloud          // upsampled cloud as output
        )
{
    if (inputCloud->points.empty())
    {
        std::cout << "upSample(): The inputCloud is empty!" << std::endl;
        return;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr addedCloud (new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<NeighborPointIdxPair> PIdxPairVec;
    NeighborPointIdxPair PIdxPair;
    std::vector<int> searchedPointIdx;
    pcl::PointXYZI newPoint;
    for (std::size_t i = 0; i < inputCloud->points.size(); ++i)
    {
        // search mode: k search. number of points to search: 2   
        kdtreeSearch(inputCloud, inputCloud->points[i], searchedPointIdx, "k", 2);
        // searchedPointIdx[0] = i, searchedPointIdx[1] = index of the searched point
        PIdxPair.index1 = i;
        PIdxPair.index2 = searchedPointIdx[1];
        // // debug
        // std::cout << "PIdxPair.index1: " << PIdxPair.index1 << std::endl;
        // std::cout << "PIdxPair.index2: " << PIdxPair.index2 << std::endl;

        if (inVector(PIdxPair, PIdxPairVec))
            continue;
        PIdxPairVec.push_back(PIdxPair);
        newPoint.x = (inputCloud->points[i].x + inputCloud->points[searchedPointIdx[1]].x) / 2.0;
        newPoint.y = (inputCloud->points[i].y + inputCloud->points[searchedPointIdx[1]].y) / 2.0;
        newPoint.z = (inputCloud->points[i].z + inputCloud->points[searchedPointIdx[1]].z) / 2.0;
        newPoint.intensity = (inputCloud->points[i].intensity + 
                                inputCloud->points[searchedPointIdx[1]].intensity) / 2.0;
        addedCloud->points.push_back(newPoint);
    }
    addedCloud->width = addedCloud->points.size();
    addedCloud->height = 1;
    addedCloud->is_dense = true;
    // clear the possible invalid residual values in outputCloud
    if (!outputCloud->points.empty())
        outputCloud->points.clear();
    // concatenate the inputCloud and the addedCloud
    (*outputCloud) = (*inputCloud) + (*addedCloud); 
}
