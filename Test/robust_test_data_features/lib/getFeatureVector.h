#ifndef GETFEATUREVECTOR_H
#define GETFEATUREVECTOR_H

#include "accessFile.h"
#include "sampleCloud.h"
#include "extractFeature.h"



// global variables for table drive
const std::string label_name[5] = {"car", "van", "pedestrian", "truck", "cyclist"};
const unsigned int size_low_limit[5] = {1000, 1000, 150, 1000, 500};
const unsigned int size_high_limit[5] = {2000, 2000, 1000, 3000, 1500};
const unsigned int label[5] = {0, 1, 2, 3, 4};

/*
***********************************************
********** build feature vector ***************
***********************************************
*/

void
getFeatureVector (
                    const float & searchRadius,                     // param search radius
                    pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,   // input cloud
                    const int & i,                                  // param label name index
                    std::vector<float> & featVec                    // output feature vector
    );


#endif
