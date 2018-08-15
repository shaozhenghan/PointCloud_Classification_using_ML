#ifndef GETFEATUREVECTOR_H
#define GETFEATUREVECTOR_H

#include "accessFile.h"
#include "sampleCloud.h"
#include "extractFeature.h"

/*
***********************************************
********** build feature vector ***************
***********************************************
*/

void
getFeatureVector (
                    const float & searchRadius,
                    bool is_training
    );


#endif
