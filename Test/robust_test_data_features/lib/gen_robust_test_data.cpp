/* 
**************************************************
****** generate data (cloud) for robust test *****
**************************************************
*/


#include "gen_robust_test_data.h"

#define PI 3.1415926
#define HIGH_NOISE_LIMIT 0.12
#define LOW_NOISE_LIMIT -HIGH_NOISE_LIMIT

// occlusion with different percentages
void 
getOccludedCloud (
    const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
    const std::string & label_name, 
    const float & occlusion_percentage,
    pcl::PointCloud<pcl::PointXYZI>::Ptr & occluded_cloud
    )
{
    assert(occlusion_percentage >=0.0 && occlusion_percentage <= 100.0);

    float   mp = 0.0, 
            length = 0.0, 
            half_occluded_length = 0.0;

    if (label_name == "car" || label_name == "van" || label_name == "truck")
    {
        float minpx = cloud->points[0].x;
        float maxpx = cloud->points[0].x;
        float minpy = cloud->points[0].y;
        float maxpy = cloud->points[0].y;
        for (auto & p : cloud->points)
        {
            if (p.x > maxpx)
                maxpx = p.x;
            else if (p.x < minpx)
                minpx = p.x;
            if (p.y > maxpy)
                maxpy = p.y;
            else if (p.y < minpy)
                minpy = p.y;
        }
        if (maxpx-minpx > maxpy-minpy)
        {
            length = maxpx - minpx;
            mp = (maxpx + minpx) / 2.0;
            half_occluded_length = length * occlusion_percentage / 200.0;
            for (auto & p : cloud->points)
            {
                if (p.x <= mp+half_occluded_length && p.x >= mp-half_occluded_length)
                    continue;
                else
                    occluded_cloud->points.push_back(p);
            }
        }    
        else
        {
            length = maxpy-minpy;
            mp = (maxpy + minpy) / 2.0;
            half_occluded_length = length * occlusion_percentage / 200.0;
            for (auto & p : cloud->points)
            {
                if (p.y <= mp+half_occluded_length && p.y >= mp-half_occluded_length)
                    continue;
                else
                    occluded_cloud->points.push_back(p);
            }
        }
    }
    else if (label_name == "pedestrian" || label_name == "cyclist")
    {
        float maxpz = cloud->points[0].z;
        float minpz = cloud->points[0].z;
        for (auto & p : cloud->points)
        {
            if (p.z > maxpz)
                maxpz = p.z;
            else if (p.z < minpz)
                minpz = p.z;
        }
        length = maxpz - minpz;
        mp = (maxpz + minpz) / 2.0;
        half_occluded_length = length * occlusion_percentage / 200.0;
        for (auto & p : cloud->points)
        {
            if (p.z <= mp+half_occluded_length && p.z >= mp-half_occluded_length)
                continue;
            else
                occluded_cloud->points.push_back(p);
        }
    }
    else
    {
        std::cerr << "getOccludedCloud(): invalid label name!" << std::endl;
        return;
    }
    occluded_cloud->width = occluded_cloud->points.size();
    occluded_cloud->height = 1;
    occluded_cloud->is_dense = true;
}


// random downsample with different percentage
void
getSparseCloud (
    const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,      // input cloud
    const float & sparse_percentage,                         // percentage of points after downsampling
    pcl::PointCloud<pcl::PointXYZI>::Ptr & sparse_cloud      // output sparse cloud
    )
{
    assert(sparse_percentage >= 0 && sparse_percentage <= 100);

    unsigned int num_points_after_downsampled = sparse_percentage * cloud->points.size() / 100;
    downSample_rand(cloud, sparse_cloud, num_points_after_downsampled, false);
}


// prepare random number
void 
UNIFORM (float *p)
{
	int i = 0, a = 0, x = 0;
	double f;
	for (i = 0; i != 2; ++i, x += 689)
	{
		a = rand() + x;  // 加上689是因为系统产生随机数的更换频率远远不及程序调用函数的时间
		a = a % 1000;
		f = (float)a;
		f = f / 1000.0;
		*p = f;
		p++;
    }
}

// add gaussian noise to cloud using Box-Muller algorithm
void
addNoise (
    const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,      // input cloud
    const float & sigma,                                     // param standard variance
    pcl::PointCloud<pcl::PointXYZI>::Ptr & noise_cloud       // output cloud with noise  
    )
{
    float A = 0.0, B = 0.0, C = 0.0, r = 0.0;
	float uni[2];
    pcl::PointXYZI noise_point;
	srand((unsigned)time(NULL));
    if (!noise_cloud->points.empty())
        noise_cloud->points.clear();

    for (auto & p : cloud->points)
    {
        std::vector<float> rnv;
        for (unsigned int j = 0; j != 3; ++j)
        {
            // Box-Muller algorithm
            UNIFORM(uni);
            A = sqrt((-2)*log(uni[0]));
            B = 2 * PI*uni[1];
            C = A*cos(B);
            r = 0.0 + C * sigma; // meanvalue = 0.0, stdvar = sigma 
            // clip
            if (r > HIGH_NOISE_LIMIT)
                r = HIGH_NOISE_LIMIT;
            else if (r < LOW_NOISE_LIMIT)
                r = LOW_NOISE_LIMIT;
            rnv.push_back(r);
        }

        assert(rnv.size() == 3);
        
        noise_point.x = p.x + rnv[0];
        noise_point.y = p.y + rnv[1];
        noise_point.z = p.z + rnv[2];
        noise_point.intensity = p.intensity;
        noise_cloud->points.push_back(noise_point);
    }
    noise_cloud->width = noise_cloud->points.size();
    noise_cloud->height = 1;
    noise_cloud->is_dense = true;
}