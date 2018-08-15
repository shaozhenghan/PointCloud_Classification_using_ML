/* ******** test every module ********** */

#include "accessFile.h"
#include <sstream>
#include "sampleCloud.h"
#include "extractFeature.h"
#include "searchKdtree.h"

int
main(void)
{
    // /*
    // * Test functions: readPCD and writePCD *
    // */
    // std::stringstream r_filename, w_filename;
    // r_filename << "car1.pcd";
    // w_filename << "car1_w.pcd";
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    // readPCD(r_filename.str(), cloud);
    // writePCD(w_filename.str(), cloud);




//     /*
//     * Test function: writeTXT *
//     */
//    std::vector<float> featureVector {1,2,3,4,5,6,5,4,3,2,1};
//    for (unsigned int label = 0; label < 4; ++label)
//    {
//        const std::string txt_path = "/media/shao/TOSHIBA EXT/data_object_velodyne/feature_matrix_with_label/test.txt";
//        writeTXT(featureVector, label, txt_path);
//    }






    // /*
    // * Test function: getFileName *
    // */
    // std::string path = "/home/shao/文档/VSCodeWS/Masterarbeit_Code/PointCloud_FeatureEngineering/dataset_example";
    // std::vector<std::string> file_name_vec;
    // getFileName(path, file_name_vec);
    // for (auto & f : file_name_vec)
    //     std::cout << f << std::endl;





    // /*
    // * Test function: downSample_vg *
    // */    
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZI>);
    // readPCD("car1.pcd", cloud);
    // downSample_vg(cloud, cloud_downsampled, 0.2, true);
    // writePCD("car1_downsampled.pcd", cloud_downsampled);




    // /*
    // * Test function: kdtreeSearch *
    // */
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_searched (new pcl::PointCloud<pcl::PointXYZI>);
   
    // readPCD("car1.pcd", cloud);
    // pcl::PointXYZI searchPoint = cloud.points[0];
    // std::string mode = "x";
    // std::vector<int> index;

    // kdtreeSearch(cloud, searchPoint, index, mode, 100, 0);

    // std::cout << "number of searched points: " << index.size() << std::endl;

    // for (auto it = index.cbegin(); it != index.cend(); ++it)
    // {
    //     cloud_searched.points.push_back(cloud.points[*it]);
    // }
    // cloud_searched.width = cloud_searched.points.size();
    // cloud_searched.height = 1;

    // writePCD("car1_kdtree.pcd", cloud_searched);






    // /*
    // * Test function: estimateNormals, removeNanNormals, removePointWithNanNormal, visualizeNormals *
    // */
    // // estimateNormals
    // pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud (new pcl::PointCloud<pcl::PointXYZI>);
    // readPCD("car742.pcd", inputCloud);
    // pcl::PointCloud<pcl::PointXYZI>::Ptr searchSurface (new pcl::PointCloud<pcl::PointXYZI>);
    // searchSurface = inputCloud;
    // float searchRadius = 0.08;
    // pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    // estimateNormals(inputCloud, searchSurface, searchRadius, normals, false);
    // writePCD("car742_Normals.pcd", normals);
    // // delete searchSurface; // shared_ptr 自动释放内存，不能手动delete

    // // removeNanNormals
    // pcl::PointCloud<pcl::Normal>::Ptr noNaNnormals (new pcl::PointCloud<pcl::Normal>);
    // std::vector<int> mapping;
    // removeNanNormals(normals, noNaNnormals, mapping);

    // // removePointWithNanNormal
    // pcl::PointCloud<pcl::PointXYZI>::Ptr noNanNormalsCloud (new pcl::PointCloud<pcl::PointXYZI>);
    // removePointWithNanNormal(inputCloud, noNanNormalsCloud, mapping, true);
    // writePCD("car742_noNanNormalsCloud.pcd", noNanNormalsCloud);
    // writePCD("car742_noNanNormals.pcd", noNaNnormals);

    // // visualizeNormals
    // visualizeNormals(noNanNormalsCloud, noNaNnormals);






    // /*
    // * Test function: inVector, upSample 1*
    // */
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    // readPCD("car2.pcd", cloud); // car2.pcd is sparse
    // std::cout << "cloud size before upsampling: " << cloud->points.size() << std::endl;
    // pcl::PointCloud<pcl::PointXYZI>::Ptr outcloud (new pcl::PointCloud<pcl::PointXYZI>);
    // upSample(cloud, outcloud);
    // std::cout << "cloud size after upsampling: " << outcloud->points.size() << std::endl;
    // writePCD("car2_dense.pcd", outcloud);
    // std::cout << cloud->points[348].x << " "
    //             << cloud->points[348].y << " "
    //             << cloud->points[348].z << " "
    //             << cloud->points[348].intensity << std::endl;





    // /*
    // * Test function: inVector, upSample 2*
    // */
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    // readPCD("/media/shao/TOSHIBA EXT/data_object_velodyne/Daten/train/data_augmented/car/car2.pcd", cloud);
    // while (cloud->points.size() < 5000)
    // {
    //     pcl::PointCloud<pcl::PointXYZI>::Ptr 
    //     cloud_dense (new pcl::PointCloud<pcl::PointXYZI>);
    //     upSample(cloud, cloud_dense);
    //     cloud->points = cloud_dense->points;
    // }
    // cloud->width = cloud->points.size();
    // cloud->height = 1;
    // cloud->is_dense = true;
    // std::cout << "cloud size after upsampling: " << cloud->points.size() << std::endl;
    // writePCD("car2_dense.pcd", cloud);



    // /*
    // * Test function: cloud2Matrix *
    // */
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    // readPCD("car2.pcd", cloud);
    // Eigen::MatrixXf pointMatrix;
    // cloud2Matrix(cloud, pointMatrix, "xyz");
    // std::cout << pointMatrix << std::endl;





//     /*
//     * Test function: covMatrix *
//     */
//    Eigen::MatrixXf M(10,2);
//    M << 3.7, 1.7,
//         4.1, 3.8,
//         4.7, 2.9,
//         5.2, 2.8,
//         6.0, 4.0,
//         6.3, 3.6,
//         9.7, 6.3,
//         10.0, 4.9,
//         11.0, 3.6,
//         12.5, 6.4;
//    Eigen::MatrixXf Cov;
//    covMatrix(M, Cov);
//    // ground truth:
//    // 9.0836  3.365
//    // 3.365  2.016
//    std::cout << Cov << std::endl;






    // /*
    // * Test function: geometrySize *
    // */
    // Eigen::MatrixXf M(10,3);
    // M << 3.7, 1.7, 0,
    //     4.1, 3.8, 0,
    //     4.7, 2.9, 0,
    //     5.2, 2.8, 5,
    //     6.0, 4.0, 6,
    //     6.3, 3.6, 7,
    //     9.7, 6.3, 7,
    //     10.0, 4.9, 3,
    //     11.0, 3.6, 10,
    //     12.5, 6.4, 1.1;

    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    // pcl::PointXYZI point;
    // for(int i = 0; i != 10; ++i)
    // {
    //     point.x = M(i, 0);
    //     point.y = M(i, 1);
    //     point.z = M(i, 2);
    //     point.intensity = 0.0;
    //     cloud->points.push_back(point);
    // }
    // cloud->height = 1;
    // cloud->width = cloud->points.size();
    // cloud->is_dense = true;

    // float length = 0.0, width = 0.0, height = 0.0;
    // geometrySize(cloud, length, width, height);
    // // ground truth: length = 9.916, width = 2.989, height = 10
    // std::cout << length << std::endl;
    // std::cout << width << std::endl;
    // std::cout << height << std::endl;





    // /*
    // * Test function: geoCovEigen 1*
    // */

    // Eigen::MatrixXf M(10,3);
    // M << 3.7, 1.7, 0,
    //     4.1, 3.8, 0,
    //     4.7, 2.9, 0,
    //     5.2, 2.8, 0,
    //     6.0, 4.0, 0,
    //     6.3, 3.6, 0,
    //     9.7, 6.3, 0,
    //     10.0, 4.9, 0,
    //     11.0, 3.6, 0,
    //     12.5, 6.4, 0;

    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    // pcl::PointXYZI point;
    // for(int i = 0; i != 10; ++i)
    // {
    //     point.x = M(i, 2);
    //     point.y = M(i, 1);
    //     point.z = M(i, 0);
    //     point.intensity = 0.0;
    //     cloud->points.push_back(point);
    // }
    // cloud->height = 1;
    // cloud->width = cloud->points.size();
    // cloud->is_dense = true;
    // std::vector<float> eigvals {1,2,3};
    // geoCovEigen(cloud, eigvals);
    // geoCovEigen(cloud, eigvals);
    // // ground truth: 0.939624>=0.0603762>=0
    // std::cout << eigvals[0] << ">=" << eigvals[1] << ">=" << eigvals[2] << std::endl; 






    // /*
    // * Test function: geoCovEigen 2: gloabal; *
    // */
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    // readPCD("car742.pcd", cloud);
    // std::vector<float> eigvals {1,2,3};
    // geoCovEigen(cloud, eigvals, true);
    // std::cout << "eigen values of global covariance matrix before downsampling: " << std::endl;
    // std::cout << eigvals[0] << ">=" << eigvals[1] << ">=" << eigvals[2] << std::endl; 

    // std::vector<float> mean_eigvals {0.0, 0.0, 0.0};
    // for (int i = 0; i != 1000; ++i)
    // {
    //     pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZI>);
    //     downSample_rand(cloud, cloud_downsampled, 500);
    //     // downSample_vg(cloud, cloud_downsampled, 0.2, 0.2, 0.2);
    //     geoCovEigen(cloud_downsampled, eigvals, true);
    //     for (size_t j = 0; j != eigvals.size(); ++j)
    //         mean_eigvals[j] += eigvals[j];
    // }
    // for (auto & e : mean_eigvals)
    //     e /= 1000;

    // std::cout << "mean eigen values of global covariance matrix after downsampling over 1000 running: " 
    //             << std::endl;
    // std::cout << mean_eigvals[0] << ">=" << mean_eigvals[1] << ">=" << mean_eigvals[2] << std::endl; 






    // /*
    // * Test function: lalondeFeat *
    // */    
    // pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud (new pcl::PointCloud<pcl::PointXYZI>);
    // pcl::PointCloud<pcl::PointXYZI>::Ptr searchSurface (new pcl::PointCloud<pcl::PointXYZI>);
    // // readPCD("car2.pcd", inputCloud);
    // // readPCD("car2_dense.pcd", searchSurface);
    // readPCD("pedestrian0.pcd", inputCloud);
    // readPCD("pedestrian0.pcd", searchSurface);  
    // std::vector<float> lalondeHisto;
    // float r = 0.1; // searchRadius = 0.1m 此参数特别关键
    // lalondeFeat(inputCloud, searchSurface, r, lalondeHisto);
    // for(auto & r : lalondeHisto)
    //     std::cout << r << " ";
    // std::cout << "\n";






    // /*
    // * Test function: intensity *
    // */
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    // // readPCD("car742.pcd", cloud);
    // pcl::PointXYZI point;
    // for (unsigned int i = 1; i !=4; ++i)
    // {
    //     point.x = 0;
    //     point.y = 0;
    //     point.z = 0;
    //     point.intensity = i;
    //     cloud->points.push_back(point);
    // }
    // cloud->width = cloud->points.size();
    // cloud->height = 1;
    // cloud->is_dense = true;
    // float Imax = 0.0, Imean = 0.0, Ivar = 0.0;
    // intensity(cloud, Imax, Imean, Ivar);
    // // ground truth: Imax=3, Imean=2, Ivar = 0.6667;
    // std::cout << Imax << " " << Imean << " " << Ivar << std::endl;






    // /*
    // * Test function: momentInvariants  and downSample_vg, downSample_rand *
    // */
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    // readPCD("car742.pcd", cloud);
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZI>);
    // // downSample_vg(cloud, cloud_downsampled, 0.05);
    // downSample_rand(cloud, cloud_downsampled, 1000);
    // writePCD("car742_sparse.pcd", cloud_downsampled);
    // std::vector<float> j;
    // size_t size_m = cloud->points.size();
    // size_t size_f = cloud_downsampled->points.size();
    // momentInvariants(cloud, j);
    // std::cout << "moment invriants before downsampling: " << "\n";
    // std::cout << j[0]/size_m << " " << j[1]/size_m << " " << j[2]/size_m << std::endl;
    // momentInvariants(cloud_downsampled, j);
    // std::cout << "moment invriants after downsampling: " << "\n";
    // std::cout << j[0]/size_f << " " << j[1]/size_f << " " << j[2]/size_f << std::endl;






    // /*
    // * Test function: estimateFPFH part*
    // */
    // pcl::PointCloud<pcl::FPFHSignature33> fpfh;
    // fpfh.height = 1;
    // fpfh.width = 3;
    // fpfh.is_dense = true;
    // fpfh.points.resize(fpfh.height * fpfh.width);
    // for (int i = 0;  i != fpfh.points.size(); ++i)
    //     for (int j = 0; j != 33; ++j)
    //         fpfh.points[i].histogram[j] = j + i + 1;
    // std::vector<float> FPFHHisto {3,2,1};
    // // build FPFH histogram
    // if (!FPFHHisto.empty())
    //     FPFHHisto.clear();
    // size_t size = fpfh.points.size();
    // float mean = 0.0;
    // for (unsigned int i = 0; i != 33; ++i)
    // {
    //     mean = 0.0;
    //     for (auto & r : fpfh.points)
    //         mean += r.histogram[i];
    //     mean /= size;
    //     FPFHHisto.push_back(mean);
    // }
    // // ground truth: 2,3,4,5,...,34
    // for (auto & r : FPFHHisto)
    //     std::cout << r << " ";
    // std::cout << "\n" << std::endl;






    // /*
    // * Test function: estimateFPFH full*
    // */
    // pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud (new pcl::PointCloud<pcl::PointXYZI>);
    // pcl::PointCloud<pcl::PointXYZI>::Ptr searchSurface (new pcl::PointCloud<pcl::PointXYZI>);
    // float searchRadius = 0.2;
    // pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    // readPCD("car742.pcd", inputCloud);
    // searchSurface = inputCloud;
    // estimateNormals(inputCloud, searchSurface, searchRadius, normals);
    // std::vector<int> mapping;
    // pcl::PointCloud<pcl::Normal>::Ptr outputNormals (new pcl::PointCloud<pcl::Normal>);
    // removeNanNormals(normals, outputNormals, mapping);
    // pcl::PointCloud<pcl::PointXYZI>::Ptr outputCloud (new pcl::PointCloud<pcl::PointXYZI>);
    // removePointWithNanNormal(inputCloud, outputCloud, mapping);
    // std::vector<float> FPFHHisto;
    // estimateFPFH(outputCloud, outputNormals, searchRadius/2, FPFHHisto, true);
    // for (auto & r : FPFHHisto)
    //     std::cout << r << " ";
    // std::cout << "\n" << std::endl;






    // /*
    // * Test function: meanIntensityGradient*
    // */
    // pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud (new pcl::PointCloud<pcl::PointXYZI>);
    // pcl::PointCloud<pcl::PointXYZI>::Ptr searchSurface (new pcl::PointCloud<pcl::PointXYZI>);
    // float searchRadius = 0.08;
    // pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    // readPCD("car1.pcd", inputCloud);
    // searchSurface = inputCloud;
    // estimateNormals(inputCloud, searchSurface, searchRadius, normals, true);
    // std::vector<int> mapping;
    // pcl::PointCloud<pcl::Normal>::Ptr outputNormals (new pcl::PointCloud<pcl::Normal>);
    // removeNanNormals(normals, outputNormals, mapping);
    // pcl::PointCloud<pcl::PointXYZI>::Ptr outputCloud (new pcl::PointCloud<pcl::PointXYZI>);
    // removePointWithNanNormal(inputCloud, outputCloud, mapping, true);
    // // compute mean intensity gradient (squared)
    // float meanIG = 0.0;
    // meanIntensityGradient(outputCloud, outputNormals, searchRadius, meanIG, true);
    // std::cout << meanIG << std::endl;
}
