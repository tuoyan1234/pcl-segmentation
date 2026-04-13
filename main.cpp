#include "segmentation.h"

int main(){
    SegmentationParams params;
    PclSegmentation segmentation;
    // 读取参数
    std::string configPath = "../config/config.yaml";
    if (!segmentation.readParams(configPath,params)){
        return -1;
    }else{
        std::cout << "读取参数成功" << std::endl;
    }
    // 读取点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::io::loadPLYFile(params.inputPath, *cloud_in);
    // 平面拟合
    if(params.isPlaneSegmentation){
        segmentation.ransacPlaneSegmentation(cloud_in, coefficients, cloud_out);
    }
    // 直线拟合
    if(params.isLineSegmentation){
        segmentation.ransacLineSegmentation(cloud_in, coefficients, cloud_out);
    }
    /*
    // 圆柱拟合
    if(params.isCylindricalSegmentation){
        segmentation.ransacCylinderSegmentation(cloud_in, coefficients, cloud_out);
    }*/

    return 1;
}