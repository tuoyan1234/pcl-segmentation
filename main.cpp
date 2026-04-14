/*
 * @Author: tyz 1872516355@qq.com
 * @Date: 2026-04-13 22:21:35
 * @LastEditors: tyz 1872516355@qq.com
 * @LastEditTime: 2026-04-14 20:30:22
 * @FilePath: /平面拟合/main.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
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