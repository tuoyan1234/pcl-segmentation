/*
 * @Author: tyz 1872516355@qq.com
 * @Date: 2026-04-13 21:51:59
 * @LastEditors: tyz 1872516355@qq.com
 * @LastEditTime: 2026-04-13 23:02:08
 * @FilePath: /Desktop/平面拟合/src/segmentation.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "segmentation.h"

PclSegmentation::PclSegmentation(){}
PclSegmentation::~PclSegmentation(){}

bool PclSegmentation::ransacPlaneSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::ModelCoefficients::Ptr coefficients, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out){
    // 检查输入点云大小
    if(cloud_in->points.size() < 3){
        std::cout << "点云数量小于3,无法进行平面拟合" << std::endl;

        return false;
    }
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(50);
    seg.setDistanceThreshold(0.05);
    seg.setInputCloud(cloud_in);
    seg.segment(*inliers, *coefficients);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_in);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_out);
    std::cout  << "平面系数: " << coefficients->values[0] << " "    << coefficients->values[1] << " " << coefficients->values[2] << " " << coefficients->values[3] << std::endl;
    // 点云拟合数量
    std::cout << "平面内点云数量: " << cloud_out->points.size() << std::endl;
    return true;
}
// 直线拟合
bool PclSegmentation::ransacLineSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::ModelCoefficients::Ptr coefficients, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out){
    // 检查输入点云大小
    if(cloud_in->points.size() < 2){
        std::cout << "点云数量小于3,无法进行直线拟合" << std::endl;

        return false;
    }
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(10);
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud(cloud_in);
    seg.segment(*inliers, *coefficients);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_in);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_out);
    std::cout  << "直线系数: " << coefficients->values[0] << " "    << coefficients->values[1] << " " << coefficients->values[2] << " " << coefficients->values[3] << std::endl;
    std::cout << "直线拟合点云数量: " << cloud_out->points.size() << std::endl;

    return true;
}
// 圆柱拟合
/*  
bool ransacCylinderSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::ModelCoefficients::Ptr coefficients, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out){
    if(cloud_in->points.size() < 2){
        std::cout << "点云数量小于2，无法进行圆柱拟合" << std::endl;

        return false;
    }
    // 法线估计
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud_in);
    ne.setKSearch(50);
    ne.compute(*cloud_normals);

    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(10);
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud(cloud_in);
    seg.setInputNormals(cloud_normals);
    seg.segment(*inliers, *coefficients);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_in);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_out);
    std::cout << "圆柱拟合结果：" << coefficients->values[0] << " " << coefficients->values[1] << " " << coefficients->values[2] << " " << coefficients->values[3] << " " << coefficients->values[4] << " " << coefficients->values[5] << std::endl;
    // 圆柱拟合点云数量
    std::cout << "圆柱拟合点云数量：" << cloud_out->points.size() << std::endl;

    return true;
}*/
// 参数读取
bool PclSegmentation::readParams(const std::string &file_name,SegmentationParams &params){
    YAML::Node config = YAML::LoadFile(file_name);
    // isPlaneSegmentation
    if(config["isPlaneSegmentation"].IsDefined()){
        params.isPlaneSegmentation = config["isPlaneSegmentation"].as<bool>();
    }else{
        std::cout << "未找到参数isPlaneSegmentation" << std::endl;
        return false;
    }
    // isLineSegmentation
    if(config["isLineSegmentation"].IsDefined()){
        params.isLineSegmentation = config["isLineSegmentation"].as<bool>();
    }else{
        std::cout << "未找到参数isLineSegmentation" << std::endl;
        return false;
    }
    // isCylinderSegmentation
    if(config["isCylindricalSegmentation"].IsDefined()){
        params.isCylindricalSegmentation = config["isCylindricalSegmentation"].as<bool>();
    }else{
        std::cout << "未找到参数isCylindricalSegmentation" << std::endl;
        return false;
    }
    // inputPath
    if(config["inputPath"].IsDefined()){
        params.inputPath = config["inputPath"].as<std::string>();
    }else{
        std::cout << "未找到参数inputPath" << std::endl;
        return false;
    }
    // outputPath
    if(config["outputPath"].IsDefined()){
        params.outputPath = config["outputPath"].as<std::string>();
    }else{
        std::cout << "未找到参数outputPath" << std::endl;
        return false;
    }
    // disThreshold
    if(config["disThreshold"].IsDefined()){
        params.disThreshold = config["disThreshold"].as<double>();
    }else{
        std::cout << "未找到参数disThreshold" << std::endl;
        return false;
    }
    // maxIterationCount
    if(config["maxIterationCount"].IsDefined()){
        params.maxIterationCount = config["maxIterationCount"].as<int>();
    }else{
        std::cout << "未找到参数maxIterationCount" << std::endl;
        return false;
    }

    return true;
}