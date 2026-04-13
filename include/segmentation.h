/*
 * @Author: tyz 1872516355@qq.com
 * @Date: 2026-04-13 21:51:49
 * @LastEditors: tyz 1872516355@qq.com
 * @LastEditTime: 2026-04-13 22:51:41
 * @FilePath: /Desktop/平面拟合/include/segmentation.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _SEGMENTATION_H
#define _SEGMENTATION_H
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
// 平面提取
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
// 可视化
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
// yaml
#include <yaml-cpp/yaml.h>

#include <iostream>

struct SegmentationParams{
    bool isPlaneSegmentation = false;
    bool isLineSegmentation = false;
    bool isCylindricalSegmentation = false;
    std::string inputPath = "../data/pointclou.ply";
    std::string outputPath = "../data/pointcloud_output.ply";
    // 超参数
    float disThreshold = 0.1;
    float maxIterationCount = 10;
};
class PclSegmentation{
public:
    PclSegmentation();
    ~PclSegmentation();
    // 平面拟合
    bool ransacPlaneSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::ModelCoefficients::Ptr coefficients, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out);
    // 直线拟合
    bool ransacLineSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::ModelCoefficients::Ptr coefficients, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out);
    // 圆柱拟合
    bool ransacCylinderSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::ModelCoefficients::Ptr coefficients, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out);
    // 参数读取
    bool readParams(const std::string &file_name,SegmentationParams &params);
}; // CLASS PclSegmentation
#endif