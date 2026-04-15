/*
 * @Author: tyz 1872516355@qq.com
 * @Date: 2026-04-13 22:21:35
 * @LastEditors: tyz 1872516355@qq.com
 * @LastEditTime: 2026-04-14 20:30:22
 * @FilePath: /平面拟合/main.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "segmentation.h"
// 显示点云
bool visPointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out){
    // 显示数据
    pcl::visualization::PCLVisualizer viewer;

    // 3. 创建两个视口
    int v1(0); // 左
    int v2(1); // 右
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1); // 左半
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2); // 右半

    // 设置背景颜色
    viewer.setBackgroundColor(0, 0, 0, v1);
    viewer.setBackgroundColor(0.1, 0.1, 0.1, v2);

    // 4. 添加点云（不同颜色）
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_in(cloud_in, 0, 255, 0);   // 绿色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_out(cloud_out, 255, 0, 0); // 红色
    viewer.addPointCloud<pcl::PointXYZ>(cloud_in, color_in, "input_cloud", v1);
    viewer.addPointCloud<pcl::PointXYZ>(cloud_out, color_out, "output_cloud", v2);

    // 点大小
    viewer.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "input_cloud");
    viewer.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "output_cloud");

    // 5. 添加文字
    viewer.addText("Input Cloud", 10, 10, "v1 text", v1);
    viewer.addText("Output Cloud", 10, 10, "v2 text", v2);

    while(!viewer.wasStopped()){
        viewer.spinOnce(100);
    }

    return true;
}

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
    // 结果可视化
    visPointcloud(cloud_in, cloud_out);
    /*
    // 圆柱拟合
    if(params.isCylindricalSegmentation){
        segmentation.ransacCylinderSegmentation(cloud_in, coefficients, cloud_out);
    }*/

    return 1;
}