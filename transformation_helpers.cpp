// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "transformation_helpers.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <vector>
#include "KinectImage.h"
#include "KinectClient.h"
#include <mutex>


// 降采样
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <string>
#include <pcl/filters/random_sample.h>

#include <random>

// 点云变换坐标系
#include <Eigen/Dense>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>

// icp精配准
#include <pcl/registration/icp.h>


// 读取变换矩阵
void readMatrixFromFile(const std::string& file_path, Eigen::Matrix4f& transformation_matrix) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << file_path << std::endl;
        return;
    }

    Eigen::Matrix3f rotation_matrix;
    Eigen::Vector3f translation_vector;

    for (int i = 0; i < 3; ++i) {
        file >> rotation_matrix(i, 0) >> rotation_matrix(i, 1) >> rotation_matrix(i, 2) >> translation_vector(i);
    }

    transformation_matrix.block<3, 3>(0, 0) = rotation_matrix;
    transformation_matrix.block<3, 1>(0, 3) = translation_vector;
    transformation_matrix.row(3) << 0, 0, 0, 1;

    file.close();
}

void tranformation_helpers_write_point_cloud(std::vector<WritePointCloudData>& WritePCD)
{
    std::vector<color_point_t> points0, points1, points2;

    for (size_t index = 0; index < WritePCD.size(); ++index) {
        auto& data = WritePCD[index];
        std::vector<color_point_t>* current_points;

        if (index == 0) {
            current_points = &points0;
        }
        else if (index == 1) {
            current_points = &points1;
        }
        else {
            current_points = &points2;
        }

        int width = k4a_image_get_width_pixels(data.point_cloud_image);
        int height = k4a_image_get_height_pixels(data.color_image);

        int16_t* point_cloud_image_data = (int16_t*)(void*)k4a_image_get_buffer(data.point_cloud_image);
        uint8_t* color_image_data = k4a_image_get_buffer(data.color_image);

        // 初始化随机数生成器
        std::random_device rd;  // 随机数种子
        std::mt19937 gen(rd()); // 标准 mersenne_twister_engine
        std::uniform_real_distribution<> dis(0.0, 1.0);

        auto start_time = std::chrono::high_resolution_clock::now();

        Eigen::Matrix4f transformation_matrix;

        if (data.matrix_file != "") {
            // 读取变换矩阵
            readMatrixFromFile(data.matrix_file, transformation_matrix);
        }

        int a[2] = { 1 ,7 }; // 范围0到9
        int t = 0; // 范围0到2

        for (int i = 0; i < width * height; i++)
        {
            if (i % 10 == a[t]) {
                a[t] = (a[t] + 1) % 10;
                t = (t + 1) % 2;
                continue;
            }

            color_point_t point;
            point.xyz[0] = point_cloud_image_data[3 * i + 0];
            point.xyz[1] = point_cloud_image_data[3 * i + 1];
            point.xyz[2] = point_cloud_image_data[3 * i + 2];

            if (point.xyz[2] == 0)
            {
                continue;
            }

            point.rgb[0] = color_image_data[4 * i + 0];
            point.rgb[1] = color_image_data[4 * i + 1];
            point.rgb[2] = color_image_data[4 * i + 2];
            uint8_t alpha = color_image_data[4 * i + 3];

            if (point.rgb[0] == 0 && point.rgb[1] == 0 && point.rgb[2] == 0 && alpha == 0)
            {
                continue;
            }

            // 距离阈值判断
            float distance = std::sqrt(point.xyz[0] * point.xyz[0] + point.xyz[1] * point.xyz[1] + point.xyz[2] * point.xyz[2]);
            if (distance > data.distance_threshold)
            {
                continue;
            }

            auto matrix_start = std::chrono::high_resolution_clock::now();

            //if (data.matrix_file != "") {
            //    Eigen::Vector4f vec(point.xyz[0], point.xyz[1], point.xyz[2], 1.0f);
            //    Eigen::Vector4f transformed_vec = transformation_matrix * vec;
            //    point.xyz[0] = transformed_vec[0];
            //    point.xyz[1] = transformed_vec[1];
            //    point.xyz[2] = transformed_vec[2];
            //}

            //1.引入open3d、opencv；2.libtorch向量化操作
            auto matrix_end = std::chrono::high_resolution_clock::now();

            // 计算执行时间
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(matrix_end - matrix_start);

            current_points->push_back(point);
        }

        auto end_time = std::chrono::high_resolution_clock::now();

        // 计算执行时间
        auto duration0 = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

        // 打印执行时间（以毫秒为单位）
        std::cout << "一次for写入点时间：" << duration0.count() << " 毫秒" << std::endl;
   
        // lv
        //if (data.matrix_file != "") {
        //    // 读取变换矩阵
        //    Eigen::Matrix4f transformation_matrix;
        //    readMatrixFromFile(data.matrix_file, transformation_matrix);

        //    // 假设 points 是包含 color_point_t 结构的 std::vector
        //    for (auto& point : points) {
        //        // 创建一个4维向量，表示点的3D坐标
        //        Eigen::Vector4f vec(point.xyz[0], point.xyz[1], point.xyz[2], 1.0f);

        //        // 应用变换矩阵
        //        Eigen::Vector4f transformed_vec = transformation_matrix * vec;

        //        // 更新点的坐标
        //        point.xyz[0] = transformed_vec[0];
        //        point.xyz[1] = transformed_vec[1];
        //        point.xyz[2] = transformed_vec[2];
        //    }
        //}
    }

    const char* file_name = WritePCD[0].file_name;

    // 保存不花时间
    std::ofstream plyFile(file_name, std::ios::binary);

    if (!plyFile.is_open()) {
        std::cerr << "无法打开文件：" << file_name << std::endl;
        return;
    }

    // 计算所有points数组的大小总和，为空的points大小为0
    size_t total_points_size = points0.size() + points1.size() + points2.size();

    // 写入PLY文件头部
    plyFile << "ply\n";
    plyFile << "format binary_little_endian 1.0\n";
    plyFile << "element vertex " << total_points_size << "\n";
    plyFile << "property float x\n";
    plyFile << "property float y\n";
    plyFile << "property float z\n";
    plyFile << "property int blue\n";
    plyFile << "property int green\n";
    plyFile << "property int red\n";
    plyFile << "end_header\n";

    // 写入二进制点云数据
    plyFile.write(reinterpret_cast<const char*>(points0.data()), points0.size() * sizeof(color_point_t));
    plyFile.write(reinterpret_cast<const char*>(points1.data()), points1.size() * sizeof(color_point_t));
    plyFile.write(reinterpret_cast<const char*>(points2.data()), points2.size() * sizeof(color_point_t));

    plyFile.close();
    
    // 保存写到主函数逻辑里，每个相机一个points
}


// 不带传输，适用于点播，直接保存ply文件
//void tranformation_helpers_write_point_cloud(const k4a_image_t point_cloud_image,
//    const k4a_image_t color_image,
//    const char* file_name,
//    float distance_threshold,
//    float leaf_size,
//    const std::string& matrix_file)
//{
//    //auto start_time = std::chrono::high_resolution_clock::now();
//    std::vector<color_point_t> points;
//
//    int width = k4a_image_get_width_pixels(point_cloud_image);
//    int height = k4a_image_get_height_pixels(color_image);
//
//    int16_t* point_cloud_image_data = (int16_t*)(void*)k4a_image_get_buffer(point_cloud_image);
//    uint8_t* color_image_data = k4a_image_get_buffer(color_image);
//
//    printf("save fig");
//
//    // 初始化随机数生成器
//    std::random_device rd;  // 随机数种子
//    std::mt19937 gen(rd()); // 标准 mersenne_twister_engine
//    std::uniform_real_distribution<> dis(0.0, 1.0);
//
//    for (int i = 0; i < width * height; i++)
//    {
//        //if (i % 5 == 0)
//        //    continue;
//
//        if (dis(gen) > 0.8) // 只处理大约60%的点
//            continue;
//
//        color_point_t point;
//        point.xyz[0] = point_cloud_image_data[3 * i + 0];
//        point.xyz[1] = point_cloud_image_data[3 * i + 1];
//        point.xyz[2] = point_cloud_image_data[3 * i + 2];
//        if (point.xyz[2] == 0)
//        {
//            continue;
//        }
//
//        point.rgb[0] = color_image_data[4 * i + 0];
//        point.rgb[1] = color_image_data[4 * i + 1];
//        point.rgb[2] = color_image_data[4 * i + 2];
//        uint8_t alpha = color_image_data[4 * i + 3];
//
//        if (point.rgb[0] == 0 && point.rgb[1] == 0 && point.rgb[2] == 0 && alpha == 0)
//        {
//            continue;
//        }
//
//        // 距离阈值判断
//        float distance = std::sqrt(point.xyz[0] * point.xyz[0] + point.xyz[1] * point.xyz[1] + point.xyz[2] * point.xyz[2]);
//        if (distance > distance_threshold)
//        {
//            continue;
//        }
//
//        /*if (i == 300000)
//            break;*/
//        points.push_back(point);
//    }
//
//    // 降采样
//    // 转换为PCL点云格式
//    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//    //for (const auto& point : points) {
//    //    pcl::PointXYZRGB pcl_point;
//    //    pcl_point.x = point.xyz[0];
//    //    pcl_point.y = point.xyz[1];
//    //    pcl_point.z = point.xyz[2];
//    //    pcl_point.r = static_cast<uint8_t>(point.rgb[2]);
//    //    pcl_point.g = static_cast<uint8_t>(point.rgb[1]);
//    //    pcl_point.b = static_cast<uint8_t>(point.rgb[0]);
//    //    pcl_cloud->push_back(pcl_point);
//    //}
//
//    //auto start_time = std::chrono::high_resolution_clock::now();
//
//    //// 创建一个点云指针用于存储随机下采样的结果
//    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
//
//    //// 创建随机下采样滤波器对象
//    //pcl::RandomSample<pcl::PointXYZRGB> random_sampler;
//    //random_sampler.setInputCloud(pcl_cloud);     // 设置待滤波点云
//    //random_sampler.setSample(0.6 * width * height);  // 设置下采样点云的点数（例如200）
//    //// random_sampler.setSeed(1);                // （可选）获得相同的下采样结果
//
//    //// 应用随机下采样滤波
//    //random_sampler.filter(*cloud_filtered);
//
//    //// 记录结束时间点
//    //auto end_time = std::chrono::high_resolution_clock::now();      
//
//    //// 计算执行时间
//    //auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
//
//    //// 打印执行时间（以毫秒为单位）
//    //std::cout << "随机下采样时间：" << duration.count() << " 毫秒" << std::endl;
//
//
//    //// 应用体素网格滤波器进行降采样
//    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
//    //pcl::VoxelGrid<pcl::PointXYZRGB> sor;
//    //sor.setInputCloud(pcl_cloud);
//    //sor.setLeafSize(leaf_size, leaf_size, leaf_size);
//    //sor.filter(*cloud_filtered);
//
//    //// 记录结束时间点
//    //auto end_time = std::chrono::high_resolution_clock::now();
//
//    //// 计算执行时间
//    //auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
//
//    //// 打印执行时间（以毫秒为单位）
//    //std::cout << "Execution Time: " << duration.count() << " milliseconds" << std::endl;
//
//
//
//    if (matrix_file != "") {
//        auto start_time = std::chrono::high_resolution_clock::now();
//
//        printf("\n粗配准------\n");
//        // 读取变换矩阵
//        Eigen::Matrix4f transformation_matrix;
//        readMatrixFromFile(matrix_file, transformation_matrix);
//
//        // 假设 points 是包含 color_point_t 结构的 std::vector
//        for (auto& point : points) {
//            // 创建一个4维向量，表示点的3D坐标
//            Eigen::Vector4f vec(point.xyz[0], point.xyz[1], point.xyz[2], 1.0f);
//
//            // 应用变换矩阵
//            Eigen::Vector4f transformed_vec = transformation_matrix * vec;
//
//            // 更新点的坐标
//            point.xyz[0] = transformed_vec[0];
//            point.xyz[1] = transformed_vec[1];
//            point.xyz[2] = transformed_vec[2];
//        }
//
//
//        // 记录结束时间点
//        auto end_time = std::chrono::high_resolution_clock::now();
//
//        // 计算执行时间
//        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
//
//        // 打印执行时间（以毫秒为单位）
//        std::cout << "粗配准时间: " << duration.count() << "毫秒" << std::endl;
//
//        //std::cout << "Transformation Matrix:" << std::endl;
//        //for (int i = 0; i < transformation_matrix.rows(); ++i) {
//        //    for (int j = 0; j < transformation_matrix.cols(); ++j) {
//        //        std::cout << transformation_matrix(i, j);
//        //        if (j != transformation_matrix.cols() - 1)
//        //            std::cout << ", ";
//        //    }
//        //    std::cout << std::endl;
//        //}
//
//
//
//        // 对降采样后的点云应用变换
//        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//        //pcl::transformPointCloud(*cloud_filtered, *transformed_cloud, transformation_matrix);
//        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//    }
//
//    //// icp算法配准
//
//    //start_time = std::chrono::high_resolution_clock::now();
//
//    ////写入降采样后的PLY文件
//    //if (pcl::io::savePLYFileBinary(file_name, *cloud_filtered) == -1)
//    //{
//    //    PCL_ERROR("无法保存PLY文件\n");
//    //}
//
//    //// 记录结束时间点
//    //end_time = std::chrono::high_resolution_clock::now();
//
//    //// 计算执行时间
//    //duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
//
//    //// 打印执行时间（以毫秒为单位）
//    //std::cout << "随机下采样保存时间：" << duration.count() << " 毫秒" << std::endl;
//
//    auto start_time = std::chrono::high_resolution_clock::now();
//    std::cout << "开始存储" << std::endl;
//    std::ofstream plyFile(file_name, std::ios::binary);
//
//    if (!plyFile.is_open()) {
//        std::cerr << "无法打开文件：" << file_name << std::endl;
//        return;
//    }
//
//    // 写入PLY文件头部
//    plyFile << "ply\n";
//    plyFile << "format binary_little_endian 1.0\n";
//    plyFile << "element vertex " << points.size() << "\n";
//    plyFile << "property float x\n";
//    plyFile << "property float y\n";
//    plyFile << "property float z\n";
//    plyFile << "property int blue\n";
//    plyFile << "property int green\n";
//    plyFile << "property int red\n";
//    plyFile << "end_header\n";
//
//    // 写入二进制点云数据
//    plyFile.write(reinterpret_cast<const char*>(points.data()), points.size() * sizeof(color_point_t));
//
//    plyFile.close();
//
//    // 记录结束时间点
//    auto end_time = std::chrono::high_resolution_clock::now();
//
//    // 计算执行时间
//    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
//
//    // 打印执行时间（以毫秒为单位）
//    std::cout << "保存时间: " << duration.count() << "毫秒" << std::endl;
//}

k4a_image_t downscale_image_2x2_binning(const k4a_image_t color_image)
{
    int color_image_width_pixels = k4a_image_get_width_pixels(color_image);
    int color_image_height_pixels = k4a_image_get_height_pixels(color_image);
    int color_image_downscaled_width_pixels = color_image_width_pixels / 2;
    int color_image_downscaled_height_pixels = color_image_height_pixels / 2;
    k4a_image_t color_image_downscaled = NULL;
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
        color_image_downscaled_width_pixels,
        color_image_downscaled_height_pixels,
        color_image_downscaled_width_pixels * 4 * (int)sizeof(uint8_t),
        &color_image_downscaled))
    {
        printf("Failed to create downscaled color image\n");
        return color_image_downscaled;
    }

    uint8_t* color_image_data = k4a_image_get_buffer(color_image);
    uint8_t* color_image_downscaled_data = k4a_image_get_buffer(color_image_downscaled);
    for (int j = 0; j < color_image_downscaled_height_pixels; j++)
    {
        for (int i = 0; i < color_image_downscaled_width_pixels; i++)
        {
            int index_downscaled = j * color_image_downscaled_width_pixels + i;
            int index_tl = (j * 2 + 0) * color_image_width_pixels + i * 2 + 0;
            int index_tr = (j * 2 + 0) * color_image_width_pixels + i * 2 + 1;
            int index_bl = (j * 2 + 1) * color_image_width_pixels + i * 2 + 0;
            int index_br = (j * 2 + 1) * color_image_width_pixels + i * 2 + 1;

            color_image_downscaled_data[4 * index_downscaled + 0] = (uint8_t)(
                (color_image_data[4 * index_tl + 0] + color_image_data[4 * index_tr + 0] +
                    color_image_data[4 * index_bl + 0] + color_image_data[4 * index_br + 0]) /
                4.0f);
            color_image_downscaled_data[4 * index_downscaled + 1] = (uint8_t)(
                (color_image_data[4 * index_tl + 1] + color_image_data[4 * index_tr + 1] +
                    color_image_data[4 * index_bl + 1] + color_image_data[4 * index_br + 1]) /
                4.0f);
            color_image_downscaled_data[4 * index_downscaled + 2] = (uint8_t)(
                (color_image_data[4 * index_tl + 2] + color_image_data[4 * index_tr + 2] +
                    color_image_data[4 * index_bl + 2] + color_image_data[4 * index_br + 2]) /
                4.0f);
            color_image_downscaled_data[4 * index_downscaled + 3] = (uint8_t)(
                (color_image_data[4 * index_tl + 3] + color_image_data[4 * index_tr + 3] +
                    color_image_data[4 * index_bl + 3] + color_image_data[4 * index_br + 3]) /
                4.0f);
        }
    }

    return color_image_downscaled;
}
