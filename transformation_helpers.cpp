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

// 点云变换坐标系
#include <Eigen/Dense>

#include <Python.h>


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

//点播
void tranformation_helpers_write_point_cloud(std::vector<WritePointCloudData>& WritePCD,
    const char* file_name, float distance_threshold, PyObject* pModule)
{
    std::vector<color_point_t> points0, points1, points2;
    std::vector<int> originalRGB1, originalRGB2;

    Eigen::Matrix4f transformation_matrix;

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

        auto start_time = std::chrono::high_resolution_clock::now();

        //if (data.matrix_file != "") {
        //    // 读取变换矩阵
        //    readMatrixFromFile(data.matrix_file, transformation_matrix);
        //}

        int a[3] = { 1, 4, 7 }; // 范围0到9
        int t = 0; // 范围0到2

        for (int i = 0; i < width * height; i++)
        {
            // 去除30%的点
            if (i % 10 == a[t]) {
                a[t] = (a[t] + 1) % 10;
                t = (t + 1) % 3;
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

            //if (index == 1) {
            //    //originalRGB1.push_back(color_image_data[4 * i + 0]);
            //    //originalRGB1.push_back(color_image_data[4 * i + 1]);
            //    //originalRGB1.push_back(color_image_data[4 * i + 2]);

            //    //point.rgb[0] = 1;
            //}

            point.rgb[0] = color_image_data[4 * i + 0];
            point.rgb[1] = color_image_data[4 * i + 1];
            point.rgb[2] = color_image_data[4 * i + 2];
            uint8_t alpha = color_image_data[4 * i + 3];

            //if (index == 1) {
            //    originalRGB1.push_back(point.rgb[0]);
            //    originalRGB1.push_back(point.rgb[1]);
            //    originalRGB1.push_back(point.rgb[2]);
            //}
            //else if (index == 2) {
            //    originalRGB2.push_back(point.rgb[0]);
            //    originalRGB2.push_back(point.rgb[1]);
            //    originalRGB2.push_back(point.rgb[2]);
            //}


            if (point.rgb[0] == 0 && point.rgb[1] == 0 && point.rgb[2] == 0 && alpha == 0)
            {
                continue;
            }

            // 距离阈值判断
            float distance = std::sqrt(point.xyz[0] * point.xyz[0] + point.xyz[1] * point.xyz[1] + point.xyz[2] * point.xyz[2]);
            if (distance > distance_threshold)
            {
                continue;
            }

            auto matrix_start = std::chrono::high_resolution_clock::now();

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
    }

    //auto start_time = std::chrono::high_resolution_clock::now();

    //// 20到30毫秒
    //for (const auto& point : points1) {
    //    originalRGB1.push_back(point.rgb[0]);
    //    originalRGB1.push_back(point.rgb[1]);
    //    originalRGB1.push_back(point.rgb[2]);
    //}

    // 0毫秒 建立映射
    //Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>> xyzMatrix(
    //    reinterpret_cast<float*>(points1.data()), points1.size(), 3);
    // 在xyzMatrix的最后一列添加一列全为1的列，以便与4x4矩阵相乘
    //// 0毫秒
    //Eigen::MatrixXf augmentedMatrix(points1.size(), 4);
    //// 30毫秒左右
    //augmentedMatrix << xyzMatrix, Eigen::MatrixXf::Ones(points1.size(), 1);
    //// 35毫秒左右
    //Eigen::MatrixXf resultMatrix = augmentedMatrix * transformation_matrix.transpose();

    // 希望不修改rgb信息
    // 创建一个临时矩阵来存储 XYZ 数据和额外的一列
    //Eigen::MatrixXf tempMatrix(points1.size(), 4);
    //tempMatrix.leftCols<3>() = xyzMatrix; // 复制 XYZ 数据
    //tempMatrix.col(3).setOnes(); // 在第4列添加1
    //// 现在使用 tempMatrix 来进行变换
    //Eigen::MatrixXf resultMatrix = tempMatrix * transformation_matrix.transpose();

    //// 计算 color_point_t 结构的总大小（以float为单位）
    //constexpr int stride = sizeof(color_point_t) / sizeof(float);

    //// 创建一个Eigen Map，映射XYZ数据和一个额外的1列
    //Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, 4, Eigen::RowMajor>, 0, Eigen::OuterStride<>>
    //    mappedMatrix(reinterpret_cast<float*>(points1.data()), points1.size(), 4, Eigen::OuterStride<>(stride));

    //// 设置第四列为1
    //mappedMatrix.col(3).setOnes();
    //Eigen::MatrixXf resultMatrix = mappedMatrix * transformation_matrix.transpose();

    //Eigen::MatrixXf augmentedMatrix = Eigen::MatrixXf::Zero(points1.size(), 4);
    //augmentedMatrix.block(0, 0, points1.size(), 3) = xyzMatrix;
    //augmentedMatrix.col(3).setOnes();
    //Eigen::MatrixXf resultMatrix = augmentedMatrix * transformation_matrix.transpose();

    // 将结果覆盖到points0的前三列
    // 20毫秒左右
    //Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>>(xyzMatrix.data(), points1.size(), 3)
    //    = resultMatrix.leftCols<3>();

    //// 2到3毫秒
    //for (size_t i = 0; i < points1.size(); ++i) {
    //    points1[i].rgb[0] = originalRGB1[i * 3];
    //    points1[i].rgb[1] = originalRGB1[i * 3 + 1];
    //    points1[i].rgb[2] = originalRGB1[i * 3 + 2];
    //}

    //auto end_time = std::chrono::high_resolution_clock::now();

    //// 计算执行时间
    //auto duration0 = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    //// 打印执行时间（以毫秒为单位）
    //std::cout << "一次粗配准时间：" << duration0.count() << " 毫秒" << std::endl;

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



    //// 初始化Python解释器
    //Py_Initialize();

    //// 修改sys.path
    //PyObject* sysPath = PySys_GetObject("path");
    //PyList_Append(sysPath, PyUnicode_FromString("D:/Code/python"));

    //// 加载Python脚本
    //PyObject* pName = PyUnicode_DecodeFSDefault("relative_trans");

    //auto start_time = std::chrono::high_resolution_clock::now();

    //PyObject* pModule = PyImport_Import(pName);

    auto start_time = std::chrono::high_resolution_clock::now();

    if (pModule != nullptr) {
        // 获取函数引用 0ms
        PyObject* pFunc = PyObject_GetAttrString(pModule, "yytyyds");
        // 检查函数有效性并调用
        if (pFunc && PyCallable_Check(pFunc)) {
            // 0ms
            PyObject* pArgs = PyTuple_New(0); // 函数没有参数
            // 40ms
            PyObject* pValue = PyObject_CallObject(pFunc, pArgs);
            // 0ms
            Py_DECREF(pArgs);
            // 处理返回值（本例中函数没有返回值）
            if (pValue != nullptr) {
                // 0ms
                Py_DECREF(pValue);
            }
            else {
                PyErr_Print();
            }
            // 0ms
            Py_XDECREF(pFunc);
        }
        else {
            if (PyErr_Occurred())
                PyErr_Print();
            fprintf(stderr, "Cannot find function \"yytyyds\"\n");
        }
        // 0ms
        Py_DECREF(pModule);
    }
    else {
        PyErr_Print();
        fprintf(stderr, "Failed to load \"relative_trans\"\n");
    }

    // 165ms
    // 清理Python环境
    // Py_Finalize();

    auto end_time = std::chrono::high_resolution_clock::now();

    // 计算执行时间
    auto duration0 = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    // 打印执行时间（以毫秒为单位）
    std::cout << "一次python时间：" << duration0.count() << " 毫秒" << std::endl;


}


//// 直播
//const std::string ipAddr = "10.3.242.131";
//const int port = 8888;
//std::mutex mtx; // 用于同步的互斥锁
//std::chrono::duration<double, std::milli> sum;
//SocketClient client(ipAddr, port);
//// 带传输，适用于直播
//void tranformation_helpers_write_point_cloud(const k4a_image_t point_cloud_image,
//    const k4a_image_t color_image,
//    int i)
//{
//
//    kinect_image image;
//    image.id_camera = i;
//    image.id_order = i;
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
//    for (int i = 0; i < width * height; i++)
//    {
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
//        /*if (i == 400000)
//            break;*/
//            //points.push_back(point);
//        image.points.push_back(point);
//    }
//
//
//
//    client.sendImage(image);
//
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
