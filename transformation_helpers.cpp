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


// ������
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <string>
#include <pcl/filters/random_sample.h>

#include <random>

// ���Ʊ任����ϵ
#include <Eigen/Dense>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>

// icp����׼
#include <pcl/registration/icp.h>


// ��ȡ�任����
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

        // ��ʼ�������������
        std::random_device rd;  // ���������
        std::mt19937 gen(rd()); // ��׼ mersenne_twister_engine
        std::uniform_real_distribution<> dis(0.0, 1.0);

        auto start_time = std::chrono::high_resolution_clock::now();

        Eigen::Matrix4f transformation_matrix;

        if (data.matrix_file != "") {
            // ��ȡ�任����
            readMatrixFromFile(data.matrix_file, transformation_matrix);
        }

        int a[2] = { 1 ,7 }; // ��Χ0��9
        int t = 0; // ��Χ0��2

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

            // ������ֵ�ж�
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

            //1.����open3d��opencv��2.libtorch����������
            auto matrix_end = std::chrono::high_resolution_clock::now();

            // ����ִ��ʱ��
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(matrix_end - matrix_start);

            current_points->push_back(point);
        }

        auto end_time = std::chrono::high_resolution_clock::now();

        // ����ִ��ʱ��
        auto duration0 = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

        // ��ӡִ��ʱ�䣨�Ժ���Ϊ��λ��
        std::cout << "һ��forд���ʱ�䣺" << duration0.count() << " ����" << std::endl;
   
        // lv
        //if (data.matrix_file != "") {
        //    // ��ȡ�任����
        //    Eigen::Matrix4f transformation_matrix;
        //    readMatrixFromFile(data.matrix_file, transformation_matrix);

        //    // ���� points �ǰ��� color_point_t �ṹ�� std::vector
        //    for (auto& point : points) {
        //        // ����һ��4ά��������ʾ���3D����
        //        Eigen::Vector4f vec(point.xyz[0], point.xyz[1], point.xyz[2], 1.0f);

        //        // Ӧ�ñ任����
        //        Eigen::Vector4f transformed_vec = transformation_matrix * vec;

        //        // ���µ������
        //        point.xyz[0] = transformed_vec[0];
        //        point.xyz[1] = transformed_vec[1];
        //        point.xyz[2] = transformed_vec[2];
        //    }
        //}
    }

    const char* file_name = WritePCD[0].file_name;

    // ���治��ʱ��
    std::ofstream plyFile(file_name, std::ios::binary);

    if (!plyFile.is_open()) {
        std::cerr << "�޷����ļ���" << file_name << std::endl;
        return;
    }

    // ��������points����Ĵ�С�ܺͣ�Ϊ�յ�points��СΪ0
    size_t total_points_size = points0.size() + points1.size() + points2.size();

    // д��PLY�ļ�ͷ��
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

    // д������Ƶ�������
    plyFile.write(reinterpret_cast<const char*>(points0.data()), points0.size() * sizeof(color_point_t));
    plyFile.write(reinterpret_cast<const char*>(points1.data()), points1.size() * sizeof(color_point_t));
    plyFile.write(reinterpret_cast<const char*>(points2.data()), points2.size() * sizeof(color_point_t));

    plyFile.close();
    
    // ����д���������߼��ÿ�����һ��points
}


// �������䣬�����ڵ㲥��ֱ�ӱ���ply�ļ�
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
//    // ��ʼ�������������
//    std::random_device rd;  // ���������
//    std::mt19937 gen(rd()); // ��׼ mersenne_twister_engine
//    std::uniform_real_distribution<> dis(0.0, 1.0);
//
//    for (int i = 0; i < width * height; i++)
//    {
//        //if (i % 5 == 0)
//        //    continue;
//
//        if (dis(gen) > 0.8) // ֻ�����Լ60%�ĵ�
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
//        // ������ֵ�ж�
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
//    // ������
//    // ת��ΪPCL���Ƹ�ʽ
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
//    //// ����һ������ָ�����ڴ洢����²����Ľ��
//    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
//
//    //// ��������²����˲�������
//    //pcl::RandomSample<pcl::PointXYZRGB> random_sampler;
//    //random_sampler.setInputCloud(pcl_cloud);     // ���ô��˲�����
//    //random_sampler.setSample(0.6 * width * height);  // �����²������Ƶĵ���������200��
//    //// random_sampler.setSeed(1);                // ����ѡ�������ͬ���²������
//
//    //// Ӧ������²����˲�
//    //random_sampler.filter(*cloud_filtered);
//
//    //// ��¼����ʱ���
//    //auto end_time = std::chrono::high_resolution_clock::now();      
//
//    //// ����ִ��ʱ��
//    //auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
//
//    //// ��ӡִ��ʱ�䣨�Ժ���Ϊ��λ��
//    //std::cout << "����²���ʱ�䣺" << duration.count() << " ����" << std::endl;
//
//
//    //// Ӧ�����������˲������н�����
//    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
//    //pcl::VoxelGrid<pcl::PointXYZRGB> sor;
//    //sor.setInputCloud(pcl_cloud);
//    //sor.setLeafSize(leaf_size, leaf_size, leaf_size);
//    //sor.filter(*cloud_filtered);
//
//    //// ��¼����ʱ���
//    //auto end_time = std::chrono::high_resolution_clock::now();
//
//    //// ����ִ��ʱ��
//    //auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
//
//    //// ��ӡִ��ʱ�䣨�Ժ���Ϊ��λ��
//    //std::cout << "Execution Time: " << duration.count() << " milliseconds" << std::endl;
//
//
//
//    if (matrix_file != "") {
//        auto start_time = std::chrono::high_resolution_clock::now();
//
//        printf("\n����׼------\n");
//        // ��ȡ�任����
//        Eigen::Matrix4f transformation_matrix;
//        readMatrixFromFile(matrix_file, transformation_matrix);
//
//        // ���� points �ǰ��� color_point_t �ṹ�� std::vector
//        for (auto& point : points) {
//            // ����һ��4ά��������ʾ���3D����
//            Eigen::Vector4f vec(point.xyz[0], point.xyz[1], point.xyz[2], 1.0f);
//
//            // Ӧ�ñ任����
//            Eigen::Vector4f transformed_vec = transformation_matrix * vec;
//
//            // ���µ������
//            point.xyz[0] = transformed_vec[0];
//            point.xyz[1] = transformed_vec[1];
//            point.xyz[2] = transformed_vec[2];
//        }
//
//
//        // ��¼����ʱ���
//        auto end_time = std::chrono::high_resolution_clock::now();
//
//        // ����ִ��ʱ��
//        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
//
//        // ��ӡִ��ʱ�䣨�Ժ���Ϊ��λ��
//        std::cout << "����׼ʱ��: " << duration.count() << "����" << std::endl;
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
//        // �Խ�������ĵ���Ӧ�ñ任
//        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//        //pcl::transformPointCloud(*cloud_filtered, *transformed_cloud, transformation_matrix);
//        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//    }
//
//    //// icp�㷨��׼
//
//    //start_time = std::chrono::high_resolution_clock::now();
//
//    ////д�뽵�������PLY�ļ�
//    //if (pcl::io::savePLYFileBinary(file_name, *cloud_filtered) == -1)
//    //{
//    //    PCL_ERROR("�޷�����PLY�ļ�\n");
//    //}
//
//    //// ��¼����ʱ���
//    //end_time = std::chrono::high_resolution_clock::now();
//
//    //// ����ִ��ʱ��
//    //duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
//
//    //// ��ӡִ��ʱ�䣨�Ժ���Ϊ��λ��
//    //std::cout << "����²�������ʱ�䣺" << duration.count() << " ����" << std::endl;
//
//    auto start_time = std::chrono::high_resolution_clock::now();
//    std::cout << "��ʼ�洢" << std::endl;
//    std::ofstream plyFile(file_name, std::ios::binary);
//
//    if (!plyFile.is_open()) {
//        std::cerr << "�޷����ļ���" << file_name << std::endl;
//        return;
//    }
//
//    // д��PLY�ļ�ͷ��
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
//    // д������Ƶ�������
//    plyFile.write(reinterpret_cast<const char*>(points.data()), points.size() * sizeof(color_point_t));
//
//    plyFile.close();
//
//    // ��¼����ʱ���
//    auto end_time = std::chrono::high_resolution_clock::now();
//
//    // ����ִ��ʱ��
//    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
//
//    // ��ӡִ��ʱ�䣨�Ժ���Ϊ��λ��
//    std::cout << "����ʱ��: " << duration.count() << "����" << std::endl;
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
