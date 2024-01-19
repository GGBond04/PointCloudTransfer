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

// �㲥ʱȡ��ע��
//struct color_point_t
//{
//    float xyz[3];
//    int rgb[3];
//};

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

void ICP(pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned_cloud) {
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setInputSource(source_cloud);
    icp.setInputTarget(target_cloud);

    // ����ICP�Ĳ����������������������任�����̶ȵ�
    icp.setMaximumIterations(50);
    icp.setMaxCorrespondenceDistance(0.05); // 5����
    icp.setTransformationEpsilon(1e-8);
    icp.setEuclideanFitnessEpsilon(1);

    // ����ICP�㷨���������
    icp.align(*aligned_cloud);

    if (icp.hasConverged()) {
        std::cout << "ICP ������" << std::endl;
        //std::cout << "�任����\n" << icp.getFinalTransformation() << std::endl;
    }
    else {
        PCL_ERROR("ICP δ����.\n");
    }
}

//const std::string ipAddr = "10.3.242.131";
//const int port = 8888;
//std::mutex mtx; // ����ͬ���Ļ�����
//std::chrono::duration<double, std::milli> sum;
//SocketClient client(ipAddr, port);
/*SocketClient client2(ipAddr, port);
SocketClient client3(ipAddr, port);*/
// �����䣬������ֱ��
//void tranformation_helpers_write_point_cloud(const k4a_image_t point_cloud_image,
//    const k4a_image_t color_image,
//    int i,
//    float distance_threshold,
//    float leaf_size,
//    const std::string& matrix_file)
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
//        //points.push_back(point);
//        image.points.push_back(point);
//    }
//
//
//
//    client.sendImage(image);
//
//}

// �������䣬�����ڵ㲥��ֱ�ӱ���ply�ļ�
void tranformation_helpers_write_point_cloud(const k4a_image_t point_cloud_image,
    const k4a_image_t color_image,
    const char* file_name,
    float distance_threshold,
    float leaf_size,
    const std::string& matrix_file)
{
    //auto start_time = std::chrono::high_resolution_clock::now();
    std::vector<color_point_t> points;

    int width = k4a_image_get_width_pixels(point_cloud_image);
    int height = k4a_image_get_height_pixels(color_image);

    int16_t* point_cloud_image_data = (int16_t*)(void*)k4a_image_get_buffer(point_cloud_image);
    uint8_t* color_image_data = k4a_image_get_buffer(color_image);

    printf("save fig\n");

    // ��ʼ�������������
    std::random_device rd;  // ���������
    std::mt19937 gen(rd()); // ��׼ mersenne_twister_engine
    std::uniform_real_distribution<> dis(0.0, 1.0);

    
    // ��������һ���洢ÿ�����ݵ� TensorList
    //torch::TensorList rows;
    //std::vector<torch::Tensor> rows;
    //int cnt_tensor = 0;
    auto start_time_1 = std::chrono::high_resolution_clock::now();//ת��ʱ��
    for (int i = 0; i < width * height; i++)
    {
        if (i % 5 == 0)
            continue;

        //torch::Tensor row_tensor = torch::empty({ 1, 6 }, torch::kFloat32);
        float point_data[6];
        if (dis(gen) > 0.8) // ֻ�����Լ60%�ĵ�
            continue;

        color_point_t point;
        point.xyz[0] = point_cloud_image_data[3 * i + 0];
        point.xyz[1] = point_cloud_image_data[3 * i + 1];
        point.xyz[2] = point_cloud_image_data[3 * i + 2];

       /* point_data[0]= point_cloud_image_data[3 * i + 0];
        point_data[1] = point_cloud_image_data[3 * i + 1];
        point_data[2] = point_cloud_image_data[3 * i + 2];*/
        if (point.xyz[2] == 0)
        {
            continue;
        }

        point.rgb[0] = color_image_data[4 * i + 0];
        point.rgb[1] = color_image_data[4 * i + 1];
        point.rgb[2] = color_image_data[4 * i + 2];

        /*point_data[3] = color_image_data[4 * i + 0];
        point_data[4] = color_image_data[4 * i + 1];
        point_data[5] = color_image_data[4 * i + 2];*/
        uint8_t alpha = color_image_data[4 * i + 3];

        if (point.rgb[0] == 0 && point.rgb[1] == 0 && point.rgb[2] == 0 && alpha == 0)
        {
            continue;
        }

        // ������ֵ�ж�
        float distance = std::sqrt(point.xyz[0] * point.xyz[0] + point.xyz[1] * point.xyz[1] + point.xyz[2] * point.xyz[2]);
        if (distance > distance_threshold)
        {
            continue;
        }

        /*if (i == 300000)
            break;*/
        points.push_back(point);
        //torch::Tensor row_tensor = torch::from_blob(point_data, { 1, 6 }, torch::kFloat32).clone();
        /*row_tensor[0] = (float)point.xyz[0];
        row_tensor[1] = (float)point.xyz[1];
        row_tensor[2] = (float)point.xyz[2];
        row_tensor[3] = point.rgb[0];
        row_tensor[4] = point.rgb[1];
        row_tensor[5] = point.rgb[2];*/
        //rows.push_back(row_tensor);
        //std::cout << cnt_tensor << std::endl;
        //cnt_tensor++;
    }
    std::cout << "points2tensor begin" << std::endl;
    //torch::Tensor matrix_tensor = torch::stack(rows);
    
    /*torch::Tensor matrix_tensor = torch::empty({ static_cast<long>(points.size()), 6 }, torch::kFloat32);
    for (int i = 0;i < points.size();i++)
    {
        matrix_tensor[i][0] = points[i].xyz[0];
        matrix_tensor[i][1] = points[i].xyz[1];
        matrix_tensor[i][2] = points[i].xyz[2];

        matrix_tensor[i][3] = static_cast<float>(points[i].rgb[0]);
        matrix_tensor[i][4] = static_cast<float>(points[i].rgb[1]);
        matrix_tensor[i][5] = static_cast<float>(points[i].rgb[2]);
    }*/
    // ����һ��������ֱ�Ӵ� points �е������л�ȡ
    torch::Tensor matrix_tensor = torch::from_blob(points.data(), { static_cast<long>(points.size()), 6 }, torch::kFloat32);

    std::cout << "points2tensor done" << std::endl;

    //for (int i = 0; i < std::min(5, static_cast<int>(points.size())); ++i) {
    //    std::cout << "Row " << i << ": ";
    //    for (int j = 0; j < 3; ++j) {
    //        std::cout << matrix_tensor[i][j].item<float>() << " ";
    //    }
    //    for (int j = 3; j < 6; ++j) {
    //        std::cout << matrix_tensor[i][j].item<int>() << " ";
    //    }
    //    std::cout << "\n";
    //}

    auto start_time_2 = std::chrono::high_resolution_clock::now();  //�˷�ʱ��
    //ʹ��libtorch���о���˷�����
    if (matrix_file != "") {
        torch::Tensor other_matrix = torch::randn({ 4, 4 });

        // ��ȡǰ����
        torch::Tensor sub_matrix = matrix_tensor.index({ torch::indexing::Slice(torch::indexing::None, torch::indexing::None), torch::indexing::Slice(torch::indexing::None, 4) });

        // ʹ�� mm �������о������
        torch::Tensor result = torch::mm(sub_matrix, other_matrix);

        // ����ԭʼ������ǰ���У������µ����У�
        matrix_tensor.index_put_({ torch::indexing::Slice(torch::indexing::None, torch::indexing::None), torch::indexing::Slice(torch::indexing::None, 3) }, result.index({ torch::indexing::Slice(torch::indexing::None, torch::indexing::None), torch::indexing::Slice(torch::indexing::None, 3) }));

        auto start_time_3 = std::chrono::high_resolution_clock::now();  //д��ʱ��
        
                                                                        // �򿪶������ļ�
        std::ofstream ply_file(file_name, std::ios::binary);

        // д�� PLY �ļ�ͷ
        ply_file << "ply\n";
        ply_file << "format binary_little_endian 1.0\n";
        ply_file << "element vertex " << matrix_tensor.size(0) << "\n";
        ply_file << "property float x\n";
        ply_file << "property float y\n";
        ply_file << "property float z\n";
        ply_file << "property uchar red\n";
        ply_file << "property uchar green\n";
        ply_file << "property uchar blue\n";
        ply_file << "end_header\n";

         //д��������ݣ������Ƹ�ʽ��
        auto data_accessor = matrix_tensor.accessor<float, 2>();
        for (int i = 0; i < matrix_tensor.size(0); ++i) {
            for (int j = 0; j < 3; ++j) {
                float value = data_accessor[i][j];
                ply_file.write(reinterpret_cast<char*>(&value), sizeof(float));
            }
            for (int j = 3; j < 6; ++j) {
                uint8_t value = static_cast<uint8_t>(data_accessor[i][j]);
                ply_file.write(reinterpret_cast<char*>(&value), sizeof(uint8_t));
            }
        }

        //// ��ȡ����ǰ���е� float ��ͼ
        //auto float_view = matrix_tensor.narrow(1, 0, 3).to(torch::kFloat32);

        //// ֱ��д�� float ����
        //ply_file.write(reinterpret_cast<const char*>(float_view.data_ptr<float>()), float_view.numel() * sizeof(float));

        //// ��ȡ���������е� int ��ͼ
        //auto int_view = matrix_tensor.narrow(1, 3, 3).to(torch::kInt32);

        //// ֱ��д�� int ����
        //ply_file.write(reinterpret_cast<const char*>(int_view.data_ptr<int>()), int_view.numel() * sizeof(int));

        //�ر��ļ�
        ply_file.close();

        // ��¼����ʱ���
        auto end_time = std::chrono::high_resolution_clock::now();

        // ����ִ��ʱ��
        auto duration_1 = std::chrono::duration_cast<std::chrono::milliseconds>(start_time_2 - start_time_1);
        std::cout << "image2tensorʱ��: " << duration_1.count() << "����" << std::endl;
        auto duration_2 = std::chrono::duration_cast<std::chrono::milliseconds>(start_time_3 - start_time_2);
        std::cout << "����˷�ʱ��: " << duration_2.count() << "����" << std::endl;
        auto duration_3 = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time_3);
        // ��ӡִ��ʱ�䣨�Ժ���Ϊ��λ��
        std::cout << "д��plyʱ��: " << duration_3.count() << "����" << std::endl;
    }
    std::cout << "�ļ�д�����" << std::endl;

    //if (matrix_file != "") {
    //    auto start_time = std::chrono::high_resolution_clock::now();

    //    printf("\n����׼------\n");
    //    // ��ȡ�任����
    //    Eigen::Matrix4f transformation_matrix;
    //    readMatrixFromFile(matrix_file, transformation_matrix);

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


        

        //std::cout << "Transformation Matrix:" << std::endl;
        //for (int i = 0; i < transformation_matrix.rows(); ++i) {
        //    for (int j = 0; j < transformation_matrix.cols(); ++j) {
        //        std::cout << transformation_matrix(i, j);
        //        if (j != transformation_matrix.cols() - 1)
        //            std::cout << ", ";
        //    }
        //    std::cout << std::endl;
        //}



        // �Խ�������ĵ���Ӧ�ñ任
        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        //pcl::transformPointCloud(*cloud_filtered, *transformed_cloud, transformation_matrix);
        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    //}

    //// icp�㷨��׼

    //start_time = std::chrono::high_resolution_clock::now();

    ////д�뽵�������PLY�ļ�
    //if (pcl::io::savePLYFileBinary(file_name, *cloud_filtered) == -1)
    //{
    //    PCL_ERROR("�޷�����PLY�ļ�\n");
    //}

    //// ��¼����ʱ���
    //end_time = std::chrono::high_resolution_clock::now();

    //// ����ִ��ʱ��
    //duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    //// ��ӡִ��ʱ�䣨�Ժ���Ϊ��λ��
    //std::cout << "����²�������ʱ�䣺" << duration.count() << " ����" << std::endl;

    //auto start_time = std::chrono::high_resolution_clock::now();
    //std::cout << "��ʼ�洢" << std::endl;
    //std::ofstream plyFile(file_name, std::ios::binary);

    //if (!plyFile.is_open()) {
    //    std::cerr << "�޷����ļ���" << file_name << std::endl;
    //    return;
    //}

    //// д��PLY�ļ�ͷ��
    //plyFile << "ply\n";
    //plyFile << "format binary_little_endian 1.0\n";
    //plyFile << "element vertex " << points.size() << "\n";
    //plyFile << "property float x\n";
    //plyFile << "property float y\n";
    //plyFile << "property float z\n";
    //plyFile << "property int blue\n";
    //plyFile << "property int green\n";
    //plyFile << "property int red\n";
    //plyFile << "end_header\n";

    //// д������Ƶ�������
    //plyFile.write(reinterpret_cast<const char*>(points.data()), points.size() * sizeof(color_point_t));

    //plyFile.close();

    //// ��¼����ʱ���
    //auto end_time = std::chrono::high_resolution_clock::now();

    //// ����ִ��ʱ��
    //auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    //// ��ӡִ��ʱ�䣨�Ժ���Ϊ��λ��
    //std::cout << "����ʱ��: " << duration.count() << "����" << std::endl;
}

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
