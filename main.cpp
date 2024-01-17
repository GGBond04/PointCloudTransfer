// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
#include <algorithm>
#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <limits>
#include <iostream>
#include <fstream>

#include <k4a/k4a.hpp>
#include <time.h>

using std::cerr;
using std::cout;
using std::endl;
using std::vector;

#include "transformation.h"
#include "MultiDeviceCapturer.h"
#include "transformation_helpers.h"

// Allowing at least 160 microseconds between depth cameras should ensure they do not interfere with one another.
constexpr uint32_t MIN_TIME_BETWEEN_DEPTH_CAMERA_PICTURES_USEC = 160;

static k4a_device_configuration_t get_master_config();
static k4a_device_configuration_t get_subordinate_config();

// �ϲ����溯��
struct PointCloudData {
    k4a_transformation_t transformation_handle;
    k4a::image depth_image;
    k4a::image color_image;
    std::string file_name;
    std::string matrix_file;
    int count;
};

void save_point_cloud_new(std::vector<PointCloudData>& point_clouds);

//void save_point_cloud_new(k4a_transformation_t transformation_handle,
//    k4a::image depth_image,
//    k4a::image color_image,
//    //int count,
//    //std::string matrix);
////�����ǵ㲥
//    std::string file_name,
//    std::string matrix);

k4a_transformation_t transformation_handle_main = NULL;
k4a_transformation_t transformation_handle_second = NULL;
k4a_transformation_t transformation_handle_second_1 = NULL;
k4a_transformation_t transformation_handle_second_2 = NULL;

int main(int argc, char** argv)
{
    float chessboard_square_length = 30; // ÿ�����̸�ı߳�����Ҫ��Ϊ��������ṩ
    int32_t color_exposure_usec = 8000;  // ��ɫͼ���ع�ʱ�䣬Ĭ��Ϊ8000΢��
    int32_t powerline_freq = 2;          // ԴƵ�ʣ�Ĭ��Ϊ60Hz
    cv::Size chessboard_pattern(8, 13);  // ���̸�ĳߴ磬�߶ȺͿ�ȶ���Ҫ����
    uint16_t depth_threshold = 1000;     // �����ֵ��Ĭ��Ϊ1��
    size_t num_devices = 2;              // �豸������Ĭ��Ϊ2̨
    double calibration_timeout = 120.0;  // У׼��ʱʱ�䣬Ĭ��Ϊ120�루2���ӣ�
    double greenscreen_duration = std::numeric_limits<double>::max(); // ��Ļ����ʱ�䣬Ĭ��Ϊ���޴�һֱ����

    vector<uint32_t> device_indices{ 0 }; // ����һ��MultiDeviceCapturer��ͬʱ�����̨�����ͼ��
    // ע��device_indices��������˳��һ��������
    // ��ΪMultiDeviceCapturer�᳢�Ը��ݲ���ͬ���ߵ��豸��ȷ�����豸��
    // ��ʼʱֻ����0�������Ҫ�������������豸����

    if (argc > 5)
    {
        cout << "Usage: green_screen <num-cameras> <board-height> <board-width> <board-square-length> "
            "[depth-threshold-mm (default 1000)] [color-exposure-time-usec (default 8000)] "
            "[powerline-frequency-mode (default 2 for 60 Hz)] [calibration-timeout-sec (default 60)]"
            "[greenscreen-duration-sec (default infinity- run forever)]"
            << endl;

        cerr << "Not enough arguments!\n";
        exit(1);
    }
    else
    {
        //num_devices = static_cast<size_t>(atoi(argv[1]));
        if (num_devices > k4a::device::get_installed_count())
        {
            cerr << "Not enough cameras plugged in!\n";
            exit(1);
        }

        if (argc > 5)
        {
            depth_threshold = static_cast<uint16_t>(atoi(argv[5]));
            if (argc > 6)
            {
                color_exposure_usec = atoi(argv[6]);
                if (argc > 7)
                {
                    powerline_freq = atoi(argv[7]);
                    if (argc > 8)
                    {
                        calibration_timeout = atof(argv[8]);
                        if (argc > 9)
                        {
                            greenscreen_duration = atof(argv[9]);
                        }
                    }
                }
            }
        }
    }

    // ����豸�����Ƿ�Ϊ1��2��������������������Ϣ���˳���������豸����Ϊ2�����豸������ӵ�device_indices�С�
    if (num_devices != 3 && num_devices != 2 && num_devices != 1)
    {
        cerr << "Invalid choice for number of devices!\n";
        exit(1);
    }
    else if (num_devices == 2)
    {
        device_indices.emplace_back(1); // �����豸����Ϊ { 0, 1 }
    }
    else if (num_devices == 3) {
        device_indices.emplace_back(1);
        device_indices.emplace_back(2);
    }// �޸Ĳ���

    // ʹ��device_indices��color_exposure_usec��powerline_freq��������MultiDeviceCapturer����capturer��
    MultiDeviceCapturer capturer(device_indices, color_exposure_usec, powerline_freq);

    // ��ȡ���豸��������Ϣ����ͬ��ģʽ����Ϊ����ģʽ����ȡ�����豸��������Ϣ��
    k4a_device_configuration_t main_config = get_master_config();
    if (num_devices == 1) // no need to have a master cable if it's standalone
    {
        main_config.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
    }
    k4a_device_configuration_t secondary_config = get_subordinate_config();

    // ��ȡ���豸��У׼��Ϣ��
    k4a::calibration main_calibration = capturer.get_master_device().get_calibration(main_config.depth_mode,
        main_config.color_resolution);

    // ʹ�����豸��У׼��Ϣ����һ��ת����������������ɫ����ռ���ת�������ͼ��
    k4a::transformation main_depth_to_main_color(main_calibration);
    std::cout << "I am here" << std::endl;
    // ʹ�����豸�ʹ����豸��������Ϣ�����豸��
    capturer.start_devices(main_config, secondary_config);
    
    if (num_devices == 1) {
        //This wraps all the device-to-device details
        //���豸֮���ϸ�ڽ��з�װ
        std::cout << "I am here" << std::endl;
        std::chrono::time_point<std::chrono::system_clock> start_time = std::chrono::system_clock::now();
        transformation_handle_main = k4a_transformation_create(&main_calibration);
        int count = 0;
        // ��һ��ʱ���ڽ��д���
        while (std::chrono::duration<double>(std::chrono::system_clock::now() - start_time).count() <
            greenscreen_duration)
        {

            // pointcloud ������������ļ���Ŀ¼
            std::string outputDirectory0 = "E:\\code\\kinect\\greenScreen\\ply1";

            // ��ȡͬ���Ĳ���֡
            vector<k4a::capture> captures;
            captures = capturer.get_synchronized_captures(secondary_config, true);

            // ��ȡ����ɫͼ��������ͼ��
            k4a::image main_color_image = captures[0].get_color_image();
            k4a::image main_depth_image = captures[0].get_depth_image();

            // ��������ļ�
            std::string filename = outputDirectory0 + "\\" + std::to_string(count) + ".ply";
            auto time_1 = std::chrono::high_resolution_clock::now();

            std::vector<PointCloudData> point_clouds;

            // ��� point_clouds ����
            point_clouds.push_back({ transformation_handle_main, main_depth_image, main_color_image, filename, ""});

            // �㲥
            save_point_cloud_new(point_clouds);

            //ֱ��
            //save_point_cloud_new(transformation_handle_main, main_depth_image, main_color_image, count);
            auto time_2 = std::chrono::high_resolution_clock::now();

            auto duration_1 = std::chrono::duration_cast<std::chrono::microseconds>(time_2 - time_1);

            std::cout << "��һ�α���ʹ�õ�����" << duration_1.count() * 1.0 / 1000000 << endl;
            count++;
            if (count == 9)
                break;
        }
    }
    else if (num_devices == 2)
    {
        //This wraps all the device-to-device details
        //���豸֮���ϸ�ڽ��з�װ
        
        // ��ȡ�μ��豸��У׼����
        k4a::calibration secondary_calibration =
            capturer.get_subordinate_device_by_index(0).get_calibration(secondary_config.depth_mode,
                secondary_config.color_resolution);

        transformation_handle_main = k4a_transformation_create(&main_calibration);
        transformation_handle_second = k4a_transformation_create(&secondary_calibration);

        std::chrono::time_point<std::chrono::system_clock> start_time = std::chrono::system_clock::now();

        int count = 0;
        // ��һ��ʱ���ڽ��д���
        while (std::chrono::duration<double>(std::chrono::system_clock::now() - start_time).count() <
            greenscreen_duration)
        {
            // pointcloud ������������ļ���Ŀ¼
            std::string outputDirectory0 = "pointcloud/capture";

            // ��ȡͬ���Ĳ���֡
            vector<k4a::capture> captures = capturer.get_synchronized_captures(secondary_config, true);

            // ��ȡ����ɫͼ��������ͼ��
            k4a::image main_color_image = captures[0].get_color_image();
            k4a::image main_depth_image = captures[0].get_depth_image();

            // ��ȡ�μ����ͼ�� �ʹμ���ɫͼ��
            k4a::image secondary_depth_image = captures[1].get_depth_image();
            k4a::image secondary_color_image = captures[1].get_color_image();

            // ��������ļ�
            std::string filename0 = outputDirectory0 + "\\" + std::to_string(count) + ".ply";

            // ��׼����
            std::string matrix1 = "D:\\Code\\kinect\\pointcloud\\capture2\\2to0_matrix.txt";

            // �ϲ�����
            std::vector<PointCloudData> point_clouds;

            // push back����ʱ��
            // ��� point_clouds ����
            point_clouds.push_back({ transformation_handle_main, main_depth_image, main_color_image, filename0, "" });
            point_clouds.push_back({ transformation_handle_second, secondary_depth_image, secondary_color_image, filename0, matrix1 });
           
            auto start_time = std::chrono::high_resolution_clock::now();

            save_point_cloud_new(point_clouds);

            // ��¼����ʱ���
            auto end_time = std::chrono::high_resolution_clock::now();      
    
            // ����ִ��ʱ��
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
            // ��ӡִ��ʱ�䣨�Ժ���Ϊ��λ��
            std::cout << "һ��save����ʱ�䣺" << duration.count() << " ����" << std::endl;
            count++;

        }
    }
    else if (num_devices == 3)
    {
        int count = 0;
        // ��ȡ�����豸��У׼����
        k4a::calibration secondary_calibration_1 =
            capturer.get_subordinate_device_by_index(0).get_calibration(secondary_config.depth_mode,
                secondary_config.color_resolution);

        k4a::calibration secondary_calibration_2 =
            capturer.get_subordinate_device_by_index(1).get_calibration(secondary_config.depth_mode,
                secondary_config.color_resolution);

        transformation_handle_main = k4a_transformation_create(&main_calibration);
        transformation_handle_second_1 = k4a_transformation_create(&secondary_calibration_1);
        transformation_handle_second_2 = k4a_transformation_create(&secondary_calibration_2);

        std::chrono::time_point<std::chrono::system_clock> start_time = std::chrono::system_clock::now();

        // ��һ��ʱ���ڽ��д���
        while (std::chrono::duration<double>(std::chrono::system_clock::now() - start_time).count() <
            greenscreen_duration)
        {
            time_t start = clock();    //��ȡ��ʼ��ʱ���

            // ��������ļ������·��  
            std::string outputDirectory0 = "pointcloud/capture0";
            std::string outputDirectory1 = "pointcloud/capture1";
            std::string outputDirectory2 = "pointcloud/capture2";

            // ��ȡͬ���Ĳ���֡
            vector<k4a::capture> captures;
            captures = capturer.get_synchronized_captures(secondary_config, true);

            // ��ȡ����ɫͼ��������ͼ��
            k4a::image main_color_image = captures[0].get_color_image();
            k4a::image main_depth_image = captures[0].get_depth_image();

            // ��ȡ����1���ͼ��ʹ���1��ɫͼ��
            k4a::image secondary_depth_image_1 = captures[1].get_depth_image();
            k4a::image secondary_color_image_1 = captures[1].get_color_image();

            // ��ȡ����2���ͼ��ʹ���2��ɫͼ��
            k4a::image secondary_depth_image_2 = captures[2].get_depth_image();
            k4a::image secondary_color_image_2 = captures[2].get_color_image();

            // ��������ļ�
            std::string filename0 = outputDirectory0 + "\\" + std::to_string(count) + ".ply";
            std::string filename1 = outputDirectory1 + "\\" + std::to_string(count) + ".ply";
            std::string filename2 = outputDirectory2 + "\\" + std::to_string(count) + ".ply";
            
            // ��׼����
            std::string matrix1 = "D:\\Code\\kinect\\pointcloud\\capture1\\1to0_matrix.txt";
            std::string matrix2 = "D:\\Code\\kinect\\pointcloud\\capture2\\2to0_matrix.txt";

            std::vector<PointCloudData> point_clouds;

            // ��� point_clouds ����
            point_clouds.push_back({ transformation_handle_main, main_depth_image, main_color_image, filename0, "" });
            point_clouds.push_back({ transformation_handle_main, main_depth_image, main_color_image, filename0, matrix1 });
            point_clouds.push_back({ transformation_handle_main, main_depth_image, main_color_image, filename0, matrix2 });

            save_point_cloud_new(point_clouds); //���ַ���
            count++;
            //break;


            time_t end = clock();     //��ȡ������ʱ��� 
            time_t run_time = end - start;  //����ʱ��=������ʱ���-��ȥ-��ʼ��ʱ���  
            cout << run_time << "ms" << endl;   //�������ʱ�䣬��λ�Ǻ���ms,  1��=1000���� 


        }
    }
    else
    {
        cerr << "Invalid number of devices!" << endl;
        exit(1);
    }
    return 0;
}

static k4a_device_configuration_t get_default_config()
{
    k4a_device_configuration_t camera_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    camera_config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    camera_config.color_resolution = K4A_COLOR_RESOLUTION_720P;
    camera_config.depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED; // No need for depth during calibration
    camera_config.camera_fps = K4A_FRAMES_PER_SECOND_15;     // Don't use all USB bandwidth
    camera_config.subordinate_delay_off_master_usec = 0;     // Must be zero for master
    camera_config.synchronized_images_only = true;
    return camera_config;
}

// Master customizable settings
static k4a_device_configuration_t get_master_config()
{
    k4a_device_configuration_t camera_config = get_default_config();
    camera_config.wired_sync_mode = K4A_WIRED_SYNC_MODE_MASTER;

    // Two depth images should be seperated by MIN_TIME_BETWEEN_DEPTH_CAMERA_PICTURES_USEC to ensure the depth imaging
    // sensor doesn't interfere with the other. To accomplish this the master depth image captures
    // (MIN_TIME_BETWEEN_DEPTH_CAMERA_PICTURES_USEC / 2) before the color image, and the subordinate camera captures its
    // depth image (MIN_TIME_BETWEEN_DEPTH_CAMERA_PICTURES_USEC / 2) after the color image. This gives us two depth
    // images centered around the color image as closely as possible.
    camera_config.depth_delay_off_color_usec = -static_cast<int32_t>(MIN_TIME_BETWEEN_DEPTH_CAMERA_PICTURES_USEC / 2);
    camera_config.synchronized_images_only = true;
    return camera_config;
}

// Subordinate customizable settings
static k4a_device_configuration_t get_subordinate_config()
{
    k4a_device_configuration_t camera_config = get_default_config();
    camera_config.wired_sync_mode = K4A_WIRED_SYNC_MODE_SUBORDINATE;

    // Two depth images should be seperated by MIN_TIME_BETWEEN_DEPTH_CAMERA_PICTURES_USEC to ensure the depth imaging
    // sensor doesn't interfere with the other. To accomplish this the master depth image captures
    // (MIN_TIME_BETWEEN_DEPTH_CAMERA_PICTURES_USEC / 2) before the color image, and the subordinate camera captures its
    // depth image (MIN_TIME_BETWEEN_DEPTH_CAMERA_PICTURES_USEC / 2) after the color image. This gives us two depth
    // images centered around the color image as closely as possible.
    camera_config.depth_delay_off_color_usec = MIN_TIME_BETWEEN_DEPTH_CAMERA_PICTURES_USEC / 2;
    return camera_config;
}

void save_point_cloud_new(std::vector<PointCloudData>& point_clouds) {

    std::vector<WritePointCloudData> WritePCD;

    for (auto& pcd : point_clouds) { // PointCloudData
        k4a_image_t color_image = pcd.color_image.handle(); // handle��������ת��
        k4a_image_t depth_image = pcd.depth_image.handle();

        // ��ȡ��ɫͼ��Ŀ�Ⱥ͸߶�
        int color_image_width_pixels = k4a_image_get_width_pixels(color_image);
        int color_image_height_pixels = k4a_image_get_height_pixels(color_image);

        // �������ڱ���ת�������ͼ���ͼ�����
        k4a_image_t transformed_depth_image = NULL;
        if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
            color_image_width_pixels,
            color_image_height_pixels,
            color_image_width_pixels * (int)sizeof(uint16_t),
            &transformed_depth_image))
        {
            printf("Failed to create transformed depth image\n");
        }

        // �������ڱ���������ݵ�ͼ�����
        k4a_image_t point_cloud_image = NULL;
        if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
            color_image_width_pixels,
            color_image_height_pixels,
            color_image_width_pixels * 3 * (int)sizeof(int16_t),
            &point_cloud_image))
        {
            printf("Failed to create point cloud image\n");
        }

        // �����ͼ��ת��Ϊ���ɫͼ���������ͼ��
        if (K4A_RESULT_SUCCEEDED !=
            k4a_transformation_depth_image_to_color_camera(pcd.transformation_handle, depth_image, transformed_depth_image))
        {
            printf("Failed to compute transformed depth image\n");
        }

        // ʹ��ת��������ͼ�����������ݣ�У׼����Ϊ��ɫͼ��
        if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_point_cloud(pcd.transformation_handle,
            transformed_depth_image,
            K4A_CALIBRATION_TYPE_COLOR,
            point_cloud_image))
        {
            printf("Failed to compute point cloud\n");
        }

        // ��� WritePCD ����
        WritePCD.push_back({ point_cloud_image, color_image, pcd.file_name.c_str(), 1000, 1.0, pcd.matrix_file });
    }
    
    tranformation_helpers_write_point_cloud(WritePCD);

}



//void save_point_cloud_new(k4a_transformation_t transformation_handle,
//    k4a::image depth_image1,
//    k4a::image color_image1,
//    //int count,
//    //std::string matrix)
//    // �㲥�Ļ���������һ��ȥ��
//    std::string file_name,
//    std::string matrix)
//{
//    auto time1 = std::chrono::high_resolution_clock::now();
//    //std::string file_name = output_dir + "\\depth_to_color.ply";
//    //std::cout << file_name << std::endl;
//    //��k4a::imageת����k4a_image_t
//    k4a_image_t color_image;
//    k4a_image_t depth_image;
//    color_image = color_image1.handle();
//    depth_image = depth_image1.handle();
//    cout << "save_point_cloud_new" << endl;
//    std::cout << "matrix is: " << matrix << std::endl;
//
//    // transform color image into depth camera geometry
//    // ��ȡ��ɫͼ��Ŀ�Ⱥ͸߶�
//    int color_image_width_pixels = k4a_image_get_width_pixels(color_image);
//    int color_image_height_pixels = k4a_image_get_height_pixels(color_image);
//
//    // �������ڱ���ת�������ͼ���ͼ�����
//    k4a_image_t transformed_depth_image = NULL;
//    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
//        color_image_width_pixels,
//        color_image_height_pixels,
//        color_image_width_pixels * (int)sizeof(uint16_t),
//        &transformed_depth_image))
//    {
//        printf("Failed to create transformed depth image\n");
//    }
//
//    // �������ڱ���������ݵ�ͼ�����
//    k4a_image_t point_cloud_image = NULL;
//    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
//        color_image_width_pixels,
//        color_image_height_pixels,
//        color_image_width_pixels * 3 * (int)sizeof(int16_t),
//        &point_cloud_image))
//    {
//        printf("Failed to create point cloud image\n");
//    }
//
//    // �����ͼ��ת��Ϊ���ɫͼ���������ͼ��
//    if (K4A_RESULT_SUCCEEDED !=
//        k4a_transformation_depth_image_to_color_camera(transformation_handle, depth_image, transformed_depth_image))
//    {
//        printf("Failed to compute transformed depth image\n");
//    }
//
//    // ʹ��ת��������ͼ�����������ݣ�У׼����Ϊ��ɫͼ��
//    if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_point_cloud(transformation_handle,
//        transformed_depth_image,
//        K4A_CALIBRATION_TYPE_COLOR,
//        point_cloud_image))
//    {
//        printf("Failed to compute point cloud\n");
//    }
//    auto time2 = std::chrono::high_resolution_clock::now();
//    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(time2 - time1);
//    cout << "ͼ�������õ�ʱ�䣨s��" << duration.count() * 1.0 / 1000000 << endl;
//
//    // ���������ݺͲ�ɫͼ�񱣴浽�ļ���
//    // �㲥 
//    tranformation_helpers_write_point_cloud(point_cloud_image, color_image, file_name.c_str(), 1000, 1.0, matrix);
//    // ֱ��
//    //tranformation_helpers_write_point_cloud(point_cloud_image, color_image, count, 600, 1.0, matrix);
//    auto time3 = std::chrono::high_resolution_clock::now();
//    auto duration2 = std::chrono::duration_cast<std::chrono::microseconds>(time3 - time2);
//
//    cout << "ͼ�񱣴����õ�ʱ�䣨s��" << duration2.count() * 1.0 / 1000000 << endl;    
//    // �ͷ�ͼ����Դ
//    k4a_image_release(transformed_depth_image);
//    k4a_image_release(point_cloud_image);
//}
////line 188 �� line 421���ظ���