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


#include <Python.h>


// Allowing at least 160 microseconds between depth cameras should ensure they do not interfere with one another.
constexpr uint32_t MIN_TIME_BETWEEN_DEPTH_CAMERA_PICTURES_USEC = 160;

static k4a_device_configuration_t get_master_config();
static k4a_device_configuration_t get_subordinate_config();

// 合并保存函数
struct PointCloudData {
    k4a_transformation_t transformation_handle;
    k4a::image depth_image;
    k4a::image color_image;
    std::string matrix_file;
    int count;
};

void save_point_cloud_new(std::vector<PointCloudData>& point_clouds, std::string file_name, PyObject* pName);

//void save_point_cloud_new(k4a_transformation_t transformation_handle,
//    k4a::image depth_image,
//    k4a::image color_image,
//    //int count,
//    //std::string matrix);
////以下是点播
//    std::string file_name,
//    std::string matrix);

k4a_transformation_t transformation_handle_main = NULL;
k4a_transformation_t transformation_handle_second = NULL;
k4a_transformation_t transformation_handle_second_1 = NULL;
k4a_transformation_t transformation_handle_second_2 = NULL;

int main(int argc, char** argv)
{
    float chessboard_square_length = 30; // 每个棋盘格的边长，需要作为输入参数提供
    int32_t color_exposure_usec = 8000;  // 彩色图像曝光时间，默认为8000微秒
    int32_t powerline_freq = 2;          // 源频率，默认为60Hz
    cv::Size chessboard_pattern(8, 13);  // 棋盘格的尺寸，高度和宽度都需要设置
    uint16_t depth_threshold = 1000;     // 深度阈值，默认为1米
    size_t num_devices = 2;              // 设备数量，默认为2台
    double calibration_timeout = 120.0;  // 校准超时时间，默认为120秒（2分钟）
    double greenscreen_duration = std::numeric_limits<double>::max(); // 绿幕持续时间，默认为无限大，一直运行

    vector<uint32_t> device_indices{ 0 }; // 设置一个MultiDeviceCapturer来同时捕获多台相机的图像
    // 注意device_indices中索引的顺序不一定保留，
    // 因为MultiDeviceCapturer会尝试根据插入同步线的设备来确定主设备。
    // 初始时只包含0，如果需要可以添加另外的设备索引

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

    // 检查设备数量是否为1或2，如果不是则输出错误信息并退出程序。如果设备数量为2，则将设备索引添加到device_indices中。
    if (num_devices != 3 && num_devices != 2 && num_devices != 1)
    {
        cerr << "Invalid choice for number of devices!\n";
        exit(1);
    }
    else if (num_devices == 2)
    {
        device_indices.emplace_back(1); // 现在设备索引为 { 0, 1 }
    }
    else if (num_devices == 3) {
        device_indices.emplace_back(1);
        device_indices.emplace_back(2);
    }// 修改部分

    // 使用device_indices、color_exposure_usec和powerline_freq参数创建MultiDeviceCapturer对象capturer。
    MultiDeviceCapturer capturer(device_indices, color_exposure_usec, powerline_freq);

    // 获取主设备的配置信息并将同步模式设置为独立模式。获取从属设备的配置信息。
    k4a_device_configuration_t main_config = get_master_config();
    if (num_devices == 1) // no need to have a master cable if it's standalone
    {
        main_config.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
    }
    k4a_device_configuration_t secondary_config = get_subordinate_config();

    // 获取主设备的校准信息。
    k4a::calibration main_calibration = capturer.get_master_device().get_calibration(main_config.depth_mode,
        main_config.color_resolution);

    // 使用主设备的校准信息创建一个转换对象，用于在主颜色相机空间中转换主深度图像。
    k4a::transformation main_depth_to_main_color(main_calibration);
    std::cout << "I am here" << std::endl;
    // 使用主设备和从属设备的配置信息启动设备。
    capturer.start_devices(main_config, secondary_config);



    // 初始化Python解释器
    Py_Initialize();

    // 修改sys.path
    PyObject* sysPath = PySys_GetObject("path");
    PyList_Append(sysPath, PyUnicode_FromString("D:/Code/python"));

    // 加载Python脚本
    PyObject* pName = PyUnicode_DecodeFSDefault("relative_trans");

    PyObject* pModule = PyImport_Import(pName);





    
    if (num_devices == 1) {
        //This wraps all the device-to-device details
        //对设备之间的细节进行封装
        std::cout << "I am here" << std::endl;
        std::chrono::time_point<std::chrono::system_clock> start_time = std::chrono::system_clock::now();
        transformation_handle_main = k4a_transformation_create(&main_calibration);
        int count = 0;
        // 在一定时间内进行处理
        while (std::chrono::duration<double>(std::chrono::system_clock::now() - start_time).count() <
            greenscreen_duration)
        {
            // pointcloud 定义输出点云文件的目录
            std::string outputDirectory0 = "E:\\code\\kinect\\greenScreen\\ply1";

            // 获取同步的捕获帧
            vector<k4a::capture> captures;
            captures = capturer.get_synchronized_captures(secondary_config, true);

            // 获取主彩色图像和主深度图像
            k4a::image main_color_image = captures[0].get_color_image();
            k4a::image main_depth_image = captures[0].get_depth_image();

            // 保存点云文件
            std::string filename = outputDirectory0 + "\\" + std::to_string(count) + ".ply";
            auto time_1 = std::chrono::high_resolution_clock::now();

            std::vector<PointCloudData> point_clouds;

            // 填充 point_clouds 数组
            point_clouds.push_back({ transformation_handle_main, main_depth_image, main_color_image, ""});

            // 点播
            save_point_cloud_new(point_clouds, filename, pName);

            //直播
            //save_point_cloud_new(transformation_handle_main, main_depth_image, main_color_image, count);
            auto time_2 = std::chrono::high_resolution_clock::now();

            auto duration_1 = std::chrono::duration_cast<std::chrono::microseconds>(time_2 - time_1);

            std::cout << "第一次保存使用的秒数" << duration_1.count() * 1.0 / 1000000 << endl;
            count++;
            if (count == 9)
                break;
        }
    }
    else if (num_devices == 2)
    {
        //This wraps all the device-to-device details
        //对设备之间的细节进行封装
        
        // 获取次级设备的校准数据
        k4a::calibration secondary_calibration =
            capturer.get_subordinate_device_by_index(0).get_calibration(secondary_config.depth_mode,
                secondary_config.color_resolution);

        transformation_handle_main = k4a_transformation_create(&main_calibration);
        transformation_handle_second = k4a_transformation_create(&secondary_calibration);

        std::chrono::time_point<std::chrono::system_clock> start_time = std::chrono::system_clock::now();

        int count = 0;




        // 初始化Python解释器
        //Py_Initialize();

        //// 修改sys.path
        //PyObject* sysPath = PySys_GetObject("path");
        //PyList_Append(sysPath, PyUnicode_FromString("D:/Code/python"));

        //// 加载Python脚本
        //PyObject* pName = PyUnicode_DecodeFSDefault("relative_trans");

        //PyObject* pModule = PyImport_Import(pName);

        //Py_DECREF(pName);



        auto end_time = std::chrono::high_resolution_clock::now();



        // 在一定时间内进行处理
        while (std::chrono::duration<double>(std::chrono::system_clock::now() - start_time).count() <
            greenscreen_duration)
        {
            // pointcloud 定义输出点云文件的目录
            std::string outputDirectory = "pointcloud/capture";

            // 获取同步的捕获帧
            vector<k4a::capture> captures = capturer.get_synchronized_captures(secondary_config, true);

            // 获取主彩色图像和主深度图像
            k4a::image main_color_image = captures[0].get_color_image();
            k4a::image main_depth_image = captures[0].get_depth_image();

            // 获取次级深度图像 和次级彩色图像
            k4a::image secondary_depth_image = captures[1].get_depth_image();
            k4a::image secondary_color_image = captures[1].get_color_image();

            // 保存点云文件
            std::string filename = outputDirectory + "\\" + std::to_string(count) + ".ply";

            // 配准矩阵
            std::string matrix1 = "pointcloud/1to0_matrix.txt";

            // 合并保存
            std::vector<PointCloudData> point_clouds;

            // push back不花时间
            // 填充 point_clouds 数组
            point_clouds.push_back({ transformation_handle_main, main_depth_image, main_color_image, "" });
            point_clouds.push_back({ transformation_handle_second, secondary_depth_image, secondary_color_image, matrix1 });
           
            auto start_time = std::chrono::high_resolution_clock::now();

            save_point_cloud_new(point_clouds, filename, pModule);

            // 记录结束时间点
            auto end_time = std::chrono::high_resolution_clock::now();      
    
            // 计算执行时间
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
            // 打印执行时间（以毫秒为单位）
            std::cout << "一次save函数时间：" << duration.count() << " 毫秒" << std::endl;
            count++;

            break;

        }
    }
    else if (num_devices == 3)
    {
        int count = 0;
        // 获取从属设备的校准数据
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

        // 在一定时间内进行处理
        while (std::chrono::duration<double>(std::chrono::system_clock::now() - start_time).count() <
            greenscreen_duration)
        {
            time_t start = clock();    //获取开始的时间戳

            // 输出点云文件的相对路径  
            std::string outputDirectory = "pointcloud/capture";

            // 获取同步的捕获帧
            vector<k4a::capture> captures;
            captures = capturer.get_synchronized_captures(secondary_config, true);

            // 获取主彩色图像和主深度图像
            k4a::image main_color_image = captures[0].get_color_image();
            k4a::image main_depth_image = captures[0].get_depth_image();

            // 获取从属1深度图像和从属1彩色图像
            k4a::image secondary_depth_image_1 = captures[1].get_depth_image();
            k4a::image secondary_color_image_1 = captures[1].get_color_image();

            // 获取从属2深度图像和从属2彩色图像
            k4a::image secondary_depth_image_2 = captures[2].get_depth_image();
            k4a::image secondary_color_image_2 = captures[2].get_color_image();

            // 保存点云文件
            std::string filename = outputDirectory + "\\" + std::to_string(count) + ".ply";
            
            // 配准矩阵
            std::string matrix1 = "pointcloud/1to0_matrix.txt";
            std::string matrix2 = "pointcloud/2to0_matrix.txt";

            // 填充 point_clouds 数组
            std::vector<PointCloudData> point_clouds;
            point_clouds.push_back({ transformation_handle_main, main_depth_image, main_color_image, "" });
            point_clouds.push_back({ transformation_handle_main, main_depth_image, main_color_image, matrix1 });
            point_clouds.push_back({ transformation_handle_main, main_depth_image, main_color_image, matrix2 });

            save_point_cloud_new(point_clouds, filename, pName);
            count++;
            //break;


            time_t end = clock();     //获取结束的时间戳 
            time_t run_time = end - start;  //运行时间=结束的时间戳-减去-开始的时间戳  
            cout << run_time << "ms" << endl;   //输出运行时间，单位是毫秒ms,  1秒=1000毫秒 
        }
    }
    else
    {
        cerr << "Invalid number of devices!" << endl;
        exit(1);
    }

    Py_Finalize();


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

void save_point_cloud_new(std::vector<PointCloudData>& point_clouds, std::string file_name, PyObject* pModule) {

    std::vector<WritePointCloudData> WritePCD;

    for (auto& pcd : point_clouds) { // PointCloudData
        k4a_image_t color_image = pcd.color_image.handle(); // handle用于类型转换
        k4a_image_t depth_image = pcd.depth_image.handle();

        // 获取彩色图像的宽度和高度
        int color_image_width_pixels = k4a_image_get_width_pixels(color_image);
        int color_image_height_pixels = k4a_image_get_height_pixels(color_image);

        // 创建用于保存转换后深度图像的图像对象
        k4a_image_t transformed_depth_image = NULL;
        if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
            color_image_width_pixels,
            color_image_height_pixels,
            color_image_width_pixels * (int)sizeof(uint16_t),
            &transformed_depth_image))
        {
            printf("Failed to create transformed depth image\n");
        }

        // 创建用于保存点云数据的图像对象
        k4a_image_t point_cloud_image = NULL;
        if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
            color_image_width_pixels,
            color_image_height_pixels,
            color_image_width_pixels * 3 * (int)sizeof(int16_t),
            &point_cloud_image))
        {
            printf("Failed to create point cloud image\n");
        }

        // 将深度图像转换为与彩色图像对齐的深度图像
        if (K4A_RESULT_SUCCEEDED !=
            k4a_transformation_depth_image_to_color_camera(pcd.transformation_handle, depth_image, transformed_depth_image))
        {
            printf("Failed to compute transformed depth image\n");
        }

        // 使用转换后的深度图像计算点云数据，校准类型为彩色图像
        if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_point_cloud(pcd.transformation_handle,
            transformed_depth_image,
            K4A_CALIBRATION_TYPE_COLOR,
            point_cloud_image))
        {
            printf("Failed to compute point cloud\n");
        }

        // 填充 WritePCD 数组
        WritePCD.push_back({ point_cloud_image, color_image, pcd.matrix_file });
    }
    
    tranformation_helpers_write_point_cloud(WritePCD, file_name.c_str(), 1000, pModule);
}