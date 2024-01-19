// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once
#include <k4a/k4a.h>
#include <string>
#include <torch/torch.h>
#include <fstream>
#include <vector>

struct WritePointCloudData {
    const k4a_image_t point_cloud_image;
    const k4a_image_t color_image;
    const std::string& matrix_file;
};
// 以下是点播
void tranformation_helpers_write_point_cloud(const k4a_image_t point_cloud_image,
    const k4a_image_t color_image,
    const char* file_name,
    float distance_threshold,
    float leaf_size,
    const std::string& matrix_file);


// 以下是直播
//void tranformation_helpers_write_point_cloud(const k4a_image_t point_cloud_image,
//    const k4a_image_t color_image,
//    int i,
//    float distance_threshold,
//    float leaf_size,
//    const std::string& matrix_file);

k4a_image_t downscale_image_2x2_binning(const k4a_image_t color_image);