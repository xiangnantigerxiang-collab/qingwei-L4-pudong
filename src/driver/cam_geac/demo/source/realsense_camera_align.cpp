#ifdef SUPPORT_D415_ALIGN
#include "librealsense/realsense_camera_align.h"

template<class GET_DEPTH, class TRANSFER_PIXEL>
void align_images(const rs2_intrinsics& depth_intrin, const rs2_extrinsics& depth_to_other,
        const rs2_intrinsics& other_intrin, GET_DEPTH get_depth, TRANSFER_PIXEL transfer_pixel)
{
    // Iterate over the pixels of the depth image
#pragma omp parallel for schedule(dynamic)
    for (int depth_y = 0; depth_y < depth_intrin.height; ++depth_y)
    {
        int depth_pixel_index = depth_y * depth_intrin.width;
        for (int depth_x = 0; depth_x < depth_intrin.width; ++depth_x, ++depth_pixel_index)
        {
            // Skip over depth pixels with the value of zero, we have no depth data so we will not write anything into our aligned images
            if (float depth = get_depth(depth_pixel_index))
            {
                // Map the top-left corner of the depth pixel onto the other image
                float depth_pixel[2] = { depth_x - 0.5f, depth_y - 0.5f }, depth_point[3], other_point[3], other_pixel[2];
                rs2_deproject_pixel_to_point(depth_point, &depth_intrin, depth_pixel, depth);
                rs2_transform_point_to_point(other_point, &depth_to_other, depth_point);
                rs2_project_point_to_pixel(other_pixel, &other_intrin, other_point);
                const int other_x0 = static_cast<int>(other_pixel[0] + 0.5f);
                const int other_y0 = static_cast<int>(other_pixel[1] + 0.5f);

                // Map the bottom-right corner of the depth pixel onto the other image
                depth_pixel[0] = depth_x + 0.5f; depth_pixel[1] = depth_y + 0.5f;
                rs2_deproject_pixel_to_point(depth_point, &depth_intrin, depth_pixel, depth);
                rs2_transform_point_to_point(other_point, &depth_to_other, depth_point);
                rs2_project_point_to_pixel(other_pixel, &other_intrin, other_point);
                const int other_x1 = static_cast<int>(other_pixel[0] + 0.5f);
                const int other_y1 = static_cast<int>(other_pixel[1] + 0.5f);

                if (other_x0 < 0 || other_y0 < 0 || other_x1 >= other_intrin.width || other_y1 >= other_intrin.height)
                    continue;

                // Transfer between the depth pixels and the pixels inside the rectangle on the other image
                for (int y = other_y0; y <= other_y1; ++y)
                {
                    for (int x = other_x0; x <= other_x1; ++x)
                    {
                        transfer_pixel(depth_pixel_index, y * other_intrin.width + x);
                    }
                }
            }
        }
    }
}

CMgrCameraAlign::CMgrCameraAlign()
{
    rgb_len = 0;
    depth_len = 0;
    align_depth_to_color = 1;  // 默认启用深度到彩色对齐
    align_color_to_depth = 0;  // 默认禁用彩色到深度对齐
    realsense_device = 1;  // 默认设备型号 (D415)
    high_accuracy_mode = 0;  // 默认禁用高准确率模式
}

CMgrCameraAlign::~CMgrCameraAlign()
{
}

bool CMgrCameraAlign::getparameter()
{
    RealsenseConfig AlignConfig;
    int ret = parse_realsense_config("./cfg/align.json", &AlignConfig);
    if (ret != 0) {
        debug_err("getparameter: Failed to parse JSON configuration!\n");
        return false;
    }

    I2COperations* ops = i2c_ops_create(AlignConfig.bus_number, AlignConfig.device_addr);
    if (ops == NULL) {
        debug_err("Getparameter: Failed to create I2C operation instance !!!\n");
        return false; 
    }

    align_depth_to_color = AlignConfig.Realsense_D2C;
    align_color_to_depth = AlignConfig.Realsense_C2D;
    realsense_device = AlignConfig.Realsense_Device;
    high_accuracy_mode = AlignConfig.High_Accuracy_Mode;

    //Depth intrinsics 获取，优先使用新版本的命令获取，如果失败再使用老版本的命令获取
    uint8_t* depth_data = execute_all_operations(ops, DATA_TYPE_DEPTH_NEW, &depth_len);
    if (depth_data && depth_len > 0) {
        depth_raw_data.assign(depth_data, depth_data + depth_len);
    }
    if (!try_get_d400_intrinsic_by_resolution_new(depth_raw_data, depth_dwWidth, depth_dwHeight, &depth_intrinsics)) 
    {
        debug_warn("getparameter: Failed to get depth intrinsics by resolution coefficients table !!!\n");
        debug_warn("getparameter: Falling back to old method to get depth intrinsics !!!\n");
        uint8_t* depth_data = execute_all_operations(ops, DATA_TYPE_DEPTH_OLD, &depth_len);
        if (depth_data && depth_len > 0) {
            depth_raw_data.assign(depth_data, depth_data + depth_len);
        }
        depth_intrinsics = get_d400_intrinsic_by_resolution_coefficients_table(depth_raw_data, depth_dwWidth, depth_dwHeight);
    }

    // Color intrinsics 获取
    uint8_t* rgb_data = execute_all_operations(ops, DATA_TYPE_RGB, &rgb_len);
    if (rgb_data && rgb_len > 0) {
        color_raw_data.assign(rgb_data, rgb_data + rgb_len);
    }
    if(realsense_device == 0)
    {
        color_intrinsics = get_d405_color_stream_intrinsic(color_raw_data, rgb_dwWidth, rgb_dwHeight);
        //  D405 颜色流内参获取需要使用此方法
    }
    else
    {
        color_intrinsics = get_d400_color_stream_intrinsic(color_raw_data, rgb_dwWidth, rgb_dwHeight);
        //  D415 D457 颜色流内参获取需要使用此方法
    }

    // Color extrinsics 获取
    color_extrinsics = inverse(get_d400_color_stream_extrinsic(color_raw_data));
    debug_info("=== DEPTH TO COLOR EXTRINSICS (POSE) ===\n");
    debug_info("Rotation Matrix (3x3):\n");
    debug_info("%.6f, %.6f, %.6f\n", color_extrinsics.orientation(0,0), color_extrinsics.orientation(0,1), color_extrinsics.orientation(0,2));
    debug_info("%.6f, %.6f, %.6f\n", color_extrinsics.orientation(1,0), color_extrinsics.orientation(1,1), color_extrinsics.orientation(1,2));
    debug_info("%.6f, %.6f, %.6f\n", color_extrinsics.orientation(2,0), color_extrinsics.orientation(2,1), color_extrinsics.orientation(2,2));
    debug_info("Translation Vector (meters):\n");
    debug_info("%.6f, %.6f, %.6f\n", color_extrinsics.position.x, color_extrinsics.position.y, color_extrinsics.position.z);

    if(high_accuracy_mode)
    {
        int ret = set_high_accuracy_mode(ops, (DeviceType)realsense_device);
        if(ret != 0)
        {
            debug_err("getparameter: Failed to set high accuracy mode !!!\n");
            return false;
        }
        debug_info("High Accuracy Mode is enabled. Performing additional calibration steps for improved alignment accuracy.\n");
    }

    // 释放资源
    i2c_ops_destroy(ops);
    return true;
}


bool CMgrCameraAlign::setDepthResolution(uint32_t width, uint32_t height) {
    depth_dwWidth = width;
    depth_dwHeight = height;
    return true;
}
bool CMgrCameraAlign::setRgbResolution(uint32_t width, uint32_t height) {
    rgb_dwWidth = width;
    rgb_dwHeight = height;
    return true;
}

bool CMgrCameraAlign::inputFrame(int nChan, unsigned char *pData, int nDatalen, uint8_t *outData, int *out_nDatalen)
{
	if (pData == NULL)
	{
		return false;
	}
    float z_scale = 0.001;  // 固定值
    auto z_pixels = reinterpret_cast<const uint16_t*>(pData);
    *out_nDatalen = rgb_dwWidth * rgb_dwHeight * 2;


    if (nChan == 0 && align_depth_to_color) {
        memset(outData,0,*out_nDatalen);
        /*
        // 深度图对齐到颜色图 use CPU, 速度很慢，实际使用中建议使用CUDA加速
        align_images(depth_intrinsics, color_extrinsics, color_intrinsics,
            [z_pixels, z_scale](int z_pixel_index) { return z_scale * z_pixels[z_pixel_index]; },
            [outData, z_pixels](int z_pixel_index, int other_pixel_index)
        {
            outData[other_pixel_index] = outData[other_pixel_index] ?
                std::min((int)outData[other_pixel_index], (int)z_pixels[z_pixel_index]) :
                z_pixels[z_pixel_index];
        });        
        */

        // 深度图对齐到颜色图 use CUDA
        align_depth_to_other(outData, z_pixels, z_scale, depth_intrinsics, from_pose(color_extrinsics), color_intrinsics);
    }
    else if (nChan == 1 && align_color_to_depth) {
        // 颜色图对齐到深度图 use CUDA, 未做适配，不支持此功能
        // align_other_to_depth(outData, z_pixels, z_scale, depth_intrinsics, from_pose(color_extrinsics), color_intrinsics, color_pixels, 
        //     color_profile.format(), color.get_bytes_per_pixel());
    }
    else {
        debug_warn("Align inputFrame: nChan %d not support align !!!\n", nChan);
    }
    return true;
}
#endif

