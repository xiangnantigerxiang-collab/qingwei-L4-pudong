#ifdef SUPPORT_D415_ALIGN
#include "librealsense/realsense_camera_parameter.h"


ds_rect_resolutions width_height_to_ds_rect_resolutions(uint32_t width, uint32_t height)
{
    for (auto& elem : resolutions_list)
    {
        if (uint32_t(elem.second.x) == width && uint32_t(elem.second.y) == height)
            return elem.first;
    }
    return max_ds_rect_resolutions;
}

//  D415 Depth 内参获取 老版本使用
rs2_intrinsics get_d400_intrinsic_by_resolution_coefficients_table(const std::vector<uint8_t>& raw_data, uint32_t width, uint32_t height)
{
    auto table = check_calib<d400_coefficients_table>(raw_data);

    auto resolution = width_height_to_ds_rect_resolutions(width, height);

    // if (width == 848 && height == 100) // Special 848x100 resolution is available in some firmware versions
    //     // for this resolution we use the same base-intrinsics as for 848x480 and adjust them later
    // {
    //     resolution = width_height_to_ds_rect_resolutions(width, 480);
    // }

    rs2_intrinsics intrinsics;
    intrinsics.width = resolutions_list[resolution].x;
    intrinsics.height = resolutions_list[resolution].y;

    auto rect_params = static_cast<const rsutils::number::float4>(table->rect_params[resolution]);
    // DS5U - assume ideal intrinsic params
    if ((rect_params.x == rect_params.y) && (rect_params.z == rect_params.w))
    {
        rect_params.x = rect_params.y = intrinsics.width * 1.5f;
        rect_params.z = intrinsics.width * 0.5f;
        rect_params.w = intrinsics.height * 0.5f;
    }
    intrinsics.fx = rect_params[0];
    intrinsics.fy = rect_params[1];
    intrinsics.ppx = rect_params[2];
    intrinsics.ppy = rect_params[3];
    intrinsics.model = RS2_DISTORTION_BROWN_CONRADY;
    memset(intrinsics.coeffs, 0, sizeof(intrinsics.coeffs));  // All coefficients are zeroed since rectified depth is defined as CS origin

    // In case of the special 848x100 resolution adjust the intrinsics
    if (width == 848 && height == 100)
    {
        intrinsics.height = 100;
        intrinsics.ppy -= 190;
    }

    // 打印深度内参
    debug_info("=== DEPTH CAMERA INTRINSICS ===\n");
    debug_info("Resolution: %d x %d\n", intrinsics.width, intrinsics.height);
    debug_info("Focal Length (fx, fy): %.6f, %.6f\n", intrinsics.fx, intrinsics.fy);
    debug_info("Principal Point (ppx, ppy): %.6f, %.6f\n", intrinsics.ppx, intrinsics.ppy);
    debug_info("Distortion Model: %s\n", intrinsics.model == RS2_DISTORTION_INVERSE_BROWN_CONRADY ? "Inverse Brown Conrady" : "Unknown");
    debug_info("Distortion Coefficients: %.6f, %.6f, %.6f, %.6f, %.6f\n",
               intrinsics.coeffs[0], intrinsics.coeffs[1], intrinsics.coeffs[2],
               intrinsics.coeffs[3], intrinsics.coeffs[4]);
    return intrinsics;
}



//  D415 Depth 内参获取 新版本使用，适配RECPARAMSGET命令
// Parse intrinsics from newly added RECPARAMSGET command
bool try_get_d400_intrinsic_by_resolution_new(const std::vector<uint8_t>& raw_data, uint32_t width, uint32_t height, rs2_intrinsics* result)
{
    auto count = raw_data.size() / sizeof(new_calibration_item);
    auto items = (new_calibration_item*)raw_data.data();
    for (int i = 0; i < count; i++)
    {
        auto&& item = items[i];
        if (item.width == width && item.height == height)
        {
            result->width = width;
            result->height = height;
            result->ppx = item.ppx;
            result->ppy = item.ppy;
            result->fx = item.fx;
            result->fy = item.fy;
            result->model = RS2_DISTORTION_BROWN_CONRADY;
            memset(result->coeffs, 0, sizeof(result->coeffs));  // All coefficients are zeroed since rectified depth is defined as CS origin

            // 打印深度内参
            debug_info("=== DEPTH CAMERA INTRINSICS ===\n");
            debug_info("Resolution: %d x %d\n", result->width, result->height);
            debug_info("Focal Length (fx, fy): %.6f, %.6f\n", result->fx, result->fy);
            debug_info("Principal Point (ppx, ppy): %.6f, %.6f\n", result->ppx, result->ppy);
            debug_info("Distortion Model: %s\n", result->model == RS2_DISTORTION_INVERSE_BROWN_CONRADY ? "Inverse Brown Conrady" : "Unknown"); 
            debug_info("Distortion Coefficients: %.6f, %.6f, %.6f, %.6f, %.6f\n",
                       result->coeffs[0], result->coeffs[1], result->coeffs[2],
                       result->coeffs[3], result->coeffs[4]);
            return true;
        }
    }
    return false;
}

//  D415 Color 内参获取
rs2_intrinsics get_d400_color_stream_intrinsic(const std::vector<uint8_t>& raw_data, uint32_t width, uint32_t height)
{
    auto table = check_calib<d400_rgb_calibration_table>(raw_data);

    // Compensate for aspect ratio as the normalized intrinsic is calculated with a single resolution
    rsutils::number::float3x3 intrin = table->intrinsic;
    float calib_aspect_ratio = 9.f / 16.f; // shall be overwritten with the actual calib resolution

    if (table->calib_width && table->calib_height)
        calib_aspect_ratio = float(table->calib_height) / float(table->calib_width);
    else
    {
        debug_warn("RGB Calibration resolution is not specified, using default 16/9 Aspect ratio");
    }

    // Compensate for aspect ratio
    float actual_aspect_ratio = height / (float)width;
    if (actual_aspect_ratio < calib_aspect_ratio)
    {
        intrin(1, 1) *= calib_aspect_ratio / actual_aspect_ratio;
        intrin(2, 1) *= calib_aspect_ratio / actual_aspect_ratio;
    }
    else
    {
        intrin(0, 0) *= actual_aspect_ratio / calib_aspect_ratio;
        intrin(2, 0) *= actual_aspect_ratio / calib_aspect_ratio;
    }

    // Calculate specific intrinsic parameters based on the normalized intrinsic and the sensor's resolution
    rs2_intrinsics calc_intrinsic{
        static_cast<int>(width),
        static_cast<int>(height),
        ((1 + intrin(2, 0)) * width) / 2.f,
        ((1 + intrin(2, 1)) * height) / 2.f,
        intrin(0, 0) * width / 2.f,
        intrin(1, 1) * height / 2.f,
        RS2_DISTORTION_INVERSE_BROWN_CONRADY  // The coefficients shall be use for undistort
    };
    memcpy(calc_intrinsic.coeffs, table->distortion, sizeof(table->distortion));
    //LOG_DEBUG(endl << array2str((float_4&)(calc_intrinsic.fx, calc_intrinsic.fy, calc_intrinsic.ppx, calc_intrinsic.ppy)) << endl);

    static rs2_intrinsics ref{};
    if (memcmp(&calc_intrinsic, &ref, sizeof(rs2_intrinsics)))
    {
        // 打印彩色内参
        debug_info("=== COLOR CAMERA INTRINSICS ===\n");
        debug_info("Resolution: %d x %d\n", calc_intrinsic.width, calc_intrinsic.height);
        debug_info("Focal Length (fx, fy): %.6f, %.6f\n", calc_intrinsic.fx, calc_intrinsic.fy);
        debug_info("Principal Point (ppx, ppy): %.6f, %.6f\n", calc_intrinsic.ppx, calc_intrinsic.ppy);
        debug_info("Distortion Model: %s\n", calc_intrinsic.model == RS2_DISTORTION_INVERSE_BROWN_CONRADY ? "Inverse Brown Conrady" : "Unknown");
        debug_info("Distortion Coefficients: %.6f, %.6f, %.6f, %.6f, %.6f\n",
                   calc_intrinsic.coeffs[0], calc_intrinsic.coeffs[1], calc_intrinsic.coeffs[2],
                   calc_intrinsic.coeffs[3], calc_intrinsic.coeffs[4]);
        ref = calc_intrinsic;
    }
    return calc_intrinsic;
}

// D415外参数据获取
pose get_d400_color_stream_extrinsic(const std::vector<uint8_t>& raw_data)
{
    auto table = check_calib<d400_rgb_calibration_table>(raw_data);
    rsutils::number::float3 trans_vector = table->translation_rect;
    rsutils::number::float3x3 rect_rot_mat = table->rotation_matrix_rect;
    float trans_scale = -0.001f; // Convert units from mm to meter. Extrinsic of color is referenced to the Depth Sensor CS

    trans_vector.x *= trans_scale;
    trans_vector.y *= trans_scale;
    trans_vector.z *= trans_scale;

    return{ rect_rot_mat,trans_vector };
}

// D405 Color 内参数据获取
//D405 needs special calculation because the ISP crops the full sensor image using non linear transformation.
rs2_intrinsics get_d405_color_stream_intrinsic(const std::vector<uint8_t>& raw_data, uint32_t width, uint32_t height)
{
    struct resolution
    {
        uint32_t width = 0;
        uint32_t height = 0;
    };

    // Convert normalized focal length and principal point to pixel units (K matrix format)
    auto normalized_k_to_pixels = [&]( rsutils::number::float3x3 & k, resolution res )
    {
        if( res.width == 0 || res.height == 0 )
        {
            debug_err("Unsupported resolution used (%d, %d)", res.width, res.height);
        }

        k( 0, 0 ) = k( 0, 0 ) * res.width / 2.f;      // fx
        k( 1, 1 ) = k( 1, 1 ) * res.height / 2.f;     // fy
        k( 2, 0 ) = ( k( 2, 0 ) + 1 ) * res.width / 2.f;  // ppx
        k( 2, 1 ) = ( k( 2, 1 ) + 1 ) * res.height / 2.f;  // ppy
    };

    // Scale focal length and principal point in pixel units from one resolution to another (K matrix format)
    auto scale_pixel_k = [&]( rsutils::number::float3x3 & k, resolution in_res, resolution out_res)
    {
        if( in_res.width == 0 || in_res.height == 0 || out_res.width == 0 || out_res.height == 0 )
        {
            debug_err("Unsupported resolution used (in: %d, %d, out: %d, %d)", in_res.width, in_res.height, out_res.width, out_res.height);
        }

        float scale_x = out_res.width / static_cast< float >( in_res.width );
        float scale_y = out_res.height / static_cast< float >( in_res.height );
        float scale = std::max( scale_x, scale_y );
        float shift_x = ( in_res.width * scale - out_res.width ) / 2.f;
        float shift_y = ( in_res.height * scale - out_res.height ) / 2.f;

        k( 0, 0 ) = k( 0, 0 ) * scale;  // fx
        k( 1, 1 ) = k( 1, 1 ) * scale;  // fy
        k( 2, 0 ) = k( 2, 0 ) * scale - shift_x;  // ppx
        k( 2, 1 ) = k( 2, 1 ) * scale - shift_y;  // ppy
    };

    auto table = check_calib<d400_rgb_calibration_table>(raw_data);
    auto output_res = resolution{ width, height };
    auto calibration_res = resolution{ table->calib_width, table->calib_height };

    rsutils::number::float3x3 k = table->intrinsic;
    if( width == 1280 && height == 720 )
        normalized_k_to_pixels( k, output_res );
    else if( width == 640 && height == 480 ) // 640x480 is 4:3 not 16:9 like other available resolutions, ISP handling is different.
    {
        auto raw_res = resolution{ 1280, 800 };
        // Extrapolate K to raw resolution
        float scale_y = calibration_res.height / static_cast< float >( raw_res.height );
        k( 1, 1 ) = k( 1, 1 ) * scale_y;  // fy
        k( 2, 1 ) = k( 2, 1 ) * scale_y;  // ppy
        normalized_k_to_pixels( k, raw_res );
        // Handle ISP scaling to 770x480
        auto scale_res = resolution{ 770, 480 };
        scale_pixel_k( k, raw_res, scale_res );
        // Handle ISP cropping to 640x480
        k( 2, 0 ) = k( 2, 0 ) - ( scale_res.width - output_res.width ) / 2;  // ppx
        k( 2, 1 ) = k( 2, 1 ) - ( scale_res.height - output_res.height ) / 2;  // ppy
    }
    else
    {
        normalized_k_to_pixels( k, calibration_res );
        scale_pixel_k( k, calibration_res, output_res );
    }

    // Convert k matrix format to rs2 format
    rs2_intrinsics rs2_intr{
        static_cast< int >( width ),
        static_cast< int >( height ),
        k( 2, 0 ),
        k( 2, 1 ),
        k( 0, 0 ),
        k( 1, 1 ),
        RS2_DISTORTION_INVERSE_BROWN_CONRADY  // The coefficients shall be use for undistort
    };
    std::memcpy( rs2_intr.coeffs, table->distortion, sizeof( table->distortion ) );

    debug_info("=== D405 COLOR CAMERA INTRINSICS ===\n");
    debug_info("Resolution: %d x %d\n", rs2_intr.width, rs2_intr.height);
    debug_info("Focal Length (fx, fy): %.6f, %.6f\n", rs2_intr.fx, rs2_intr.fy);
    debug_info("Principal Point (ppx, ppy): %.6f, %.6f\n", rs2_intr.ppx, rs2_intr.ppy);
    debug_info("Distortion Model: %s\n", rs2_intr.model == RS2_DISTORTION_INVERSE_BROWN_CONRADY ? "Inverse Brown Conrady" : "Unknown"); 
    debug_info("Distortion Coefficients: %.6f, %.6f, %.6f, %.6f, %.6f\n",
               rs2_intr.coeffs[0], rs2_intr.coeffs[1], rs2_intr.coeffs[2],
               rs2_intr.coeffs[3], rs2_intr.coeffs[4]);
    return rs2_intr;
}

/**
 * @brief 解析Realsense I2C配置文件
 * @param json_file_path JSON配置文件路径（如"../cfg/align.json"）
 * @param config 输出参数：解析后的配置数据
 * @return 0成功，-1失败
 */
int parse_realsense_config(const char* json_file_path, RealsenseConfig* config) {
    // 入参合法性检查
    if (json_file_path == NULL || config == NULL) {
        debug_err("parse_realsense_config: Invalid input parameters!\n");
        return -1;
    }

    // 步骤1：打开并读取JSON文件
    FILE* fp = fopen(json_file_path, "r");
    if (fp == NULL) {
        debug_err("parse_realsense_config: Failed to open file %s (errno: %d)!\n", 
                 json_file_path, errno);
        return -1;
    }

    // 获取文件大小
    fseek(fp, 0, SEEK_END);
    long file_size = ftell(fp);
    if (file_size <= 0) {
        debug_err("parse_realsense_config: Empty file %s!\n", json_file_path);
        fclose(fp);
        return -1;
    }
    fseek(fp, 0, SEEK_SET);

    // 分配内存读取文件内容
    char* json_content = (char*)malloc(file_size + 1);
    if (json_content == NULL) {
        debug_err("parse_realsense_config: Memory allocation failed for file content!\n");
        fclose(fp);
        return -1;
    }

    // 读取文件内容
    size_t read_bytes = fread(json_content, 1, file_size, fp);
    if (read_bytes != file_size) {
        debug_err("parse_realsense_config: Read file %s failed (read %zu/%ld bytes)!\n",
                 json_file_path, read_bytes, file_size);
        free(json_content);
        fclose(fp);
        return -1;
    }
    json_content[file_size] = '\0'; // 确保字符串以'\0'结尾
    fclose(fp);

    // 步骤2：解析JSON数据
    cJSON* root = cJSON_Parse(json_content);
    free(json_content); // 立即释放文件内容内存
    if (root == NULL) {
        const char* error_ptr = cJSON_GetErrorPtr();
        debug_err("parse_realsense_config: JSON parse failed at: %s\n", 
                 error_ptr ? error_ptr : "unknown position");
        return -1;
    }

    // 步骤3：提取Realsense_i2c节点
    cJSON* Realsense_i2c = cJSON_GetObjectItem(root, "Realsense_i2c");
    if (Realsense_i2c == NULL || !cJSON_IsObject(Realsense_i2c)) {
        debug_err("parse_realsense_config: 'Realsense_i2c' node not found or invalid type!\n");
        cJSON_Delete(root);
        return -1;
    }

    // 步骤4：提取bus_number（整数）
    cJSON* bus_number = cJSON_GetObjectItem(Realsense_i2c, "bus_number");
    if (bus_number == NULL || !cJSON_IsNumber(bus_number)) {
        debug_err("parse_realsense_config: 'bus_number' is missing or not an integer!\n");
        cJSON_Delete(root);
        return -1;
    }
    config->bus_number = bus_number->valueint;

    // 步骤5：提取device_address（十六进制字符串转数值）
    cJSON* device_address = cJSON_GetObjectItem(Realsense_i2c, "device_address");
    if (device_address == NULL || !cJSON_IsString(device_address)) {
        debug_err("parse_realsense_config: 'device_address' is missing or not a string!\n");
        cJSON_Delete(root);
        return -1;
    }

    // 字符串转十六进制数值（适配"0x10"/"0x20"等格式）
    char* end_ptr = NULL;
    long addr_val = strtol(device_address->valuestring, &end_ptr, 16);
    // 校验转换结果（避免非法值如"0xGG"）
    if (end_ptr == device_address->valuestring || addr_val < 0 || addr_val > 0xFF) {
        debug_err("parse_realsense_config: 'device_address' %s is invalid (must be 0x00~0xFF)!\n",
                 device_address->valuestring);
        cJSON_Delete(root);
        return -1;
    }
    config->device_addr = (unsigned char)addr_val;

    // 步骤6：提取Realsense_D2C（整数）
    cJSON* Realsense_D2C = cJSON_GetObjectItem(Realsense_i2c, "Realsense_D2C");
    if (Realsense_D2C == NULL || !cJSON_IsNumber(Realsense_D2C)) {
        debug_err("parse_realsense_config: 'Realsense_D2C' is missing or not an integer!\n");
        cJSON_Delete(root);
        return -1;
    }
    config->Realsense_D2C = Realsense_D2C->valueint;
    
     // 步骤7：提取Realsense_C2D（整数）
     cJSON* Realsense_C2D = cJSON_GetObjectItem(Realsense_i2c, "Realsense_C2D");
    if (Realsense_C2D == NULL || !cJSON_IsNumber(Realsense_C2D)) {
        debug_err("parse_realsense_config: 'Realsense_C2D' is missing or not an integer!\n");
        cJSON_Delete(root);
        return -1;
    }
    config->Realsense_C2D = Realsense_C2D->valueint;

    // 步骤8：提取 Realsense_Device（整数）
    cJSON* Realsense_Device = cJSON_GetObjectItem(Realsense_i2c, "Realsense_Device");
    if (Realsense_Device == NULL || !cJSON_IsNumber(Realsense_Device)) {
        debug_err("parse_realsense_config: 'Realsense_Device' is missing or not an integer!\n");
        cJSON_Delete(root);
        return -1;
    }
    config->Realsense_Device = Realsense_Device->valueint;

    // 步骤9：提取 High_Accuracy_Mode（整数）
    cJSON* High_Accuracy_Mode = cJSON_GetObjectItem(Realsense_i2c, "High_Accuracy_Mode");
    if (High_Accuracy_Mode == NULL || !cJSON_IsNumber(High_Accuracy_Mode)) {
        debug_err("parse_realsense_config: 'High_Accuracy_Mode' is missing or not an integer!\n");
        cJSON_Delete(root);
        return -1;
    }
    config->High_Accuracy_Mode = High_Accuracy_Mode->valueint;
    
    // 步骤9：释放JSON内存并返回成功
    cJSON_Delete(root);
    debug_info("parse_realsense_config: Parse success - bus: %d, addr: 0x%02X, Realsense_Device: %d, High_Accuracy_Mode: %d\n", config->bus_number, config->device_addr, config->Realsense_Device, config->High_Accuracy_Mode);
    return 0;
}

#endif
