#include <map>
#include <iomanip>
#include <string>
#include "../inc/librealsense/realsense_camera_iic.h"
#include "../inc/librealsense/float3.h"
#include "../inc/librealsense/rsutil.h"
#include "cJSON.h"

#include <sstream>
#include <cstring>

#pragma pack(push, 1)

// Structure to store parsed configuration
typedef struct {
    int bus_number;          // I2C bus number
    unsigned char device_addr; // Device address (converted to numeric value)
    int Realsense_D2C;  // Depth to Color Enable
    int Realsense_C2D;  // Color to Depth Enable
    int Realsense_Device;  // Realsense Device Type (0: D405, 1: D415, 2: D457)
    int High_Accuracy_Mode; // High Accuracy Mode Enable（0: Disable, 1: Enable）
} RealsenseConfig;

// Note ds_rect_resolutions is used in struct d400_coefficients_table. Update with caution.
enum ds_rect_resolutions : unsigned short
{
    res_1920_1080,
    res_1280_720,
    res_640_480,
    res_848_480,
    res_640_360,
    res_424_240,
    res_320_240,
    res_480_270,
    res_1280_800,
    res_960_540,
    reserved_1,
    reserved_2,
    res_640_400,
    // Resolutions for DS5U
    res_576_576,
    res_720_720,
    res_1152_1152,
    max_ds_rect_resolutions
};

static std::map< ds_rect_resolutions, rsutils::number::int2> resolutions_list = {
    { res_320_240,{ 320, 240 } },
    { res_424_240,{ 424, 240 } },
    { res_480_270,{ 480, 270 } },
    { res_640_360,{ 640, 360 } },
    { res_640_400,{ 640, 400 } },
    { res_640_480,{ 640, 480 } },
    { res_848_480,{ 848, 480 } },
    { res_960_540,{ 960, 540 } },
    { res_1280_720,{ 1280, 720 } },
    { res_1280_800,{ 1280, 800 } },
    { res_1920_1080,{ 1920, 1080 } },
    { res_576_576,{ 576, 576 } },
    { res_720_720,{ 720, 720 } },
    { res_1152_1152,{ 1152, 1152 } },
};

struct pose
{
    rsutils::number::float3x3 orientation;
    rsutils::number::float3 position;
};

inline rs2_extrinsics from_pose( pose a )
{
    rs2_extrinsics r;
    for( int i = 0; i < 3; i++ )
        r.translation[i] = a.position[i];
    for( int j = 0; j < 3; j++ )
        for( int i = 0; i < 3; i++ )
            r.rotation[j * 3 + i] = a.orientation( i, j );
    return r;
}


inline pose inverse( const pose & a )
{
    auto inv = transpose( a.orientation );
    return { inv, inv * a.position * -1 };
}

template<class T> class big_endian
{
    T be_value;
public:
    operator T () const
    {
        T le_value = 0;
        for (unsigned int i = 0; i < sizeof(T); ++i) *(reinterpret_cast<char*>(&le_value) + i) = *(reinterpret_cast<const char*>(&be_value) + sizeof(T) - i - 1);
        return le_value;

    }
};

#pragma pack(pop)

struct new_calibration_item
{
    uint16_t width;
    uint16_t height;
    float  fx;
    float  fy;
    float  ppx;
    float  ppy;
};

struct table_header
{
    big_endian<uint16_t>    version;        // major.minor. Big-endian
    uint16_t                table_type;     // ctCalibration
    uint32_t                table_size;     // full size including: TOC header + TOC + actual tables
    uint32_t                param;          // This field content is defined ny table type
    uint32_t                crc32;          // crc of all the actual table data excluding header/CRC

    std::string to_string() const;
};


struct d400_coefficients_table
{
    table_header        header;
    rsutils::number::float3x3            intrinsic_left;             //  left camera intrinsic data, normilized
    rsutils::number::float3x3            intrinsic_right;            //  right camera intrinsic data, normilized
    rsutils::number::float3x3            world2left_rot;             //  the inverse rotation of the left camera
    rsutils::number::float3x3            world2right_rot;            //  the inverse rotation of the right camera
    float               baseline;                   //  the baseline between the cameras in mm units
    uint32_t            brown_model;                //  Distortion model: 0 - DS distorion model, 1 - Brown model
    uint8_t             reserved1[88];
    rsutils::number::float4              rect_params[max_ds_rect_resolutions];
    uint8_t             reserved2[64];
};

struct d400_rgb_calibration_table
{
    table_header        header;
    // RGB Intrinsic
    rsutils::number::float3x3            intrinsic;                  // normalized by [-1 1]
    float               distortion[5];              // RGB forward distortion coefficients, Brown model
    // RGB Extrinsic
    rsutils::number::float3              rotation;                   // RGB rotation angles (Rodrigues)
    rsutils::number::float3              translation;                // RGB translation vector, mm
    // RGB Projection
    float               projection[12];             // Projection matrix from depth to RGB [3 X 4]
    uint16_t            calib_width;                // original calibrated resolution
    uint16_t            calib_height;
    // RGB Rectification Coefficients
    rsutils::number::float3x3            intrinsic_matrix_rect;      // RGB intrinsic matrix after rectification
    rsutils::number::float3x3            rotation_matrix_rect;       // Rotation matrix for rectification of RGB
    rsutils::number::float3              translation_rect;           // Translation vector for rectification
    uint8_t             reserved[24];
};

template<class T>
const T* check_calib(const std::vector<uint8_t>& raw_data)
{
    using namespace std;

    auto table = reinterpret_cast<const T*>(raw_data.data());
    auto header = reinterpret_cast<const table_header*>(raw_data.data());
    if (raw_data.size() < sizeof(table_header))
    {
        debug_err("The data size is too small and does not include the header !!!");
        // throw invalid_value_exception( string::from()
        //                                 << "Calibration data invalid, buffer too small : expected "
        //                                 << sizeof( table_header ) << " , actual: " << raw_data.size() );
    }

    // Make sure the table size does not exceed the actual data we have!
    if( header->table_size + sizeof( table_header ) > raw_data.size() )
    {
        debug_err("Check table size exceeds actual data !!!");
        // throw invalid_value_exception( string::from()
        //                                 << "Calibration table size does not fit inside reply: expected "
        //                                 << ( raw_data.size() - sizeof( table_header ) ) << " but got "
        //                                 << header->table_size );
    }

    // verify the parsed table
    if (table->header.crc32 != calc_crc32(raw_data.data() + sizeof(table_header), raw_data.size() - sizeof(table_header)))
    {
        debug_err("Calibration data CRC error, parsing aborted !!!");
        // throw invalid_value_exception("Calibration data CRC error, parsing aborted!");
    }

    //LOG_DEBUG("Loaded Valid Table: version [mjr.mnr]: 0x" <<
    //    hex << setfill('0') << setw(4) << header->version << dec
    //    << ", type " << header->table_type << ", size " << header->table_size
    //    << ", CRC: " << hex << header->crc32 << dec );
    return table;
}


ds_rect_resolutions width_height_to_ds_rect_resolutions(uint32_t width, uint32_t height);
rs2_intrinsics get_d400_intrinsic_by_resolution_coefficients_table(const std::vector<uint8_t>& raw_data, uint32_t width, uint32_t height);
pose get_d400_color_stream_extrinsic(const std::vector<uint8_t>& raw_data);
rs2_intrinsics get_d400_color_stream_intrinsic(const std::vector<uint8_t>& raw_data, uint32_t width, uint32_t height);
rs2_intrinsics get_d405_color_stream_intrinsic(const std::vector<uint8_t>& raw_data, uint32_t width, uint32_t height);
int parse_realsense_config(const char* json_file_path, RealsenseConfig* config);
bool try_get_d400_intrinsic_by_resolution_new(const std::vector<uint8_t>& raw_data, uint32_t width, uint32_t height, rs2_intrinsics* result);