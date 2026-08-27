#include "../inc/librealsense/realsense_camera_parameter.h"
// #include "../inc/librealsense/realsense_camera_align_cuda.cu"
#include "cuda-align.cuh"
#include <omp.h>
#include <vector>
#include <cstdint>
#include <atomic>

class CMgrCameraAlign
{
public:
	CMgrCameraAlign();
	~CMgrCameraAlign();
    bool getparameter();
    bool setDepthResolution(uint32_t width, uint32_t height);
    bool setRgbResolution(uint32_t width, uint32_t height);
    bool inputFrame(int nChan, unsigned char *pData, int nDatalen, uint8_t * outData, int *out_nDatalen);
private:
	int	m_dwChan;
	int	depth_dwWidth;
	int	depth_dwHeight;
    int	rgb_dwWidth;
	int	rgb_dwHeight;
    int	rgb_len;
    int	depth_len;
    int	align_depth_to_color;
    int align_color_to_depth;
    int realsense_device;
    int high_accuracy_mode;
    std::vector<uint8_t> color_raw_data;
    std::vector<uint8_t> depth_raw_data;
    rs2_intrinsics depth_intrinsics;
    rs2_intrinsics color_intrinsics;
    pose color_extrinsics;
};
