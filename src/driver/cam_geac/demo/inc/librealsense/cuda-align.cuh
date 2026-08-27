#pragma once

#include <memory>
#include "rs_sensor.h"
#include <stdint.h>
#include "float3.h"

void align_depth_to_other(unsigned char* h_aligned_out, const uint16_t* h_depth_in,
    float depth_scale, const rs2_intrinsics& h_depth_intrin, const rs2_extrinsics& h_depth_to_other,
    const rs2_intrinsics& h_other_intrin);




