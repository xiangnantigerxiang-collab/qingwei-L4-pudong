#pragma once

#include <vector>
#include "common/struct_type.h"
#include "reference_line/reference_point.h"
#include "reference_line/reference_line.h"

namespace planning {
struct GaussData
{
    uint index;
    double x;
    double y;
    double heading;
    int type;
    double s;
    double z;
};

struct dPoint
{
    double x;
    double y;
};

class TransData
{
public:
    TransData() = default;
    ~TransData() = default;

    bool createReferenceLine(
             ReferenceLine &referenceLine,
             std::vector<OriginalInsData> &originpoint);

private:
    bool ImportData(std::vector<GaussData>& raw_reference_line,
                    std::vector<OriginalInsData> &originpoint);
};

} //namespace planning

