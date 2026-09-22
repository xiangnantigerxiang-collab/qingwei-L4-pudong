#ifndef PNC_COMMON_PATH_CSV_H
#define PNC_COMMON_PATH_CSV_H

#include <cerrno>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <limits>
#include <string>

namespace path_csv {

enum ROW_STATUS_E { SKIP_ROW, VALID_ROW, INVALID_ROW };

struct ROW_S {
    // 地图CSV严格四列：x(m)、y(m)、heading(deg)、最高限速(m/s)。
    double values[4] = {0.0, 0.0, 0.0, 10.0};
    std::size_t columns = 0;
};

inline ROW_STATUS_E ParseRow(const std::string &tLine, ROW_S &tRow) {
    const char *cursor = tLine.c_str();
    if(tLine.compare(0, 3, "\xEF\xBB\xBF") == 0) cursor += 3;
    while(std::isspace(static_cast<unsigned char>(*cursor))) ++cursor;
    if(*cursor == '\0' || *cursor == '#') return SKIP_ROW;

    ROW_S row;
    for(;;) {
        if(row.columns == 4) return INVALID_ROW;
        char *end = nullptr;
        errno = 0;
        const double value = std::strtod(cursor, &end);
        if(end == cursor || errno == ERANGE || !std::isfinite(value) ||
           std::abs(value) > std::numeric_limits<float>::max()) return INVALID_ROW;
        row.values[row.columns++] = value;
        cursor = end;
        while(std::isspace(static_cast<unsigned char>(*cursor))) ++cursor;
        if(*cursor == '\0') break;
        if(*cursor != ',') return INVALID_ROW;
        ++cursor;
    }
    if(row.columns != 4 || row.values[3] < 0.0f) return INVALID_ROW;
    tRow = row;
    return VALID_ROW;
}

}  // namespace path_csv

#endif
