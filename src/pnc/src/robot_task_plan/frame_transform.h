#ifndef LOCALIZATION_FRAME_TRANSFORM_H_
#define LOCALIZATION_FRAME_TRANSFORM_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdio>

#define CNOTNULL(ptr)    \
    if(ptr == nullptr) { \
        return;          \
    }

// namespace avos {
// namespace localization {

struct UTMCoor {
    UTMCoor()
        : x(0.0), y(0.0) {
    }
    double x;
    double y;
};

struct WGS84Corr {
    WGS84Corr()
        : log(0.0), lat(0.0) {
    }
    double log;  // longitude
    double lat;  // latitude
};

struct BD09llCorr {
    BD09llCorr()
        : log(0.0), lat(0.0) {
    }
    double log;  // longitude
    double lat;  // latitude
};

void LatlonToUtmXY(const double lon, const double lat, UTMCoor *xy);

void UtmXYToLatlon(const double x, const double y, const int zone,
                   const bool southhemi, WGS84Corr *latlon);

void XYZToBlh(const Eigen::Vector3d &xyz, Eigen::Vector3d *blh);

void BlhToXYZ(const Eigen::Vector3d &blh, Eigen::Vector3d *xyz);

BD09llCorr wgs84tobd09(double lng, double lat);
double transformlat(double lng, double lat);
double transformlng(double lng, double lat);

// }   // namespace localization
// }   // namespace avos

#endif
