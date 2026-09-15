#ifndef GPS_SPLANE_H
#define GPS_SPLANE_H

#include <typeinfo>
#include <math.h>
#include "Eigen/Eigen"
#include "robot_path_plan/common/pubalgor/pubalgor.h"

//serial::Serial ser;
using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::MatrixXd;

typedef struct gps_ch {
    uint8_t ch[1024];
    int position;
} GPS_ARRAY;

typedef struct xyz_utm_s {
    double x_axis;
    double y_axis;
    double z_axis;
    double heading;
} XYZ_UTM_S;

struct SevenParams {
    double pitch;
    double roll;
    double yaw;

    double trans_x;
    double trans_y;
    double trans_z;

    double scale;
};

const double deg2rad = 1.0 / 180.0 * M_PI;
const double sm_a = 6378137.0;
const double sm_b = 6356752.31425;
const double sm_EccSquared = 6.69437999013e-03;
const double UTMScaleFactor = 0.9996;
const double e_2 = 0.00669437999013;
const double earth_a = 6378137.0;
const double earth_b = 6356752.3142;
const double earth_e = 8.1819190842622e-2;
const double earth_f = 0.00335281066474;
const double earth_gm = 3986004.418e8;
const double ligth_speed = 299792458;

class Gps2plane {
public:
    Gps2plane();
    ~Gps2plane() {
    }

    void SetReferPosition(
        const double longitude,
        const double latitude,
        const double height);

    XYZ_UTM_S Latlon2Utmxy(double longitude, double lattitude);
    XYZ_UTM_S Transform_point_new(XYZ_UTM_S& point);
    XYZ_UTM_S Latlon2Local(double longitude, double latitude, double height);

private:
    double Utm_central_meridian(int zone);
    double Arclength0f_meridian(double phi);

    void Maplatlon_to_xy(double phi, double lambda,
                         double lambda0, XYZ_UTM_S& xy);

    Matrix3d Skew_sys(double x, double y, double z);
    Vector3d Scale_point(double x, double y, double z);
    MatrixXd One_point_coif(double x, double y, double z);
    MatrixXd Coef_matrix(const SevenParams& params);

    double mReferLon;
    double mReferLat;
    double mReferHeight;
};

#endif
