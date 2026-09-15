
#ifndef PLANNING_MATH_UTILS_H
#define PLANNING_MATH_UTILS_H

#include <limits>
#include <utility>
#include <vector>

#include "common/surface/vec2d.h"
#include "common/pnc_point/map_point.h"

namespace math {

    double Sqr(const double x);

    /**
 * @brief Cross product between two 2-D vectors from the common start point,
 *        and end at two other points.
 * @param start_point The common start point of two vectors in 2-D.
 * @param end_point_1 The end point of the first vector.
 * @param end_point_2 The end point of the second vector.
 *
 * @return The cross product result.
 */
    double CrossProd(const Vec2d &start_point, const Vec2d &end_point_1,
                     const Vec2d &end_point_2);

    /**
 * @brief Inner product between two 2-D vectors from the common start point,
 *        and end at two other points.
 * @param start_point The common start point of two vectors in 2-D.
 * @param end_point_1 The end point of the first vector.
 * @param end_point_2 The end point of the second vector.
 *
 * @return The inner product result.
 */
    double InnerProd(const Vec2d &start_point, const Vec2d &end_point_1,
                     const Vec2d &end_point_2);

    /**
 * @brief Cross product between two vectors.
 *        One vector is formed by 1st and 2nd parameters of the function.
 *        The other vector is formed by 3rd and 4th parameters of the function.
 * @param x0 The x coordinate of the first vector.
 * @param y0 The y coordinate of the first vector.
 * @param x1 The x coordinate of the second vector.
 * @param y1 The y coordinate of the second vector.
 *
 * @return The cross product result.
 */
    double CrossProd(const double x0, const double y0, const double x1,
                     const double y1);

    /**
 * @brief Inner product between two vectors.
 *        One vector is formed by 1st and 2nd parameters of the function.
 *        The other vector is formed by 3rd and 4th parameters of the function.
 * @param x0 The x coordinate of the first vector.
 * @param y0 The y coordinate of the first vector.
 * @param x1 The x coordinate of the second vector.
 * @param y1 The y coordinate of the second vector.
 *
 * @return The inner product result.
 */
    double InnerProd(const double x0, const double y0, const double x1,
                     const double y1);

    /**
 * @brief Convert degrees to radians
 * @param angle the original value(degree) of the angle.
 * @return The value(radian) of the angle.
 */
    double degreeToRadian(const double angle);

    /**
 * @brief Convert radians to degrees
 * @param angle the original value(radian) of the angle.
 * @return The value(degree) of the angle.
 */
    double radianToDegree(const double radian);

    /**
 * @brief generate a new point by translating the point with distance along the heading
 * @param point origin point.
 * @param theta the translation's heading
 * @param length the tanslation's distance
 * @return The value(degree) of the angle.
 */
    Vec2d translationPoint(const Vec2d point, const double theta,
                           const double length);
    /**
 * @brief Wrap angle to [0, 2 * PI).
 * @param angle the original value of the angle.
 * @return The wrapped value of the angle.
 */
    double WrapAngle(const double angle);

    /**
 * @brief Normalize angle to [-PI, PI).
 * @param angle the original value of the angle.
 * @return The normalized value of the angle.
 */
    double NormalizeAngle(const double angle);

    /**
 * @brief Calculate the difference between angle from and to
 * @param from the start angle
 * @param from the end angle
 * @return The difference between from and to. The range is between [0, PI).
 */
    double AngleDiff(const double from, const double to);

    /**
 * @brief Calculate the point angle
 * @param x point x axis
 * @param y point y axis
 * @return angel from y axis(0 to 2Pi Clockwise direction),.
 */
    double getAngle(const double x, const double y);

    /**
 * @brief Get a random integer between two integer values by a random seed.
 * @param s The lower bound of the random integer.
 * @param t The upper bound of the random integer.
 * @param random_seed The random seed.
 * @return A random integer between s and t based on the input random_seed.
 */
    int RandomInt(const int s, const int t, unsigned int rand_seed = 1);

    /**
 * @brief Get a random double between two integer values by a random seed.
 * @param s The lower bound of the random double.
 * @param t The upper bound of the random double.
 * @param random_seed The random seed.
 * @return A random double between s and t based on the input random_seed.
 */
    double RandomDouble(const double s, const double t, unsigned int rand_seed = 1);

    /**
 * @brief Compute squared value.
 * @param value The target value to get its squared value.
 * @return Squared value of the input value.
 */
    template <typename T>
    inline T Square(const T value) {
        return value * value;
    }

    template <typename T>
    void uniform_slice(const T start, const T end, uint32_t num,
                       std::vector<T> *sliced) {
        if(!sliced) {
            return;
        }
        if(num == 0) {
            sliced->resize(num + 1);
            sliced->at(num) = start;
            return;
        }
        const T delta = (end - start) / num;
        sliced->resize(num + 1);
        T s = start;
        for(uint32_t i = 0; i < num; ++i, s += delta) {
            sliced->at(i) = s;
        }
        sliced->at(num) = end;
    }
    /**
 * @brief search min value index.
 * @param value The target value to get its squared value.
 * @return Squared value of the input value.
 */
    template <typename T>
    bool getMinValueDoubleVector(const std::vector<std::vector<T>> &doubleValues,
                                 int &i,
                                 int &j) {
        if(doubleValues.empty()) {
            return false;
        }
        T minValue = std::numeric_limits<T>::max();
        int index1 = -1;
        for(auto values : doubleValues) {
            index1++;
            int index2 = -1;
            for(auto value : values) {
                index2++;
                if(value < minValue) {
                    minValue = value;
                    i = index1;
                    j = index2;
                }
            }
        }
        if(i == -1 || j == -1) {
            return false;
        }
        return true;
    }
    /**
 * @brief calculate acc by speed of change and distance
 * @param startSpeed initSpeed,
 * @param endSpeed endSpeed
 * @param distance distance forward
 * @param acc the car which uniform acceleration
 */
    double accBySpeedOfChange(const double startSpeed,
                              const double endSpeed,
                              const double distance);

    /**
 * @brief Convert absolute coordinates to relative coordinates FLU
 * @param originPoint The origin of coordinates
 * @param goalPoint need to convert point
 * @param relativePoint relative coordinate's point.
 */
    void gussPointToFLU(const had_map::MapPoint &originPoint,
                        const had_map::MapPoint &goalPoint,
                        had_map::MapPoint &relativePoint);

    /**
 * @brief Convert absolute coordinates to relative coordinates RFU
 * @param originPoint The origin of coordinates
 * @param goalPoint need to convert point
 * @param relativePoint relative coordinate's point.
 */
    void gussPointToRFU(const had_map::MapPoint &originPoint,
                        const had_map::MapPoint &goalPoint,
                        had_map::MapPoint &relativePoint);

    /**
 * @brief Convert absolute coordinates to relative coordinates RFU
 * @param originPoint The origin of coordinates
 * @param goalPoint need to convert point
 * @param relativePoint relative coordinate's point.
 */
    void gussPointToRFU2(const had_map::MapPoint &originPoint,
                         const math::Vec2d &goalPoint,
                         math::Vec2d &relativePoint);

    /**
 * @brief Convert relative coordinates RFU to absolute coordinates
 * @param originPoint The origin of coordinates
 * @param goalPoint need to convert point
 * @param relativePoint guss coordinate's point.
 */
    void RFUPointToGuss(const had_map::MapPoint &originPoint,
                        const had_map::MapPoint &goalPoint,
                        had_map::MapPoint &relativePoint);

    /**
 * @brief Convert relative coordinates FLU to absolute coordinates
 * @param originPoint The origin of coordinates
 * @param goalPoint need to convert point
 * @param relativePoint guss coordinate's point.
 */
    void FLUPointToGuss(const had_map::MapPoint &originPoint,
                        const had_map::MapPoint &goalPoint,
                        had_map::MapPoint &relativePoint);

    /**
 * @brief Clamp a value between two bounds.
 *        If the value goes beyond the bounds, return one of the bounds,
 *        otherwise, return the original value.
 * @param value The original value to be clamped.
 * @param bound1 One bound to clamp the value.
 * @param bound2 The other bound to clamp the value.
 * @return The clamped value.
 */
    template <typename T>
    T Clamp(const T value, T bound1, T bound2) {
        if(bound1 > bound2) {
            std::swap(bound1, bound2);
        }

        if(value < bound1) {
            return bound1;
        } else if(value > bound2) {
            return bound2;
        }
        return value;
    }

    // Gaussian
    double Gaussian(const double u, const double std, const double x);

    // Sigmoid
    double Sigmoid(const double x);

    double Sigmoid2(const double x);

    // Rotate Axis (2D):
    // convert a point (x0, y0) in axis1 to a point (x1, y1) in axis2 where the
    // angle from axis1 to axis2 is theta (counterclockwise)
    //注意：坐标系的旋转不是点的旋转
    void RotateAxis(const double theta, const double x0, const double y0,
                    double *x1, double *y1);

    double calculateKByTwoPoint(const had_map::MapPoint &p1, const had_map::MapPoint &p2);

    inline std::pair<double, double> RFUToFLU(const double x, const double y) {
        return std::make_pair(y, -x);
    }

    inline std::pair<double, double> FLUToRFU(const double x, const double y) {
        return std::make_pair(-y, x);
    }

    inline void L2Norm(int feat_dim, float *feat_data) {
        if(feat_dim == 0) {
            return;
        }
        // feature normalization
        float l2norm = 0.0;
        for(int i = 0; i < feat_dim; ++i) {
            l2norm += feat_data[i] * feat_data[i];
        }
        if(l2norm == 0) {
            float val = 1.0 / std::sqrt(feat_dim);
            for(int i = 0; i < feat_dim; ++i) {
                feat_data[i] = val;
            }
        } else {
            l2norm = std::sqrt(l2norm);
            for(int i = 0; i < feat_dim; ++i) {
                feat_data[i] /= l2norm;
            }
        }
    }

}  // namespace math
#endif  //PLANNING_MATH_UTILS_H
