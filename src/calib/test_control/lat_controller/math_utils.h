#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <algorithm>
#include <cmath>
#include <vector>

namespace math_utils
{

  /**
   * 将 value 控制在[bound1, bound2]
   */
  template <typename T>
  T clamp(const T value, T bound1, T bound2)
  {
    T result = value;
    if (bound1 > bound2)
    {
      std::swap(bound1, bound2);
    }
    result = result < bound1 ? bound1 : result;
    result = result > bound2 ? bound2 : result;
    return result;
  }

  /**
   * 将角度转为(-pi, pi]
   */
  inline double minusPi2Pi(const double &rad_angle)
  {
    double D_M_PI = 2 * M_PI;
    double angle = fmod(rad_angle, D_M_PI); // 将角度控制在（0，2*pi）
    if (angle > M_PI)
    {
      angle -= D_M_PI;
    }
    else if (angle < -M_PI)
    {
      angle += D_M_PI;
    }
    return angle;
  }

  inline double azimuthToYaw(const double &azimuth)
  {
    // 转为与正东夹角，逆时针为正
    double angle = 90 - azimuth;

    if (angle < -180)
    {
      angle += 360;
    }
    // 转为弧度
    double rad_angle = angle / 180 * M_PI;
    return rad_angle;
  }

  /**
   * 计算两点之间的距离
   */
  template <typename T>
  double distance(const T &p1, const T &p2)
  {
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
  }

  /**
   * 加密线
   */
  template <typename T>
  std::vector<T> denseLineString(const std::vector<T> &ls, const double &interval)
  {
    // printf("dense path, size: %d\n", ls.size());
    if (ls.size() <= 1)
    {
      return ls;
    }
    T last_point = ls.front();
    std::vector<T> result;
    result.push_back(ls.front());
    for (int i = 1; i < ls.size(); ++i)
    {
      const T &next_point = ls.at(i);
      double length = distance(last_point, next_point);
      int count = int(std::ceil(length / interval)) - 1;
      for (int j = 0; j < count; ++j)
      {
        double deta = length / (count + 1) * (j + 1);
        T insert_pt;
        insert_pt.x = last_point.x + deta / length * (next_point.x - last_point.x);
        insert_pt.y = last_point.y + deta / length * (next_point.y - last_point.y);
        result.push_back(insert_pt);
      }
      last_point = next_point;
      result.push_back(next_point);
    }
    // printf("after dense size: %d\n", ls.size());
    return result;
  }

  template <typename T>
  std::vector<T> sparseLineString(const std::vector<T> &ls, const double &interval)
  {
    if (ls.size() <= 2)
    {
      return ls;
    }
    T last_point = ls.front();
    std::vector<T> result = {last_point};
    double accu_l = 0.0;
    for (int i = 1; i < ls.size(); ++i)
    {
      const T next_point = ls.at(i);
      accu_l += distance(last_point, next_point);
      if (accu_l >= interval)
      {
        result.emplace_back(next_point);
        accu_l = 0.0;
      }
      last_point = next_point;
    }
    return result;
  }
  /**
   * 计算航向
   */
  template <typename T>
  double computeHeading(const T &p1, const T &p2)
  {
    double heading_rad = std::atan2(p2.y - p1.y, p2.x - p1.x);
    return heading_rad;
  }
  /**
   * 计算曲率
   */
  template <typename T>
  double computeCurvature(const T &A, const T &B, const T &C)
  {
    // 外接圆面积公式: S = abc/4R
    //  计算两个相邻点之间的距离
    double l_AB = std::sqrt((B.x - A.x) * (B.x - A.x) + (B.y - A.y) * (B.y - A.y));
    double l_BC = std::sqrt((C.x - B.x) * (C.x - B.x) + (C.y - B.y) * (C.y - B.y));
    double l_AC = std::sqrt((C.x - A.x) * (C.x - A.x) + (C.y - A.y) * (C.y - A.y));

    // array_AB = (x2-x1,y2-y1);
    // array_AC = (x3-x1,y3-y1);
    // S = 0.5|array_AB x array_AC|
    // s = 0.5*|(x2-x1)*(y3-y1) - (x3-x1)(y2-y1)|
    double S = 0.5 * (std::abs((B.x - A.x) * (C.y - A.y) - (C.x - A.x) * (B.y - A.y)));
    if (fabs(S) < 1e-6)
    {
      // std::cout << "S: " << (B.x - A.x) * (C.y - A.y) - (C.x - A.x) * (B.y - A.y) << std::endl;
      // std::cout << (B.x - A.x) << " " << (C.y - A.y) << " " << (C.x - A.x) << " " << (B.y - A.y) << std::endl;
      return 0;
    }
    else
    {
      double R = l_AB * l_AC * l_BC / (4 * S);
      // std::cout << "R: " << R << " S: " << S << std::endl;
      double curvature = std::round(100 / R) / 100.f;
      return curvature;
    }
  }

  template <typename T>
  int getNearestIndex(const T &pose, const std::vector<T> &path)
  {
    int min_index = 0;
    double min_dist = 1e9;
    // 1. 查找最近点
    for (size_t i = 0; i < path.size(); ++i)
    {
      auto &ref_pose = path.at(i);
      double dist = distance(pose, ref_pose);
      if (dist > min_dist)
      {
        break;
      }
      // min_dist = dist < min_dist ? dist : min_dist;
      min_dist = dist;
      min_index = i;
    }
    return min_index;
  }

  template <typename T>
  T getFootPose(const T &pose, const std::vector<T> &path)
  {
    if (path.empty())
    {
      return pose;
    }
    // 1. 查找最近索引
    int min_index = getNearestIndex(pose, path);
    // printf("path size:%d, min_index:%d\n", path.size(), min_index);
    //  2. 计算投影点
    int last_idx = min_index - 1;
    int next_idx = min_index + 1;
    if (min_index == 0)
    {
      last_idx = min_index;
    }
    if (min_index == path.size() - 1)
    {
      next_idx = min_index;
    }
    auto &last_pt = path.at(last_idx);
    auto &next_pt = path.at(next_idx);
    // printf("last_idx:%d, next_idx:%d, last_pt: (%f, %f), next_pt: (%f, %f)\n", last_idx, next_idx, last_pt.x, last_pt.y, next_pt.x, next_pt.y);
    //  向量last->next
    double dx_l2n = next_pt.x - last_pt.x;
    double dy_l2n = next_pt.y - last_pt.y;
    // 向量last->p
    double dx_l2p = pose.x - last_pt.x;
    double dy_l2p = pose.y - last_pt.y;
    // 向量last->next的长度
    double len_l2n = sqrt(dx_l2n * dx_l2n + dy_l2n * dy_l2n);
    // 投影长度
    double t = (dx_l2p * dx_l2n + dy_l2p * dy_l2n) / len_l2n;
    // printf("min idx: %d  len_l_p:%f t: %f\n", min_index, len_l2n, t);

    T foot_pose;
    foot_pose.s = last_pt.s + t;
    foot_pose.x = last_pt.x + t * dx_l2n / len_l2n;
    foot_pose.y = last_pt.y + t * dy_l2n / len_l2n;
    foot_pose.heading = computeHeading(last_pt, next_pt);
    foot_pose.v = last_pt.v;
    foot_pose.a = last_pt.a;
    return foot_pose;
  }

  template <typename T>
  void computePoseAttr(std::vector<T> &path)
  {
    double s = 0.0;
    auto last_pt = path.front();
    for (size_t i = 0; i < path.size(); ++i)
    {
      // printf("i:%d\n", i);
      // 三点法计算曲率
      double curvature = 0.0;
      if (path.size() > 2)
      {
        int index1 = i;
        int index2 = i + 1;
        int index3 = i + 2;
        while (index3 >= int(path.size()))
        {
          index1--;
          index2--;
          index3--;
        }
        auto cs_pt1 = path.at(index1);
        auto cs_pt2 = path.at(index2);
        auto cs_pt3 = path.at(index3);
        // 计算曲率
        curvature = math_utils::computeCurvature(cs_pt1, cs_pt2, cs_pt3);
      }

      // 计算航向
      auto current_cs_pt = path.at(i);

      double heading = 0.0;
      if (i < (path.size() - 1))
      {
        auto next_cs_pt = path.at(i + 1);
        heading = math_utils::computeHeading(current_cs_pt, next_cs_pt);
      }
      else
      {
        heading = math_utils::computeHeading(last_pt, current_cs_pt);
      }
      // 计算s
      double delta_s = distance(last_pt, current_cs_pt);
      s += delta_s;
      // printf("i:%d, delta_s: %f   s: %f\n", i, delta_s, s);
      last_pt = current_cs_pt;
      path.at(i).curvature = curvature;
      path.at(i).s = s;
      path.at(i).heading = heading;
    }
  }
}
#endif