#include "adaptiveHook.h"

namespace adaptive {

    std::vector<std::array<float, 3>> AdaptiveHook::GenerateAdaptiveHookPath(
        const float cur_x, const float cur_y, const float cur_head,
        const float lidar_x, const float lidar_y, const float vehicle_angle, const float distance, const int pallettype) {
        float angle_temp[2] = {0};
        float board_angle = 0, board_angle_reverse = 0;
        std::array<float, 3> rear_lidar_pos, hook_pos, hook_pos_front;

        if(pallettype == 0)
            angle_temp[0] = vehicle_angle;
        else
            angle_temp[0] = cur_head + 180.0 - atan(lidar_y / lidar_x) * 180.0 / M_PI;

        angle_temp[0] = (angle_temp[0] >= 360.0) ? (angle_temp[0] - 360.0) : (angle_temp[0]);
        angle_temp[0] = (angle_temp[0] < 0.0) ? (angle_temp[0] + 360.0) : (angle_temp[0]);
        rear_lidar_pos = CalculNextPointByAngle(cur_x, cur_y, cur_head, -1.0 * lidar_to_wheel_dis_c);
        hook_pos = CalculNextPointByAngle(rear_lidar_pos[0], rear_lidar_pos[1], angle_temp[0], distance);

        if(pallettype == 0) {
            board_angle = vehicle_angle;
            hook_pos[0] = lidar_x;
            hook_pos[1] = lidar_y;
        } else
            board_angle = cur_head + vehicle_angle;

        board_angle = (board_angle >= 360.0) ? (board_angle - 360.0) : (board_angle);
        board_angle = (board_angle < 0.0) ? (board_angle + 360.0) : (board_angle);
        board_angle_reverse = board_angle + 180.0;
        board_angle_reverse = (board_angle_reverse >= 360.0) ? (board_angle_reverse - 360.0) : (board_angle_reverse);
        angle_temp[1] = cur_head + 180.0;
        angle_temp[1] = (angle_temp[1] >= 360.0) ? (angle_temp[1] - 360.0) : (angle_temp[1]);

        hook_pos_front = CalculNextPointByAngle(hook_pos[0], hook_pos[1], board_angle, 1.0);

        return GenerateBezierPath(cur_x, cur_y, angle_temp[1], hook_pos_front[0], hook_pos_front[1], board_angle_reverse);
    }

    //0到2PI转换为正负PI
    float AdaptiveHook::Trans2PItoPI(const float angle) {
        float angle_temp = angle;

        angle_temp -= 90.0;

        if(angle_temp < 0) angle_temp += 360;

        angle_temp = 360.0 - angle_temp;

        if(angle_temp > 180.0) angle_temp -= 360.0;

        return angle_temp;
    }

    //正负PI转换为2PI
    float AdaptiveHook::TransPIto2PI(const float angle) {
        float angle_temp = angle;

        if(angle_temp < 0) angle_temp += 360.0;

        angle_temp -= 90.0;

        if(angle_temp < 0) angle_temp += 360.0;

        angle_temp = 360.0 - angle_temp;

        return angle_temp;
    }

    std::array<float, 3> AdaptiveHook::CalculNextPointByAngle(
        const float coor_x, const float coor_y,
        const float heading, const float distance) {
        std::array<float, 3> rtn_point;

        float slope;
        float dx, dy;

        slope = tan((90 - heading) / 180 * 3.141592);

        if(heading >= 0.5 && heading <= 179.5) {
            dx = distance / sqrt(1 + slope * slope);
            dy = dx * slope;
        } else if(heading >= 180.5 && heading <= 359.5) {
            dx = -1.0 * distance / sqrt(1 + slope * slope);
            dy = dx * slope;
        } else if(heading > 359.5 || heading < 0.5) {
            dx = 0;
            dy = distance;
        } else if(heading > 179.5 && heading < 180.5) {
            dx = 0;
            dy = -1.0 * distance;
        }

        rtn_point[0] = dx + coor_x;
        rtn_point[1] = dy + coor_y;
        rtn_point[2] = heading;

        return rtn_point;
    }

    std::vector<std::array<float, 3>> AdaptiveHook::GenerateBezierPath(
        const float start_x, const float start_y, const float start_angle,
        const float stop_x, const float stop_y, const float stop_angle) {
        float x[4], y[4];
        float angle_temp = 0;
        float distance = 0;
        int size = 0;
        float detaX, detaY;

        std::array<float, 3> point_temp;
        std::array<float, 3> point_temp1;
        std::vector<std::array<float, 3>> rtn_path;

        x[0] = start_x;
        y[0] = start_y;
        x[3] = stop_x;
        y[3] = stop_y;

        distance = CalculatePoint2PointDistance(start_x, start_y, 0, stop_x, stop_y, 0);
        point_temp = CalculNextPointByAngle(start_x, start_y, start_angle, distance * 0.4);

        x[1] = point_temp[0];
        y[1] = point_temp[1];

        angle_temp = stop_angle + 180;

        if(angle_temp > 360) angle_temp -= 360;

        point_temp = CalculNextPointByAngle(stop_x, stop_y, angle_temp, distance * 0.4);

        x[2] = point_temp[0];
        y[2] = point_temp[1];

        size = (int)(distance / 0.05);

        point_temp[0] = start_x;
        point_temp[1] = start_y;
        point_temp[2] = start_angle;
        rtn_path.push_back(point_temp);

        for(int i = 1; i < size; i++) {
            float t = 0.05 / distance;

            point_temp[0] = x[0] * (1 - i * t) * (1 - i * t) * (1 - i * t) + \
                        3 * x[1] * (i * t) * (1 - i * t) * (1 - i * t) + \
                        3 * x[2] * (i * t) * (i * t) * (1 - i * t) + x[3] * (i * t) * (i * t) * (i * t);
            point_temp[1] = y[0] * (1 - i * t) * (1 - i * t) * (1 - i * t) + \
                        3 * y[1] * (i * t) * (1 - i * t) * (1 - i * t) + \
                        3 * y[2] * (i * t) * (i * t) * (1 - i * t) + y[3] * (i * t) * (i * t) * (i * t);

            point_temp1 = rtn_path.at(rtn_path.size() - 1);

            detaX = point_temp[0] - point_temp1[0];
            detaY = point_temp[1] - point_temp1[1];

            if(detaX > -0.0001 && detaX < 0.0001) {
                if(detaY > 0)
                    angle_temp = 0;
                else
                    angle_temp = 180;
            } else {
                angle_temp = atan((float)detaY / detaX) / M_PI * 180;
                if(detaX > 0)
                    angle_temp = 90 - angle_temp;
                else
                    angle_temp = 270 - angle_temp;
            }

            point_temp[2] = angle_temp;
            rtn_path.push_back(point_temp);
        }

        point_temp[0] = stop_x;
        point_temp[1] = stop_y;
        point_temp[2] = stop_angle;

        rtn_path.push_back(point_temp);

        return rtn_path;
    }

    float AdaptiveHook::CalculatePoint2PointDistance(
        float x1_coor, float y1_coor, float z1_coor,
        float x2_coor, float y2_coor, float z2_coor) {
        float disn = 0;
        float deta_x, deta_y, deta_z;

        deta_x = x1_coor - x2_coor;
        deta_y = y1_coor - y2_coor;
        deta_z = z1_coor - z2_coor;

        disn = sqrt(deta_x * deta_x + deta_y * deta_y + deta_z * deta_z);

        return disn;
    }

}
