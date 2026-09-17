// 控制编排器 ControlComply 实现：路径预处理、电子围栏、VehicleControl
// 主流程（横纵向调度）、Stanley/R 挡几何入口和控制消息输出。
// ACC/AEB 纵向控制实现位于 longitudinal_acc_control.inc。

#include "control_comply.h"

ControlComply::ControlComply() {
    mPathList.clear();
    mGear = GEAR_N;
    mSpeed = 0;
    mKeyPoint = 0;
    mTlStatus.light_status = 2;
}

ControlComply::~ControlComply() {
}

// ---------------------------------------------------------------------------
// ROS 消息输入
// ---------------------------------------------------------------------------

void ControlComply::SetPathStatusData(robot::path_plan_status path_status_t) {
    mPathStatus = path_status_t;
}

void ControlComply::setTaskPlanData(robot::task_plan_msg task_info) {
    mTaskInfo = task_info;
}

void ControlComply::SetCanData(robot::can_msg can_msg_t) {
    mGear = can_msg_t.curGear;
    mVehicleSpeed = can_msg_t.vehicleSpeed;
    mSteerAngle = -can_msg_t.wheelAngle / 22.0 / 180 * 3.1415926;
}

void ControlComply::SetNavigationData(robot::navigation_msg navigation_t) {
    mNavData = navigation_t;
    ego_pose2d.x = mNavData.xAxis;
    ego_pose2d.y = mNavData.yAxis;
    ego_pose2d.heading = azimuthToYaw(mNavData.heading);
}

void ControlComply::SetPathPlanData(robot::path_plan_msg path_plan_t) {
    XYZ_COOR_S xyz_temp;
    std::vector<XYZ_COOR_S> src_path;

    if(path_plan_t.x.size() != path_plan_t.y.size()) return;

    src_path.clear();

    mSpeed = path_plan_t.desireSpeed;
    mPathid = path_plan_t.Path_Id;
    mPathsafety = path_plan_t.safety;

    robot::path_plan_msg path_recv = path_plan_t;
    // 反转路径顺序
    std::reverse(path_recv.x.begin(), path_recv.x.end());
    std::reverse(path_recv.y.begin(), path_recv.y.end());

    if(path_recv.x.size() < 10) return;

    robot::path_plan_msg path_filter;
    int psize = path_recv.x.size();

    for(int i = 0; i < psize; i++) {
        if(i == 0) {
            path_filter.x.push_back(path_recv.x[i]);
            path_filter.y.push_back(path_recv.y[i]);
            continue;
        }

        int fsize = path_filter.x.size();

        double x0 = path_filter.x[fsize - 1];
        double y0 = path_filter.y[fsize - 1];
        double x1 = path_recv.x[i];
        double y1 = path_recv.y[i];
        // 与路径起点(因为路径反转了)的距离
        double len = hypot(x1 - x0, y1 - y0);

        // 取与路径终点的距离大于0.1m或者小于3.0m的点
        if(len > 0.1 && len < 3.0) {
            path_filter.x.push_back(path_recv.x[i]);
            path_filter.y.push_back(path_recv.y[i]);
        }
    }

    std::reverse(path_filter.x.begin(), path_filter.x.end());
    std::reverse(path_filter.y.begin(), path_filter.y.end());
    path_plan_t = path_filter;

    if(path_plan_t.x.size() < 5) return;

    if(mPathid == 20) {
        CSpline spline;
        for(int i = 0; i < path_plan_t.x.size(); ++i) {
            xyz_temp.x_axis = path_plan_t.x[i];
            xyz_temp.y_axis = path_plan_t.y[i];
            src_path.push_back(xyz_temp);
        }

        mPathList.clear();
        mKeyPoint = 0;
        spline.SplinePointSet(src_path, mPathList, 0.1);
    } else {
        CSpline spline;

        for(int i = 0; i < path_plan_t.x.size(); ++i) {
            xyz_temp.x_axis = path_plan_t.x[i];
            xyz_temp.y_axis = path_plan_t.y[i];
            src_path.push_back(xyz_temp);
        }

        mPathList.clear();
        mKeyPoint = 0;
        spline.SplinePointSet(src_path, mPathList, 0.1);
    }

    CalcuPathHead(mPathList);
    CalcuPathCurve(mPathList);

    mControlData.bypassProcessing = path_plan_t.bypassProcessing;
}

bool ControlComply::IsGreenLight(uint8_t light_state) {
    static uint8_t light_buffer[5] = {0};

    for(int i = 0; i < 4; ++i)
        light_buffer[i] = light_buffer[i + 1];

    light_buffer[4] = light_state;

    for(int i = 0; i < 5; ++i) {
        if(light_buffer[i] != 2) return false;
    }

    return true;
}

void ControlComply::SetTlStatusData(robot::TLStatus tl_status) {
    mTlStatus = tl_status;
}

// ---------------------------------------------------------------------------
// 路径与电子围栏
// ---------------------------------------------------------------------------

void ControlComply::LoadPathFile(std::string tPath) {
    std::vector<XYZ_COOR_S> vector_list;
    std::vector<XYZ_COOR_S> incsv;
    float distance_temp = 0;
    std::string file_dir = "";
    ros::param::get("path_dir", file_dir);
    XYZ_COOR_S intp;

    FILE* fp;
    std::string path_dir = file_dir + tPath + ".csv";
    fp = fopen(path_dir.c_str(), "r");

    while(!feof(fp)) {
        fscanf(fp, "%f,%f,%f,%f", &intp.x_axis, &intp.y_axis, &intp.heading,
               &intp.z_axis);
        incsv.push_back(intp);
    }

    for(auto i : incsv) {
        XYZ_COOR_S xyz_temp;
        xyz_temp.heading = i.heading;
        xyz_temp.x_axis = i.x_axis;
        xyz_temp.y_axis = i.y_axis;
        xyz_temp.z_axis = i.z_axis;
        xyz_temp.velocity = 5;
        xyz_temp.p2pDistance = 0.04;
        distance_temp += xyz_temp.p2pDistance;
        xyz_temp.dist_origin = distance_temp;
        vector_list.push_back(xyz_temp);
    }

    mFenceList = vector_list;
}

int pointInPolygon(const std::vector<XYZ_COOR_S>& poly, const XYZ_COOR_S& p) {
    const double EPS = 1e-9;

    int n = poly.size(), inside = 0;

    double px = p.x_axis;
    double py = p.y_axis;

    for(int i = 0, j = n - 1; i < n; j = i++) {
        const auto &a = poly[j], &b = poly[i];

        double ax = a.x_axis;
        double ay = a.y_axis;
        double bx = b.x_axis;
        double by = b.y_axis;

        double cross = (bx - ax) * (py - ay) - (by - ay) * (px - ax);

        if(std::abs(cross) < EPS && px >= std::min(ax, bx) - EPS &&
           px <= std::max(ax, bx) + EPS && py >= std::min(ay, by) - EPS &&
           py <= std::max(ay, by) + EPS)
            return 2;

        if((ay > py) != (by > py) && px < (bx - ax) * (py - ay) / (by - ay) + ax)
            inside ^= 1;
    }

    return inside;
}

XYZ_COOR_S ControlComply::local2global2(double ox, double oy, double oheading,
                                        double lx, double ly, double lheading) {
    double h = 90.0 - oheading;

    if(h > 360.0) h -= 360.0;
    if(h < 0.0) h += 360.0;

    double rad = h * M_PI / 180.0;
    double cosd = cos(rad);
    double sind = sin(rad);
    // rotate
    double dx = cosd * lx - sind * ly;
    double dy = sind * lx + cosd * ly;
    // translation
    XYZ_COOR_S point;
    point.x_axis = dx + ox;
    point.y_axis = dy + oy;
    point.heading = lheading + oheading;

    return point;
}

void ControlComply::FenceAlarm() {
    if(mTaskInfo.taskType == ADAPTIVEHOOK) {  // hook循迹时不检测电子围栏
        ros::param::set("/alarmcmd", 0);
        FenceWarning = 0;
        return;
    }

    int n = mFenceList.size();

    if(n < 2) return;

    double x = mNavData.xAxis;
    double y = mNavData.yAxis;
    double h = mNavData.heading;
    auto point0 = local2global2(x, y, h, 2.3, 0.73, 0);   // 左前角
    auto point1 = local2global2(x, y, h, 2.3, -0.73, 0);  // 右前角

    int check_left = pointInPolygon(mFenceList, point0);
    int check_right = pointInPolygon(mFenceList, point1);

    if(check_left != 1 || check_right != 1) {
        ros::param::set("alarmcmd", 1);
        FenceWarning = 1;
    } else {
        ros::param::set("alarmcmd", 0);
        FenceWarning = 0;
    }
}

// ---------------------------------------------------------------------------
// 20 Hz 控制主流程
//
// 执行顺序：停车门控 -> 位姿/纵向基础量 -> 横向控制 -> 速度约束 ->
// 传感器与围栏安全覆盖 -> 输出限幅。顺序会影响最终控制量，请勿随意调整。
// ---------------------------------------------------------------------------

void ControlComply::VehicleControl() {
    if(mGear != GEAR_D) ResetLaunchSpeed();

    if(fabs(mSpeed) < 0.1 && mPathsafety) {
        printf("mPathsafety:%d\n", mPathsafety);
        mControlData.desireSpeed = 0;
        mControlData.desireAcc = 0;
        mControlData.throttlePercent = 0;
        mControlData.brakePercent = 100;
        mControlData.wheelAngle = 0;
        ResetLaunchSpeed();
        return;
    }

    if((fabs(mSpeed) < 0.5 && mPathStatus.taskExecuStatus == 2) ||
       mGear == GEAR_N) {
        printf("mGear:%d, execute stop:%d\n", mGear,
               mPathStatus.taskExecuStatus);
        mControlData.desireSpeed = 0;
        mControlData.desireAcc = 0;
        mControlData.throttlePercent = 0;
        mControlData.brakePercent = 80;
        mControlData.wheelAngle = 0;
        ResetLaunchSpeed();
        return;
    }

    if((!IsGreenLight(mTlStatus.light_status)) && mNavData.heading > 80.0 &&
       mNavData.heading < 100.0 && mNavData.xAxis > 1553.0 &&
       mNavData.xAxis < 1559.0) {
        mControlData.desireSpeed = 0;
        mControlData.desireAcc = 0;
        mControlData.throttlePercent = 0;
        mControlData.brakePercent = 30;
        mControlData.wheelAngle = 0;
        ResetLaunchSpeed();
        return;
    }

    if((!IsGreenLight(mTlStatus.light_status)) && mNavData.heading > 80.0 &&
       mNavData.heading < 100.0 && mNavData.xAxis > 425.0 &&
       mNavData.xAxis < 432.0) {
        mControlData.desireSpeed = 0;
        mControlData.desireAcc = 0;
        mControlData.throttlePercent = 0;
        mControlData.brakePercent = 50;
        mControlData.wheelAngle = 0;
        ResetLaunchSpeed();
        return;
    }
    if(mPathList.empty()) {
        printf("空路径\n");
        mControlData.desireSpeed = 0;
        mControlData.desireAcc = 0;
        mControlData.throttlePercent = 0;
        mControlData.brakePercent = 50;
        mControlData.wheelAngle = 0;
        ResetLaunchSpeed();
        return;
    }
    // 这里计算了距离车辆最近点的曲率和横向误差赋值给了mControlData
    // (但是这里并没有用到预瞄参数)
    VehiclePoseCalculation();
    // 这里的mSpeed是通过path_plan_msg传入的期望速度
    mControlData.desireSpeed = mSpeed;
    // 这里基于期望速度和当前速度的差值调用模糊pid算法计算了期望加速度
    mControlData.desireAcc = spCtr_c.AccelerationCalculateBySpeed_P(
        mControlData.desireSpeed, mVehicleSpeed);

    ros::param::get("/robot/control/accswitch", AccSwitch);

    acc_msg.active = 0;
    acc_msg.object = 0;
    acc_msg.ds = 0;
    acc_msg.dv = 0;
    acc_msg.tarspd = 0;
    acc_msg.curspd = 0;
    acc_msg.objspd = 0;
    acc_msg.Ades = 0;
    acc_msg.Av = 0;
    acc_msg.Bd = 0;
    acc_msg.Aaeb = 0;
    acc_msg.Aout = 0;
    // 自适应巡航
    if(AccSwitch) {
        LongitudinalControlOutput(acc_msg);
    }
    // 这里根据期望速度和期望加速度计算了油门和刹车的百分比
    // (但是这里并没有用到期望加速度)，被后面代码覆盖，未使用
    VehicleVerticalControl(mControlData.desireSpeed, mNavData.gpsSpeed,
                           mControlData.desireAcc, mControlData.throttlePercent,
                           mControlData.brakePercent);

    if(mPathid == 5 && mPathsafety) {
        mControlData.throttlePercent = 0;
        mControlData.brakePercent = 100;
        ResetLaunchSpeed();
    } else {
        // 横向控制，输出转向角
        if(mGear == GEAR_R) {
            mControlData.wheelAngle = VehicleLateralControl();
        } else {
            mControlData.wheelAngle = VehicleStanleyControl();
        }
        //
        {
            auto path2d = toPath2d(mPathList);
            auto foot_pose = math_utils::getFootPose(ego_pose2d, path2d);
            double lat_dev = ego_pose2d.distanceTo(foot_pose);
            double angle_dev = fabs(ego_pose2d.heading - foot_pose.heading);
            mControlData.biaDistance = lat_dev;
            mControlData.biaAngle = angle_dev;
        }

        // 处理指令边界
        double speed_now = mVehicleSpeed;
        double speed_cmd = mSpeed;  // 期望速度，路径发下来的
        double delta = speed_cmd - speed_now;

        if(delta > 0.5) speed_cmd = speed_now + 0.5;

        double curvelimitspeed = CurveLimitSpeed(mPathList);
        printf("curvelimitspeed:%f, mSpeed:%f, speed_cmd:%f, speed_node:%f\n",
               curvelimitspeed, mSpeed, speed_cmd, speed_now);
        speed_cmd = std::min<double>(speed_cmd, curvelimitspeed);

        //////////////////
        {
            // 缓停逻辑
            int current_state = 0;
            ros::param::get("/planning/sensorstate", current_state);
            if((current_state & 0x02) == 2 || (current_state & 0x04) == 4) {
                printf("huanman tingche\n");
                // 减速停车
                double dcc = 8.0;
                double dcc_speed = speed_now - dcc * 0.05;
                speed_cmd = std::min<double>(speed_cmd, dcc_speed);
                speed_cmd = std::max<double>(speed_cmd, 0.0);
                printf("slow down to stop, speed_cmd:%f\n", speed_cmd);
            }
            // 急刹车
            if((current_state & 0x08) == 8) {  // gnss故障，急停
                printf("gnss error, jiting\n");
                speed_cmd = 0.0;
                mControlData.brakePercent = 100;
            } else {
                if(delta < -1.5) {
                    if(mControlData.brakePercent < 5) {
                        mControlData.brakePercent += 1;
                    }
                } else {
                    mControlData.brakePercent = 0;
                }
            }
        }

        printf("speed_cmd: %f, speed_now: %f\n", speed_cmd, speed_now);
        if(mGear == GEAR_D) {
            // 底层是速度环,保留速度到油门的标定比例。原 5% 保底先作为
            // 目标速度下限,再走速度斜坡,避免起步直接跳到 5%。
            if(speed_cmd - speed_now > 0.05 && speed_cmd * 18.0 < 5.0) {
                speed_cmd = 5.0 / 18.0;
            }
            speed_cmd = SmoothLaunchSpeed(speed_cmd);
        }
        mControlData.throttlePercent = speed_cmd * 18.0;
        if(mGear != GEAR_D && speed_cmd - speed_now > 0.05) {
            mControlData.throttlePercent =
                mControlData.throttlePercent < 5 ? 5 : mControlData.throttlePercent;
        }

        if(FenceWarning == 1) {  // 冲出电子围栏
            printf("冲出电子围栏, 停车\n");
            mControlData.brakePercent = 70.0;
            mControlData.throttlePercent = 0;
            ResetLaunchSpeed();
        }
        // 冲出跑道
        if(mControlData.biaDistance > 5.5 ||
           (mControlData.biaDistance > 4.0 && mControlData.biaAngle > 0.3)) {
            printf("横向偏差 > 2.5m, 停车\n");
            mControlData.brakePercent = 70;
            mControlData.throttlePercent = 0;
            ResetLaunchSpeed();
        }

        std::cout << "act = " << FenceWarning << " brake = "
                  << (int)mControlData.brakePercent << std::endl;

        // 处理边界条件
        if(mControlData.brakePercent > 100) mControlData.brakePercent = 100;
        if(mControlData.throttlePercent > 100) mControlData.throttlePercent = 100;
        // 处理倒挡
        if(mGear == GEAR_R) {
            mControlData.throttlePercent = mSpeed * 10;
            if(mControlData.throttlePercent > 45) mControlData.throttlePercent = 45;
        }
    }
}

double ControlComply::SmoothLaunchSpeed(double speed_cmd) {
    if(speed_cmd <= 0.0) {
        ResetLaunchSpeed();
        return 0.0;
    }

    double slope = 0.3;  // 速度给定上升斜率(m/s^2),20Hz 下每周期增加 0.015m/s
    ros::param::get("/robot/control/launch_speed_slope", slope);
    if(!std::isfinite(slope) || slope <= 0.0) slope = 0.3;

    if(!mLaunchSpeedInitialized) {
        // 行驶中首次接入控制时从当前车速衔接;明确停车后由 Reset 从零起步。
        mLaunchSpeedCmd = std::isfinite(mVehicleSpeed)
            ? std::max(0.0, static_cast<double>(mVehicleSpeed)) : 0.0;
        mLaunchSpeedInitialized = true;
    }
    // 只限制上升。下降立即跟随,小数状态一直保留到最后的消息赋值。
    mLaunchSpeedCmd = std::min(speed_cmd, mLaunchSpeedCmd + slope * 0.05);
    return mLaunchSpeedCmd;
}

void ControlComply::ResetLaunchSpeed() {
    mLaunchSpeedCmd = 0.0;
    mLaunchSpeedInitialized = true;
}

// ---------------------------------------------------------------------------
// 控制消息输出
// ---------------------------------------------------------------------------

void ControlComply::PublishMessage(ros::Publisher& tPub) {
    if(mControlData.throttlePercent > 0 && mControlData.brakePercent > 0) {
        robot::control_msg throttle_msg = mControlData;
        robot::control_msg brake_msg = mControlData;
        throttle_msg.brakePercent = 0;
        brake_msg.throttlePercent = 0;
        tPub.publish(throttle_msg);
        tPub.publish(brake_msg);
    } else {
        tPub.publish(mControlData);
    }
}

float ControlComply::CurveLimitSpeed(std::vector<XYZ_COOR_S> pathlist) {
    static std::vector<double> last_curvatures(30, 0);
    int n = pathlist.size();
    int batch = 60;

    std::vector<double> curvatures;

    if(n < batch) {
        for(int i = 0; i < n; i++)
            curvatures.push_back(pathlist[i].curvature);
        for(int i = n; i < batch; i++)
            curvatures.push_back(0);
    } else {
        for(int i = 0; i < batch; i++)
            curvatures.push_back(pathlist[i].curvature);
    }

    double bias = last_curvatures.back() - curvatures.front();
    printf("bias:%f\n", bias);
    if(fabs(bias) > 0.001) {
        last_curvatures.erase(last_curvatures.begin());
        last_curvatures.push_back(curvatures.front());
    }

    double speed = 0.0;
    for(auto c : last_curvatures) {
        speed += c;
    }
    int num = 0;
    for(auto i : curvatures) {
        if(num++ < 30) speed += i;
    }
    printf("====sum curvature:%f, batch:%d\n", speed, batch);
    speed = sqrt(speed / (double)(batch));
    if(speed < 0.001) speed = 0.001;
    printf("8888speed: %f\n", speed);
    double final_speed = 0.3 / speed;
    printf("final speed: %f\n", final_speed);
    return final_speed;
}

// ---------------------------------------------------------------------------
// 路径几何与车辆相对位姿
// ---------------------------------------------------------------------------

void ControlComply::CalcuPathCurve(vector<XYZ_COOR_S>& path_list) {
    int size = path_list.size();
    float curve_temp = 0;

    if(size <= 1) return;

    for(int i = 0; i < size - 2; ++i) {
        if(i < size - 12) {
            float iTemp = 0;
            int count = 0;
            float length = 0;

            for(int j = i; j < i + 11; ++j) {
                float sub = path_list.at(j + 1).heading - path_list.at(j).heading;

                if(sub < 0) count++;

                iTemp += sub;
                length += 0.1;
            }

            if(iTemp > 180) iTemp -= 360;
            if(iTemp < -180) iTemp += 360;

            curve_temp = iTemp * M_PI / 180 / length;

            if(curve_temp < 0) curve_temp = -1 * curve_temp;

            path_list.at(i).curvature = curve_temp;
        }
    }

    for(int i = size - 1; i > size - 13; i--)
        path_list.at(i).curvature = path_list.at(size - 13).curvature;
}

void ControlComply::CalcuPathHead(vector<XYZ_COOR_S>& path_list) {
    int size = path_list.size();
    XYZ_COOR_S xyz_array[2];
    float angle_temp = 0;

    if(size == 0) return;

    for(int i = 0; i < size - 1; ++i) {
        xyz_array[0] = path_list.at(i);
        xyz_array[1] = path_list.at(i + 1);
        angle_temp = pubalgor.CalculatePoint2PointAngle_P(
            xyz_array[0].x_axis, xyz_array[0].y_axis, xyz_array[1].x_axis,
            xyz_array[1].y_axis);

        path_list.at(i).heading = angle_temp;
        path_list.at(i).z_axis = 0;
        path_list.at(i).p2pDistance = 0.1;
        path_list.at(i).velocity = 5;
    }

    path_list.at(size - 1).heading = path_list.at(size - 2).heading;
    path_list.at(size - 1).z_axis = 0;
    path_list.at(size - 1).p2pDistance = 0.1;
    path_list.at(size - 1).velocity = 5;
}

void ControlComply::BiaAngleCalculate(
    std::vector<XYZ_COOR_S> path_list, CONTROL_PARAM_IN para_in,
    robot::control_msg& para_out) {
    if(path_list.size() < 3) {
        return;
    }
    float distance_temp;
    int new_key_point = 0;
    XYZ_COOR_S xyz_temp;
    float delta_x[2], delta_y[2];
    float min_distance = 100;
    int size = path_list.size();
    float cur_x = para_in.cur_position.x_axis;
    float cur_y = para_in.cur_position.y_axis;
    float cur_head = para_in.cur_position.heading;

    for(int i = 0; i < size; i++) {
        xyz_temp = path_list.at(i);
        distance_temp =
            sqrt((xyz_temp.x_axis - cur_x) * (xyz_temp.x_axis - cur_x) +
                 (xyz_temp.y_axis - cur_y) * (xyz_temp.y_axis - cur_y));

        if(min_distance > distance_temp) {
            min_distance = distance_temp;
            new_key_point = i % size;
        }
    }

    mKeyPoint = new_key_point;
    para_out.preCurve = path_list.at(mKeyPoint).curvature;

    if(path_list.at(path_list.size() - 3).curvature > para_out.preCurve)
        para_out.preCurve = path_list.at(path_list.size() - 3).curvature;

    delta_x[0] = cur_x - path_list.at(new_key_point).x_axis;
    delta_y[0] = cur_y - path_list.at(new_key_point).y_axis;
    delta_x[1] = path_list.at((new_key_point + 2) % size).x_axis -
                 path_list.at(new_key_point).x_axis;
    delta_y[1] = path_list.at((new_key_point + 2) % size).y_axis -
                 path_list.at(new_key_point).y_axis;

    distance_temp = delta_x[1] * delta_y[0] - delta_y[1] * delta_x[0];

    if(distance_temp > 0)
        para_out.biaDistance =
            sqrtf(delta_x[0] * delta_x[0] + delta_y[0] * delta_y[0]);
    else
        para_out.biaDistance =
            -1 * sqrtf(delta_x[0] * delta_x[0] + delta_y[0] * delta_y[0]);

    para_out.preAngleDev = 0;
}

void ControlComply::VehiclePoseCalculation() {
    CONTROL_PARAM_IN para_in;
    para_in.cur_position.x_axis = mNavData.xAxis;
    para_in.cur_position.y_axis = mNavData.yAxis;
    para_in.cur_position.heading = mNavData.heading;
    // 这里计算了距离车辆最近点的曲率和横向误差赋值给了mControlData
    // (但是这里并没有用到预瞄参数)
    BiaAngleCalculate(mPathList, para_in, mControlData);
}

void ControlComply::VehicleVerticalControl(float tDesireSpeed, float tCurSpeed,
                                           float tAcc, uint8_t& tThrottle, uint8_t& tBrake) {
    spCtr_c.SpeedTrack(tDesireSpeed, tCurSpeed, tAcc, tThrottle, tBrake);

    if(tDesireSpeed == 0) {
        tThrottle = 0;
        tBrake = 70;
    } else {
        tBrake = 0;
    }
}

// ---------------------------------------------------------------------------
// 横向控制入口
// ---------------------------------------------------------------------------

float ControlComply::VehicleLateralControl() {
    XYZ_COOR_S xyz_temp;
    xyz_temp.x_axis = mNavData.xAxis;
    xyz_temp.y_axis = mNavData.yAxis;
    xyz_temp.heading = mNavData.heading;
    // 寻找最近点
    int keyPointTemp = pubalgor.FindKeyPointByTargetPoint_P(
        mPathList, mNavData.xAxis, mNavData.yAxis);
    // 横向控制(这里通过横向和航向误差，使用pid进行的转向角计算)
    double rtn_value = geoCon_c.LateralControlTrack1(
        mPathList, xyz_temp, keyPointTemp, mNavData.gpsSpeed, mGear);

    return rtn_value;
}

double ControlComply::azimuthToYaw(const double& azimuth) {
    // 转为与正东夹角，逆时针为正
    double angle = 90 - azimuth;

    if(angle < -180) {
        angle += 360;
    }
    // 转为弧度
    double rad_angle = angle / 180 * M_PI;
    return rad_angle;
}

float ControlComply::VehicleStanleyControl() {
    lat_controller.setParameters(1.6, 22, 8);
    std::vector<Pose2d> path_2d = toPath2d(mPathList);
    Pose2d ego_pose = Pose2d(mNavData.xAxis, mNavData.yAxis);
    ego_pose.heading = azimuthToYaw(mNavData.heading);
    float steering_angle =
        lat_controller.calculate(path_2d, ego_pose, mVehicleSpeed, mSteerAngle);
    return -1.1 * steering_angle * 180.0 / M_PI;
}

std::vector<Pose2d> ControlComply::toPath2d(
    const std::vector<XYZ_COOR_S>& old_path) {
    std::vector<Pose2d> new_path;
    for(const auto& point : old_path) {
        new_path.emplace_back(point.x_axis, point.y_axis);
    }
    math_utils::computePoseAttr(new_path);
    return new_path;
}

// 保持 ACC/AEB 与主编排处于原翻译单元，避免既有未初始化状态受链接布局影响。
#include "longitudinal_acc_control.inc"
