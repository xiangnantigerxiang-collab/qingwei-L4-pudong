#include "control_comply.h"

ControlComply::ControlComply()
{
    mPathList.clear();
    mGear = GEAR_N;
    mSpeed = 0;
    mKeyPoint = 0;
    mGpsFixed = false;
    mTlStatus.light_status = 2;
}

ControlComply::~ControlComply() {}

void ControlComply::InitParameter()
{
    string config_file = "";

    ros::param::get("control_para_file", config_file);
    config = YAML::LoadFile(config_file.c_str());

    mControlData.biaDistance = 0;
    mControlData.preCurve = 0;
    mControlData.preAngleDev = 0;
    mControlData.desireSpeed = 0;
    mControlData.desireAcc = 0;
    mControlData.throttlePercent = 0;
    mControlData.brakePercent = 70;
    mControlData.wheelAngle = 0;

    mTaskType = NOTHING;

    // geometric_control
    float lat_forward_predis = 0;
    float lat_back_predis = 0;
    float control_kv = 0;
    float vehicle_wheel_base = 0;

    lat_forward_predis = config["lat_forward_predis"].as<float>();
    lat_back_predis = config["lat_back_predis"].as<float>();
    control_kv = config["control_kv"].as<float>();
    vehicle_wheel_base = config["vehicle_wheel_base"].as<float>();
    geoCon_c.SetParameter(lat_forward_predis, lat_back_predis, vehicle_wheel_base, control_kv);

    // speed control
    float kp, ki, kd;
    float acc_min = 0;
    float acc_max = 0;
    float vehicle_mass = 0;
    float slope_para = 0;
    float froll = 0;
    float fwind = 0;
    float area = 0;
    float drive_torque = 0;
    float brake_torque = 0;
    float thro_dis = 0;
    float brk_dis = 0;

    kp = config["acc_kp"].as<float>();
    ki = config["acc_ki"].as<float>();
    kd = config["acc_kd"].as<float>();
    acc_min = config["acc_min"].as<float>();
    acc_max = config["acc_max"].as<float>();
    spCtr_c.SetAccPidParameter(kp, ki, kd, acc_min, acc_max);

    kp = config["throttle_kp"].as<float>();
    ki = config["throttle_ki"].as<float>();
    kd = config["throttle_kd"].as<float>();
    spCtr_c.SetThroPidParameter(kp, ki, kd);

    float temp_f = 0;
    for (int i = 0; i < 6; ++i)
    {
        temp_f = config["throttle_percent"][i].as<float>();
        mCurveX.push_back(temp_f);
        temp_f = config["actual_speed"][i].as<float>();
        mCurveY.push_back(temp_f);
    }

    SoundPlayCommand = 0;
}

void ControlComply::SetPathStatusData(robot::path_plan_status path_status_t)
{
    mPathStatus = path_status_t;
}

void ControlComply::setTaskPlanData(robot::task_plan_msg task_info)
{
    mTaskInfo = task_info;
}

void ControlComply::SetCanData(robot::can_msg can_msg_t)
{
    mGear = can_msg_t.curGear;
    wheelAngle = can_msg_t.wheelAngle;
    mVehicleSpeed = can_msg_t.vehicleSpeed;
    mSteerAngle = -can_msg_t.wheelAngle / 22.0 / 180 * 3.1415926;
}

void ControlComply::SetNavigationData(robot::navigation_msg navigation_t)
{
    mNavData = navigation_t;
    ego_pose2d.x = mNavData.xAxis;
    ego_pose2d.y = mNavData.yAxis;
    ego_pose2d.heading = azimuthToYaw(mNavData.heading);
    // mControlData.vehicleSpeed = mNavData.gpsSpeed;
}

void ControlComply::SetPathPlanData(robot::path_plan_msg path_plan_t)
{
    XYZ_COOR_S xyz_temp;
    std::vector<XYZ_COOR_S> src_path;

    if (path_plan_t.x.size() != path_plan_t.y.size())
        return;

    src_path.clear();

    mSpeed = path_plan_t.desireSpeed;
    mPathid = path_plan_t.Path_Id;
    mPathsafety = path_plan_t.safety;
    mPlanspeed = path_plan_t.planspeed;

    robot::path_plan_msg path_recv = path_plan_t;
    // 反转路径顺序
    std::reverse(path_recv.x.begin(), path_recv.x.end());
    std::reverse(path_recv.y.begin(), path_recv.y.end());

    if (path_recv.x.size() < 10)
        return;

    robot::path_plan_msg path_filter;
    int psize = path_recv.x.size();

    for (int i = 0; i < psize; i++)
    {
        if (i == 0)
        {
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
        if (len > 0.1 && len < 3.0)
        {
            path_filter.x.push_back(path_recv.x[i]);
            path_filter.y.push_back(path_recv.y[i]);
        }
    }

    std::reverse(path_filter.x.begin(), path_filter.x.end());
    std::reverse(path_filter.y.begin(), path_filter.y.end());
    path_plan_t = path_filter;

    if (path_plan_t.x.size() < 5)
        return;

    if (mPathid == 20)
    {
        CSpline spline;
        for (int i = 0; i < path_plan_t.x.size(); ++i)
        {
            xyz_temp.x_axis = path_plan_t.x[i];
            xyz_temp.y_axis = path_plan_t.y[i];
            src_path.push_back(xyz_temp);
        }

        mPathList.clear();
        mKeyPoint = 0;
        spline.SplinePointSet(src_path, mPathList, 0.1);
    }
    else
    {
        CSpline spline;

        for (int i = 0; i < path_plan_t.x.size(); ++i)
        {
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

void ControlComply::SetPalletCoorData(robot::hook_position hook_pos_t)
{
    mHookPos = hook_pos_t;
}

bool ControlComply::IsGreenLight(uint8_t light_state)
{
    static uint8_t light_buffer[5] = {0};

    for (int i = 0; i < 4; ++i)
        light_buffer[i] = light_buffer[i + 1];

    light_buffer[4] = light_state;

    for (int i = 0; i < 5; ++i)
    {
        if (light_buffer[i] != 2)
            return false;
    }

    return true;
}

void ControlComply::SetTlStatusData(robot::TLStatus tl_status)
{
    mTlStatus = tl_status;
}

void ControlComply::LoadPathFile(std::string tPath)
{
    std::vector<XYZ_COOR_S> vector_list;
    std::vector<XYZ_COOR_S> incsv;
    float distance_temp = 0;
    std::string file_dir = "";
    ros::param::get("path_dir", file_dir);
    XYZ_COOR_S intp;

    FILE *fp;
    std::string path_dir = file_dir + tPath + ".csv";
    fp = fopen(path_dir.c_str(), "r");

    while (!feof(fp))
    {
        fscanf(fp, "%f,%f,%f,%f", &intp.x_axis, &intp.y_axis, &intp.heading, &intp.z_axis);
        incsv.push_back(intp);
    }

    for (auto i : incsv)
    {
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

    // for (auto i : mFenceList)
    //     std::cout << "x = " << i.x_axis << " y = " << i.y_axis << std::endl;
}

int pointInPolygon(const std::vector<XYZ_COOR_S> &poly, const XYZ_COOR_S &p)
{
    const double EPS = 1e-9;

    int n = poly.size(), inside = 0;

    double px = p.x_axis;
    double py = p.y_axis;

    for (int i = 0, j = n - 1; i < n; j = i++)
    {
        const auto &a = poly[j], &b = poly[i];

        double ax = a.x_axis;
        double ay = a.y_axis;
        double bx = b.x_axis;
        double by = b.y_axis;

        double cross = (bx - ax) * (py - ay) - (by - ay) * (px - ax);

        if (std::abs(cross) < EPS && px >= std::min(ax, bx) - EPS && px <= std::max(ax, bx) + EPS && py >= std::min(ay, by) - EPS && py <= std::max(ay, by) + EPS)
            return 2;

        if ((ay > py) != (by > py) && px < (bx - ax) * (py - ay) / (by - ay) + ax)
            inside ^= 1;
    }

    return inside;
}

bool ControlComply::isWithinFence()
{
}

XYZ_COOR_S ControlComply::local2global2(double ox, double oy, double oheading,
                                        double lx, double ly, double lheading)
{
    double h = 90.0 - oheading;

    if (h > 360.0)
        h -= 360.0;
    if (h < 0.0)
        h += 360.0;

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

void ControlComply::FenceAlarm()
{
    if (mTaskInfo.taskType == ADAPTIVEHOOK) //hook循迹时不检测电子围栏
    {
        ros::param::set("/alarmcmd", 0);
        FenceWarning = 0;
        return;
    }

    int n = mFenceList.size();

    if (n < 2)
        return;

    // XYZ_COOR_S p;
    // p.x_axis = mNavData.xAxis;
    // p.y_axis = mNavData.yAxis;
    double x = mNavData.xAxis;
    double y = mNavData.yAxis;
    double h = mNavData.heading;
    auto point0 = local2global2(x, y, h, 2.3, 0.73, 0);  // 左前角
    auto point1 = local2global2(x, y, h, 2.3, -0.73, 0); // 右前角

    int check_left = pointInPolygon(mFenceList, point0);
    int check_right = pointInPolygon(mFenceList, point1);

    if (check_left != 1 || check_right != 1)
    {
        ros::param::set("alarmcmd", 1);
        FenceWarning = 1;
    }
    else
    {
        ros::param::set("alarmcmd", 0);
        FenceWarning = 0;
    }

    // std::cout << "x = " << (double)p.x_axis << " y= " << (double)p.y_axis << " check = " << check << " act = " << FenceWarning << std::endl;
}

void ControlComply::VehicleControl()
{
    //printf("ControlComply::VehicleControl()\n");

    float min_distance = 0;
    if (fabs(mSpeed) < 0.1 && mPathsafety)
    {
        printf("mPathsafety:%d\n", mPathsafety);
        mControlData.desireSpeed = 0;
        mControlData.desireAcc = 0;
        mControlData.throttlePercent = 0;
        mControlData.brakePercent = 100;
        mControlData.wheelAngle = 0;
        return;
    }

    if ((fabs(mSpeed) < 0.5 && mPathStatus.taskExecuStatus == 2) || mGear == GEAR_N)
    {
        printf("mGear:%d, execute stop:%d\n", mGear, mPathStatus.taskExecuStatus);
        mControlData.desireSpeed = 0;
        mControlData.desireAcc = 0;
        mControlData.throttlePercent = 0;
        mControlData.brakePercent = 80;
        mControlData.wheelAngle = 0;
        return;
    }

    if ((!IsGreenLight(mTlStatus.light_status)) && mNavData.heading > 80.0 && mNavData.heading < 100.0 &&
        mNavData.xAxis > 1553.0 && mNavData.xAxis < 1559.0)
    {
        mControlData.desireSpeed = 0;
        mControlData.desireAcc = 0;
        mControlData.throttlePercent = 0;
        mControlData.brakePercent = 30;
        mControlData.wheelAngle = 0;
        return;
    }

    if ((!IsGreenLight(mTlStatus.light_status)) && mNavData.heading > 80.0 && mNavData.heading < 100.0 &&
        mNavData.xAxis > 425.0 && mNavData.xAxis < 432.0)
    {
        mControlData.desireSpeed = 0;
        mControlData.desireAcc = 0;
        mControlData.throttlePercent = 0;
        mControlData.brakePercent = 50;
        mControlData.wheelAngle = 0;
        return;
    }
    if (mPathList.empty())
    {
        printf("空路径\n");
        mControlData.desireSpeed = 0;
        mControlData.desireAcc = 0;
        mControlData.throttlePercent = 0;
        mControlData.brakePercent = 50;
        mControlData.wheelAngle = 0;
        return;
    }
    // 计算车辆与轨迹最近点的横向偏差
    // printf("CalculateMinDistance_P\n");
    min_distance = pubalgor.CalculateMinDistance_P(mPathList, mNavData.xAxis, mNavData.yAxis);
    // 这里计算了距离车辆最近点的曲率和横向误差赋值给了mControlData(但是这里并没有用到预瞄参数)
    // printf("VehiclePoseCalculation\n");
    VehiclePoseCalculation();
    // 这里的mSpeed是通过path_plan_msg传入的期望速度
    mControlData.desireSpeed = mSpeed;
    // 这里基于期望速度和当前速度的差值调用模糊pid算法计算了期望加速度
    //  printf("AccelerationCalculateBySpeed_P\n");
    mControlData.desireAcc = spCtr_c.AccelerationCalculateBySpeed_P(mControlData.desireSpeed, mVehicleSpeed);

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
    if (AccSwitch)
    {
        double acc = LongitudinalControlOutput(acc_msg);
        mPlanspeed = mNavData.gpsSpeed + acc;

        if (mPlanspeed < 0.0)
            mPlanspeed = 0.0;
        if (mPlanspeed > mControlData.desireSpeed)
            mPlanspeed = mControlData.desireSpeed;

        if (mPlanspeed > 2.0)
            mPlanspeed = 2.0;
    }
    // 这里根据期望速度和期望加速度计算了油门和刹车的百分比(但是这里并没有用到期望加速度)，被后面代码覆盖，未使用
    VehicleVerticalControl(mControlData.desireSpeed, mNavData.gpsSpeed, mControlData.desireAcc,
                           mControlData.throttlePercent, mControlData.brakePercent);

    if (mPathid == 5 && mPathsafety)
    {
        mControlData.throttlePercent = 0;
        mControlData.brakePercent = 100;
    }
    else
    {
        // 横向控制，输出转向角
        if (mGear == GEAR_R)
        {
            mControlData.wheelAngle = VehicleLateralControl();
        }
        else
        {
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
        double speed_cmd = mSpeed; // 期望速度，路径发下来的
        double delta = speed_cmd - speed_now;

        if (delta > 0.5)
            speed_cmd = speed_now + 0.5;

        double curvelimitspeed = CurveLimitSpeed(mPathList);
         printf("curvelimitspeed:%f, mSpeed:%f, speed_cmd:%f, speed_node:%f\n",
               curvelimitspeed, mSpeed, speed_cmd, speed_now);
        speed_cmd = std::min<double>(speed_cmd, curvelimitspeed);

        //////////////////
        {
            // 缓停逻辑
            int current_state = 0;
            int hookstate = 0;
            ros::param::get("/canbus/hookstate", hookstate);
            ros::param::get("/planning/sensorstate", current_state);
            if ((current_state & 0x02) == 2 || (current_state & 0x04) == 4 || hookstate == 1)
            {
                printf("huanman tingche\n");
                // 减速停车
                double dcc = 8.0;
                double dcc_speed = speed_now - dcc * 0.05;
                speed_cmd = std::min<double>(speed_cmd, dcc_speed);
                speed_cmd = std::max<double>(speed_cmd, 0.0);
                printf("slow down to stop, speed_cmd:%f\n", speed_cmd);
            }
            // 急刹车
            if ((current_state & 0x08) == 8) // gnss故障，急停
            {
                printf("gnss error, jiting\n");
                speed_cmd = 0.0;
                mControlData.brakePercent = 100;
            }
            else
            {
                if (delta < -1.5)
                {
                    if (mControlData.brakePercent < 5)
                    {
                        mControlData.brakePercent += 1;
                    }
                }
                else
                {
                    mControlData.brakePercent = 0;
                }
            }
        }

        // double a = (speed_cmd - speed_now) / 0.05;
        // if(a > 0)
        // {
        //     mControlData.throttlePercent = a * 1.5 + 5;
        // }else
        // {
        //     mControlData.throttlePercent = 1/5.0;
        // }
        printf("speed_cmd: %f, speed_now: %f\n", speed_cmd, speed_now);
        mControlData.throttlePercent = speed_cmd * 18.0;
        if (speed_cmd - speed_now > 0.05)
        {
            mControlData.throttlePercent = mControlData.throttlePercent < 5 ? 5 : mControlData.throttlePercent;
        }

        // if(mControlData.throttlePercent < 10)
        // {
        //     mControlData.throttlePercent = 10;
        // }
        // printf("speed now:%f, desire_speed:%f, a:%f, current throttle cmd: %f", speed_now, speed_cmd, a, mControlData.throttlePercent);
        //////////////////////////////////////////////////////////////////
        if (FenceWarning == 1) // 冲出电子围栏
        {
            printf("冲出电子围栏, 停车\n");
            mControlData.brakePercent = 70.0;
            mControlData.throttlePercent = 0;
        }
        if (mControlData.biaDistance > 5.5 ||
            (mControlData.biaDistance > 4.0 && mControlData.biaAngle > 0.3)) // 冲出跑道
        {
            printf("横向偏差 > 2.5m, 停车\n");
            mControlData.brakePercent = 70;
            mControlData.throttlePercent = 0;
        }

        std::cout << "act = " << FenceWarning << " brake = " << (int)mControlData.brakePercent << std::endl;

        // 处理边界条件
        if (mControlData.brakePercent > 100)
            mControlData.brakePercent = 100;
        if (mControlData.throttlePercent > 100)
            mControlData.throttlePercent = 100;
        // 处理倒挡
        if (mGear == GEAR_R)
        {
            mControlData.throttlePercent = mSpeed * 10;
            if (mControlData.throttlePercent > 45)
                mControlData.throttlePercent = 45;
        }
    }
}

void ControlComply::PublishMessage(ros::Publisher &tPub)
{
    if (mControlData.throttlePercent > 0 && mControlData.brakePercent > 0)
    {
        robot::control_msg throttle_msg = mControlData;
        robot::control_msg brake_msg = mControlData;
        throttle_msg.brakePercent = 0;
        brake_msg.throttlePercent = 0;
        tPub.publish(throttle_msg);
        tPub.publish(brake_msg);
    }
    else
    {
        tPub.publish(mControlData);
    }
}

float ControlComply::CurveLimitSpeed(std::vector<XYZ_COOR_S> pathlist)
{
    static std::vector<double> last_curvatures(30, 0);
    int n = pathlist.size();
    int batch = 60;

    std::vector<double> curvatures;

    if (n < batch)
    {
        for (int i = 0; i < n; i++)
            curvatures.push_back(pathlist[i].curvature);
        for (int i = n; i < batch; i++)
            curvatures.push_back(0);
    }
    else
    {
        for (int i = 0; i < batch; i++)
            curvatures.push_back(pathlist[i].curvature);
    }
    
    double bias = last_curvatures.back() - curvatures.front();
    printf("bias:%f\n", bias);
    if(fabs(bias) > 0.001)
    {
    last_curvatures.erase(last_curvatures.begin());
    last_curvatures.push_back(curvatures.front());
    }
    
    double speed = 0.0;
    for (auto c : last_curvatures)
    {
        //printf("last curvature: %f\n", c);
        speed += c;
    }
    int num = 0;
    for (auto i : curvatures)
    {
        // printf("cur curvature: %f\n", i);
        if(num++ < 30)
          speed += i;
    }
   printf("====sum curvature:%f, batch:%d\n", speed, batch);
    speed = sqrt(speed / (double)(batch));
    if (speed < 0.001)
        speed = 0.001;
    printf("8888speed: %f\n", speed);
    double final_speed = 0.3/speed;
    printf("final speed: %f\n", final_speed);
    return final_speed;
    
}

float ControlComply::BiaAngleLimitSpeed(const float tBiaAngle)
{
    float angle[5] = {0};
    float speed[5] = {0};

    ros::param::get("speed_angle0", angle[0]);
    ros::param::get("speed_angle1", angle[1]);
    ros::param::get("speed_angle2", angle[2]);
    ros::param::get("speed_angle3", angle[3]);
    ros::param::get("speed_angle4", angle[4]);
    ros::param::get("speed_angle_limit0", angle[0]);
    ros::param::get("speed_angle_limit1", angle[1]);
    ros::param::get("speed_angle_limit2", angle[2]);
    ros::param::get("speed_angle_limit3", angle[3]);
    ros::param::get("speed_angle_limit4", angle[4]);

    return pubalgor.FuzzyDataProcess(angle, speed, 5, tBiaAngle);
}

float ControlComply::BiaDisLimitSpeed(const float tBiaDistance)
{
    float distance_array[5] = {0};
    float speed_array[5] = {0};

    ros::param::get("lateral_distance0", distance_array[0]);
    ros::param::get("lateral_distance1", distance_array[1]);
    ros::param::get("lateral_distance2", distance_array[2]);
    ros::param::get("lateral_distance3", distance_array[3]);
    ros::param::get("lateral_distance4", distance_array[4]);
    ros::param::get("speed_dis_limit0", speed_array[0]);
    ros::param::get("speed_dis_limit1", speed_array[1]);
    ros::param::get("speed_dis_limit2", speed_array[2]);
    ros::param::get("speed_dis_limit3", speed_array[3]);
    ros::param::get("speed_dis_limit4", speed_array[4]);

    return pubalgor.FuzzyDataProcess(distance_array, speed_array, 5, tBiaDistance);
}

float ControlComply::SpeedJudge(float tDesireSpeed)
{
    float rtn_speed = tDesireSpeed;
    float curve_limit_speed = 0;
    float angle_limit_speed = 0;
    float biadis_limit_speed = 0;
    float biaDisMax = 0;
    XYZ_COOR_S xyz_array[2];
    float distance_to_end = 0;

    curve_limit_speed = 1.0;

    if (rtn_speed > curve_limit_speed)
        rtn_speed = curve_limit_speed;

    angle_limit_speed = BiaAngleLimitSpeed(mControlData.preAngleDev);

    if (rtn_speed > angle_limit_speed)
        rtn_speed = angle_limit_speed;

    biadis_limit_speed = BiaDisLimitSpeed(mControlData.biaDistance);

    if (rtn_speed > biadis_limit_speed)
        rtn_speed = biadis_limit_speed;

    ros::param::get("control_lane_div", biaDisMax);
    if (mControlData.biaDistance > biaDisMax)
        rtn_speed = 0;
    if (mControlData.biaDistance > biaDisMax)
        rtn_speed = 0;

    xyz_array[0] = mPathList.at(mKeyPoint);
    xyz_array[1] = mPathList.at(mPathList.size() - 2);

    distance_to_end = pubalgor.CalculatePoint2PointDistance_P(
        xyz_array[0].x_axis, xyz_array[0].y_axis, 0,
        xyz_array[1].x_axis, xyz_array[1].y_axis, 0);

    return rtn_speed;
}

void ControlComply::CalcuPathCurve(vector<XYZ_COOR_S> &path_list)
{
    int size = path_list.size();
    float curve_temp = 0;

    if (size <= 1)
        return;

    for (int i = 0; i < size - 2; ++i)
    {
        if (i < size - 12)
        {
            float iTemp = 0;
            int count = 0;
            float length = 0;

            for (int j = i; j < i + 11; ++j)
            {
                float sub = path_list.at(j + 1).heading - path_list.at(j).heading;

                if (sub < 0)
                    count++;

                iTemp += sub;
                length += 0.1;
            }

            if (iTemp > 180)
                iTemp -= 360;
            if (iTemp < -180)
                iTemp += 360;

            curve_temp = iTemp * M_PI / 180 / length;

            if (curve_temp < 0)
                curve_temp = -1 * curve_temp;

            path_list.at(i).curvature = curve_temp;
        }
    }

    for (int i = size - 1; i > size - 13; i--)
        path_list.at(i).curvature = path_list.at(size - 13).curvature;
}

void ControlComply::CalcuPathHead(vector<XYZ_COOR_S> &path_list)
{
    int size = path_list.size();
    XYZ_COOR_S xyz_array[2];
    float angle_temp = 0;

    if (size == 0)
        return;

    for (int i = 0; i < size - 1; ++i)
    {
        xyz_array[0] = path_list.at(i);
        xyz_array[1] = path_list.at(i + 1);
        angle_temp = pubalgor.CalculatePoint2PointAngle_P(
            xyz_array[0].x_axis, xyz_array[0].y_axis, xyz_array[1].x_axis, xyz_array[1].y_axis);

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
    robot::control_msg &para_out)
{
    if (path_list.size() < 3)
    {
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

    for (int i = 0; i < size; i++)
    {
        xyz_temp = path_list.at(i);
        distance_temp = sqrt((xyz_temp.x_axis - cur_x) * (xyz_temp.x_axis - cur_x) +
                             (xyz_temp.y_axis - cur_y) * (xyz_temp.y_axis - cur_y));

        if (min_distance > distance_temp)
        {
            min_distance = distance_temp;
            new_key_point = i % size;
        }
    }

    mKeyPoint = new_key_point;
   // printf("keypoint:%d, size:%d\n", mKeyPoint, path_list.size());
    para_out.preCurve = path_list.at(mKeyPoint).curvature;

    if (path_list.at(path_list.size() - 3).curvature > para_out.preCurve)
        para_out.preCurve = path_list.at(path_list.size() - 3).curvature;

    delta_x[0] = cur_x - path_list.at(new_key_point).x_axis;
    delta_y[0] = cur_y - path_list.at(new_key_point).y_axis;
    delta_x[1] = path_list.at((new_key_point + 2) % size).x_axis - path_list.at(new_key_point).x_axis;
    delta_y[1] = path_list.at((new_key_point + 2) % size).y_axis - path_list.at(new_key_point).y_axis;

    distance_temp = delta_x[1] * delta_y[0] - delta_y[1] * delta_x[0];

    if (distance_temp > 0)
        para_out.biaDistance = sqrtf(delta_x[0] * delta_x[0] + delta_y[0] * delta_y[0]);
    else
        para_out.biaDistance = -1 * sqrtf(delta_x[0] * delta_x[0] + delta_y[0] * delta_y[0]);

    para_out.preAngleDev = 0;
}

void ControlComply::VehiclePoseCalculation()
{
    CONTROL_PARAM_IN para_in;
    para_in.cur_position.x_axis = mNavData.xAxis;
    para_in.cur_position.y_axis = mNavData.yAxis;
    para_in.cur_position.z_axis = mNavData.zAxis;
    para_in.cur_position.heading = mNavData.heading;
    para_in.key_point = mKeyPoint;
    ros::param::get("forward_preview_dis", para_in.forward_preview_dis); // 2
    ros::param::get("back_preview_dis", para_in.back_preview_dis);       // 2
    ros::param::get("preview_time", para_in.preview_time);               // 0.6
    ros::param::get("curve_preview_dis", para_in.curve_preview_dis);     // 6
    ros::param::get("preview_time2", para_in.preview_time2);             // 0.6           // 1.5
    para_in.cur_gear = mGear;
    para_in.vehicle_speed = mNavData.gpsSpeed;
    // 这里计算了距离车辆最近点的曲率和横向误差赋值给了mControlData(但是这里并没有用到预瞄参数)
    BiaAngleCalculate(mPathList, para_in, mControlData);
}

void ControlComply::VehicleVerticalControl(
    float tDesireSpeed, float tCurSpeed, float tAcc, uint8_t &tThrottle, uint8_t &tBrake)
{
    spCtr_c.SpeedTrack1(tDesireSpeed, tCurSpeed, tAcc, tThrottle, tBrake);

    if (tDesireSpeed == 0)
    {
        tThrottle = 0;
        tBrake = 70;
    }
    else
    {
        LeastSquares least_squares_c(5, mCurveX, mCurveY, 6);
        tThrottle = least_squares_c.GetXValueByY(tDesireSpeed);
        tBrake = 0;
    }
}

float ControlComply::VehicleLateralControl()
{
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

double ControlComply::azimuthToYaw(const double &azimuth)
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

float ControlComply::VehicleStanleyControl()
{
    lat_controller.setParameters(1.6, 22, 8);
    std::vector<Pose2d> path_2d = toPath2d(mPathList);
    Pose2d ego_pose = Pose2d(mNavData.xAxis, mNavData.yAxis);
    ego_pose.heading = azimuthToYaw(mNavData.heading);
    float steering_angle = lat_controller.calculate(path_2d, ego_pose, mVehicleSpeed, mSteerAngle);
    return -1.1*steering_angle * 180.0 / M_PI;
}

std::vector<Pose2d> ControlComply::toPath2d(const std::vector<XYZ_COOR_S> &old_path)
{
    std::vector<Pose2d> new_path;
    for (const auto &point : old_path)
    {
        new_path.emplace_back(point.x_axis, point.y_axis);
    }
    math_utils::computePoseAttr(new_path);
    return new_path;
}
