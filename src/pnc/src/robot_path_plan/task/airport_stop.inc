// 飞机位配置、占用状态和停车索引。由 path_plan_task.inc 包含。

void PathPlanComply::loadAirCraftPorts() {
    std::string task_file = "";
    ros::param::get("task_file", task_file);
    task_file += "aircraft_parking_port.yaml";
    printf("aircraft_parking_port.yaml path :%s\n", task_file.c_str());
    try {
        YAML::Node root = YAML::LoadFile(task_file.c_str());
        int new_go_id = root["go_task_id"].as<int>();
        int new_back_id = root["back_task_id"].as<int>();
        printf("go_task_id:%d, back_task_id:%d\n", new_go_id, new_back_id);
        // 解析port info(先组装局部表, 全部成功才提交)
        YAML::Node port_arr = root["aircraft_parkingPorts"];
        std::vector<AirCraftParkingPort> new_ports;
        for(std::size_t i = 0; i < port_arr.size(); ++i) {
            YAML::Node item = port_arr[i];
            int pid = item["id"].as<int>();

            // 读取子节点坐标
            YAML::Node go_pt = item["go_stop_point"];
            double gx = go_pt["x"].as<double>();
            double gy = go_pt["y"].as<double>();

            YAML::Node back_pt = item["back_stop_point"];
            double bx = back_pt["x"].as<double>();
            double by = back_pt["y"].as<double>();

            AirCraftParkingPort port;
            port.id = pid;
            port.parking = item["parking"].as<uint>();
            port.go_stop_point = Point(gx, gy);
            port.back_stop_point = Point(bx, by);
            new_ports.push_back(port);
            printf("i: %d,id:%d.parking:%d, gx: %f, gy: %f, bx: %f, by: %f\n", i, pid, port.parking, gx, gy, bx, by);
        }
        // 事务式提交: 中途任何字段异常都不会留下半新半旧的端口表
        go_task_id_ = new_go_id;
        back_task_id_ = new_back_id;
        aircraft_parking_ports_ = new_ports;
    } catch(const std::exception &e) {
        // 配置坏/缺不崩节点: 完整保留上一次端口表继续工作
        printf("aircraft_parking_port.yaml 加载失败, 保留上次端口表: %s\n",
               e.what());
    }
}

void PathPlanComply::updateAirPortStopIndex(std::vector<XYZ_COOR_S> &global_path, bool is_go_path) {
    // YAML 只保存停车点坐标；任务接收后把每个停车点投影到本次全局路径索引，
    // 这样周期限速逻辑可直接用 dist_origin 计算到下一飞机位的剩余距离。
    for(int i = 0; i < aircraft_parking_ports_.size(); i++) {
        auto &airport = aircraft_parking_ports_[i];
        Point stoppoint = is_go_path ? airport.go_stop_point : airport.back_stop_point;
        double min_distance = 1e9;
        airport.go_stop_index = 0;
        airport.back_stop_index = 0;
        for(int j = 0; j < global_path.size(); j++) {
            auto gpoint = global_path[j];
            double dist1 = pow((gpoint.x_axis - stoppoint.x), 2) + pow((gpoint.y_axis - stoppoint.y), 2);
            if(dist1 < min_distance) {
                min_distance = dist1;
                if(is_go_path) {
                    airport.go_stop_index = j;
                } else {
                    airport.back_stop_index = j;
                }
            }
        }
    }
}

void PathPlanComply::updateAirPortInfo(robot::AirCraftParkingPort airport) {
    for(int i = 0; i < aircraft_parking_ports_.size(); i++) {
        auto &port = aircraft_parking_ports_[i];
        if(port.id == airport.port_id) {
            port.parking = airport.status;
            return;
        }
    }
    ROS_ERROR("not found the airport:%d\n", int(airport.port_id));
}

AirCraftParkingPort PathPlanComply::findNextAirCraftParkingPort() {
    if(aircraft_parking_ports_.empty()) {
        // 空端口表(yaml 加载失败/数组为空): 停车点回退路径终点, 防 front() 越界
        printf("aircraft_parking_ports 为空, 停车点回退路径终点\n");
        AirCraftParkingPort fallback;
        fallback.go_stop_index = (int)mDrivingPath.size() - 1;
        fallback.back_stop_index = (int)mDrivingPath.size() - 1;
        return fallback;
    }
    AirCraftParkingPort next_port = aircraft_parking_ports_.front();
    int nearest_stop_index = mDrivingPath.size() - 1;
    for(int i = 0; i < aircraft_parking_ports_.size(); i++) {
        auto port = aircraft_parking_ports_[i];
        printf("mkeypoint:%d, port id:%d, go_stop_index:%d, back_stop_index:%d, nearest_stop_index:%d\n", mKeypoint,
               port.id, port.go_stop_index, port.back_stop_index, nearest_stop_index);
        if(port.go_stop_index > mKeypoint ||
           abs(port.go_stop_index - mKeypoint) < 5) {
            if(port.go_stop_index < nearest_stop_index) {
                nearest_stop_index = port.go_stop_index;
                next_port = port;
            }
        }
        if(port.back_stop_index > mKeypoint) {
            if(port.back_stop_index < nearest_stop_index ||
               abs(port.back_stop_index - mKeypoint) < 5) {
                nearest_stop_index = port.back_stop_index;
                next_port = port;
            }
        }
    }
    return next_port;
}
