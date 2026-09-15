// 周边/T 路口的跨帧观察和等待区判断。由 path_plan_output.inc 包含。

bool PathPlanComply::checkTJunctionObstacle() {
    std::vector<Dot> vertexes;
    Dot vertex = {-4.83, 18.3};
    vertexes.push_back(vertex);

    vertex = {0.22, 20};
    vertexes.push_back(vertex);

    vertex = {12.3, -17.8};
    vertexes.push_back(vertex);

    vertex = {7.2, -19.5};
    vertexes.push_back(vertex);
    bool is_unsafe = false;
    for(auto i : mPerception.objs) {
        Dot objpoint = {i.x, i.y};
        if(pointInRectangle(vertexes, objpoint) != 0) {
            // ROS_WARN("20m Object detect in lane");
            is_unsafe = true;
            break;
        }
    }
    unsafe_vec.erase(unsafe_vec.begin());
    unsafe_vec.push_back(is_unsafe);
    int sum = 0;
    printf("T unsafe: ");
    for(int i = 0; i < unsafe_vec.size(); i++) {
        printf("%d ", int(unsafe_vec[i]));
        sum += unsafe_vec[i];
    }
    printf(" %d\n", sum);
    if(sum > 0) {  // 连续4帧有一帧有障碍物，则认为有危险
        return true;
    }
    return false;
}

bool PathPlanComply::checkAroundObstacle() {
    // 自车位置
    double x = mNavData.xAxis;
    double y = mNavData.yAxis;
    double h = mNavData.heading;

    auto point0 = local2global2(x, y, h, 2.3, 0.73, 0);    // 左前角
    auto point1 = local2global2(x, y, h, 2.3, -0.73, 0);   // 右前角
    auto point2 = local2global2(x, y, h, -0.8, 0.73, 0);   // 左后角
    auto point3 = local2global2(x, y, h, -0.8, -0.73, 0);  // 右后角
    GeoPoint point0_geo(point0.x, point0.y);
    GeoPoint point1_geo(point1.x, point1.y);
    GeoPoint point2_geo(point2.x, point2.y);
    GeoPoint point3_geo(point3.x, point3.y);
    GeoLine line0(point0_geo, point1_geo);  // 前
    GeoLine line1(point1_geo, point3_geo);  // 右
    GeoLine line2(point2_geo, point3_geo);  // 后
    GeoLine line3(point0_geo, point2_geo);  // 左

    double dist0 = Distance2PerceptionObjs(line0);
    double dist1 = Distance2PerceptionObjs(line1);
    double dist2 = Distance2PerceptionObjs(line2);
    double dist3 = Distance2PerceptionObjs(line3);

    double safe_dist = 1.7 - 0.75;
    if(int(mVehicleData.linkPallet) == 1) {  // 连接托盘
        safe_dist = 2.2 - 0.75;
    }

    bool unsafe = (dist0 < safe_dist || dist1 < safe_dist || dist2 < safe_dist || dist3 < safe_dist);

    history_unsafe.erase(history_unsafe.begin());
    history_unsafe.push_back(unsafe);
    int sum = 0;
    printf("safe_dist:%.2f, 前侧 = %.2f, 右侧 = %.2f, 后侧 = %.2f, 左侧 = %.2f\n", safe_dist, dist0, dist1, dist2, dist3);
    printf("Around_unsafe: ");
    for(int i = 0; i < history_unsafe.size(); i++) {
        printf("%d ", int(history_unsafe[i]));
        if(history_unsafe[i])
            sum += 1;
    }
    printf(" %d\n", sum);
    // if(history_unsafe.back() == true)
    // {
    //     return true;
    // }
    return sum > 5;
}

bool PathPlanComply::checkIsInWaiting() {
    GeoPolygon waiting_polygon;
    waiting_polygon.outer().push_back(GeoPoint(1.0, 4.0));
    waiting_polygon.outer().push_back(GeoPoint(4.1, -4.7));
    waiting_polygon.outer().push_back(GeoPoint(-0.4, -6.13));
    waiting_polygon.outer().push_back(GeoPoint(-3.37, 2.45));
    GeoPoint cur_point(mNavData.xAxis, mNavData.yAxis);
    return bg::within(cur_point, waiting_polygon);
}
