#pragma once

// 旧场景夹具用同一实际速度驱动两种输入；来源隔离测试直接分别调用真实回调。
// 本头在 test_access.h 之后包含，以便保留夹具已有的导航位姿。
inline void SetVehicleFeedback(PathPlanComply& target, const robot::can_msg& state) {
    target.SetCanData(state);
    auto navigation = target.mNavData;
    navigation.gpsSpeed = state.vehicleSpeed;
    target.SetNavigationData(navigation);
}
