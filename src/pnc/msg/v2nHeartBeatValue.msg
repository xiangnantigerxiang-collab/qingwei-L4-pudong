string status
string text
float64 lat
float64 lon
float32 heading
string turning
float32 speed
#string speedChange
int32 power
v2nKeyPoint[] remainingKeyPoints

#驾驶状态 0-停车 1-减速 2-加速 3-巡航
int32 drivingState

#传感器状态 0-正常 1-异常 2-失效
int32 sensorsState

#雷达状态 0-有效 1-失效
bool lidarState

#摄像头状态 0-有效 1-失效
bool cameraState

#定位传感器状态 0-有效 1-失效
bool gnssState

#车辆状态 0-健康 1-故障
bool vehicleState
string errorContext

#拖挂钩状态 4-已挂钩/3-未挂钩/2-挂钩异常/1-脱钩告警
int32 hookState

#电池电量百分比(整数) 0-100%
int32 soc

#充电状态 0-未充电 1-充电中 2-已充满
bool chargingState
