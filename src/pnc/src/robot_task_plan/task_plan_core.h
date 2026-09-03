#ifndef TASK_PLAN_CORE_H
#define TASK_PLAN_CORE_H

#include <cstdint>
#include <cstdio>
#include <exception>
#include <functional>
#include <string>
#include <vector>

#include "robot_path_plan/common/pubalgor/pubalgor.h"
// struct_type.h 的任务/挡位/子动作枚举经 pubalgor.h 引入:
//   TASKTYPE_E(NOTHING/TRACKPATH..ADAPTIVEPARK/DOACTION)、
//   TASKSTATUS(TASKFINISHED)、SUBACTION_E(HOOKOPERATION/DECOUPLING/WAITING/
//   UNLOAD_LOWER)、GEAR_N、MANUALCONTROLMODE / NOMALWORKINGMODE
#include "frame_transform.h"  // LatlonToUtmXY(远程指车目标点换算)
#include <yaml-cpp/yaml.h>    // 任务 yaml / config.yaml 解析

// ===========================================================================
// TaskPlanCore - 任务规划业务逻辑, 完全不依赖 ROS。
//
// 架构(2026-08-30 重构, 行为与原 task_plan_comply 实现一致,
// 同 canbus_core 模式):
//
//   task_plan_node.cpp (ROS 层)            TaskPlanCore (本类)
//   ------------------------------------  ------------------------------------
//   subscribe /path_plan_status  ------>  SetPathPlanStatus()
//   subscribe /can_msg           ------>  SetCanData()        手自动/换挡裁决
//   subscribe /navigation_msg    ------>  SetNavigationData()
//   subscribe /cloud/task/task_info --->  SetTaskInfo()       任务装载
//   subscribe /cloud/task/remote_signal> SetRemoteSignal()   等待区空闲标志
//   subscribe /cloud/msg/running_msg ---> OnRunningMsg()      远程指车
//   subscribe /cloud/msg/command_msg ---> OnCommandMsg()      停车/暂停/继续
//   主循环 20Hz --------------------->  TaskPlanProcess()     块推进状态机
//   主循环发布门控 ----------------->  PublishTaskPlanMsg()  组块+下发
//   10Hz 定时器 -------------------->  RunHeartBeat()        云端心跳组装
//
// 结果只通过两条路径汇报:
//   - 有序事件流(TaskPlanEvent 经 TaskPlanSink): 立即发布 /task_plan_msg、
//     /cloud/task/task_status、心跳/回执消息, rosparam 写(/cloud/suggestspeed),
//     ROS_INFO 日志。ROS 层按此顺序逐条转换成 ROS 调用。
//   - 公有成员: recived_cloud_task / mCurTaskNum / mExecuteTaskNum /
//     mPathPlanStatus, 供主循环做发布门控(与旧实现相同的门控条件)。
// 单线程模型: 全部在节点单线程 spinner 内调用, 无需加锁。
// ===========================================================================

// ---- 输入镜像(只含本模块实际读取的字段, 类型与 .msg 一致) ----
// 标量成员一律默认初始化为 0: 与旧实现依赖 gencpp 消息类零初始化构造的
// 语义一致, 不依赖实例的存储期。

// robot::path_plan_status: 块完成反馈 + 心跳用的转向/车速
struct PlanStatusIn {
    uint8_t taskExecuStatus = 0;  // 0-notask 1-processing 2-finish
    float curSpeed = 0;
    std::string turning;
};

// robot::can_msg: 面板/车速/挂钩位置/电量/故障码
struct CanStateIn {
    uint8_t controlPanelState = 0;  // 0 手动 1 自动
    float vehicleSpeed = 0;         // m/s
    uint8_t hookPos = 0;            // 挂钩销位置码(小=高)
    uint8_t hookStatus = 0;         // 执行器状态(HOOKSTATUS_E,canbus 状态机)
    uint8_t batteryPower = 0;       // 百分比
    std::vector<uint8_t> faultCode;
};

// canbus 启动时经 rosparam 发布的位置限值(/canbus/hookposition/{min,max},
// /canbus/palletposition/{min,max}, 0x285 原始码值)。码值小=位置高:
// min=顶端(hook up/pallet up), max=底端(hook down/pallet down)。初始值
// =config.cfg 默认值, 参数缺失时保持(读失败不覆盖, ZOH)
struct PositionLimitsIn {
    int hookPosMin = 185;
    int hookPosMax = 240;
    int palletPosMin = 130;
    int palletPosMax = 240;
};

// robot::navigation_msg: 心跳经纬度/航向/纵加速度
struct NavStateIn {
    float lat = 0;
    float lon = 0;
    float heading = 0;
    double longitudinal_accelerate = 0;  // 全链无生产者, 恒 0(原行为)
};

// robot::TaskInfo 整包镜像(原样回显进 TaskStatus.task_info, 含嵌套 target_info)
struct TargetInfoIn {
    int8_t area_type = 0;
    double dest_x = 0;
    double dest_y = 0;
    double heading = 0;
};

struct TaskInfoIn {
    int64_t timestamp_msec = 0;
    std::string vehicle_id;
    std::string task_from;
    int64_t task_id = 0;
    int8_t task_type = 0;
    int8_t task_op = 0;
    TargetInfoIn target_info;
};

// robot::RemoteSignal: 等待区是否满
struct RemoteSignalIn {
    int8_t isfull = 0;
};

// robot::RunningMsg.values.toPoint: 远程指车目标点(经纬度/航向)
struct RunningTargetIn {
    double lon = 0;
    double lat = 0;
    double heading = 0;
};

// robot::v2nCommandFeedback(/cloud/msg/command_msg): 云端指令
struct CommandIn {
    int32_t commandState = 0;  // 0 停车 1 暂停 2 继续
    int32_t commandID = 0;
    int32_t limit = 0;
    float speedCommand = 0;
};

// 10Hz 心跳拍的外部输入: 墙钟(年..秒, 未加时区) + 两个 rosparam 快照
struct HeartBeatInputs {
    int year = 0;
    int month = 0;
    int day = 0;
    int hour = 0;
    int minute = 0;
    int second = 0;
    int sensorstate = 0;  // "/planning/sensorstate" 位掩码, 读失败为 0
    int hookstate = 0;    // "/canbus/hookstate", 读失败为 0
};

// ---- 输出镜像(发布消息的逐字段拷贝源) ----

struct KeyPointOut {
    double lat = 0;
    double lon = 0;
    float heading = 0;
};

// robot::v2nHeartBeat 整包(仅 values.errorContext 不产出, 恒空串)
struct HeartBeatOut {
    std::string ts;
    std::string deviceId;
    std::string type;
    // values.*
    std::string status;
    std::string text;
    double lat = 0;
    double lon = 0;
    float heading = 0;
    std::string turning;
    float speed = 0;
    int32_t power = 0;
    std::vector<KeyPointOut> remainingKeyPoints;
    int32_t drivingState = 0;  // 0 停车 1 减速 2 加速 3 巡航
    int32_t sensorsState = 0;
    bool lidarState = false;
    bool cameraState = false;
    bool gnssState = false;
    bool vehicleState = false;
    int32_t hookState = 0;
    int32_t soc = 0;
    bool chargingState = false;
};

// robot::v2nCommandFeedback 整包
struct CommandFbOut {
    std::string ts;
    std::string deviceId;
    std::string type;
    // values.*
    std::string vTs;
    std::string vStatus;
    std::string vText;
    int32_t ready = 0;
    int32_t commandState = 0;
    int32_t commandID = 0;
    int32_t limit = 0;
    float speedCommand = 0;
};

// robot::v2nRunningFeedback 整包
struct RunningFbOut {
    std::string ts;
    std::string deviceId;
    std::string type;
    // values.*
    std::string vTs;
    std::string vStatus;
    std::string vText;
};

// robot::task_plan_msg 整包(pathX/Y/Angle 原实现只清不填, 保持)
struct TaskPlanMsgOut {
    uint8_t workMode = 0;
    uint8_t taskType = 0;
    std::vector<std::string> pathList;
    std::vector<float> pathX;
    std::vector<float> pathY;
    std::vector<float> pathAngle;
    float desireSpeed = 0;
    float stopX = 0;
    float stopY = 0;
    float stopAngle = 0;
    uint8_t desireGear = 0;
    uint8_t hookCmd = 0;
    int64_t task_id = 0;
};

// robot::TaskStatus 整包(timestamp_msec/fail_code/fail_reason 原实现不写)
struct TaskStatusOut {
    int64_t timestamp_msec = 0;
    std::string vehicle_id;
    TaskInfoIn task_info;
    int8_t procedure = 0;  // 1 执行中 2 已完成
    int32_t fail_code = 0;
    std::string fail_reason;
};

// ---- 有序事件流(core -> ROS 层) ----
enum TaskPlanEventType {
    TP_PUBLISH_TASK_PLAN,   // 发布 /task_plan_msg(取 GetTaskPlanMsg())
    TP_PUBLISH_TASK_STATUS, // 发布 /cloud/task/task_status(取 GetTaskStatus())
    TP_PUBLISH_HEARTBEAT,   // 发布 /v2nHeartBeat(取 GetHeartBeat())
    TP_PUBLISH_RUNNING_FB,  // 发布 /v2nRunningFeedback(取 GetRunningFb())
    TP_PUBLISH_COMMAND_FB,  // 发布 /v2nCommandFeedback(取 GetCommandFb())
    TP_PARAM_INT,           // ros::param::set(key, int)——心跳传感器兜底用整型
    TP_PARAM_DOUBLE,        // ros::param::set(key, double)——指令链路用浮点
    TP_LOG_INFO             // ROS_INFO(text)
};

struct TaskPlanEvent {
    TaskPlanEventType type = TP_PUBLISH_TASK_PLAN;
    std::string paramKey;   // TP_PARAM_* 有效
    int paramInt = 0;       // TP_PARAM_INT 有效
    double paramDouble = 0; // TP_PARAM_DOUBLE 有效
    std::string text;       // TP_LOG_INFO 有效
};

// 事件出口: core 每产生一条对外副作用就同步调用一次, ROS 层逐条转换。
// 空的 std::function 表示无接收方(测试/复用时允许)。
typedef std::function<void(const TaskPlanEvent &)> TaskPlanSink;

// 一个任务块(task{N}.yaml 中的一条)
struct TASKINFO_S {
    uint8_t tTaskType;
    std::vector<std::string> tPathList;
    std::vector<float> tPathX;
    std::vector<float> tPathY;
    std::vector<float> tPathAngle;
    float tSpeed;
    float tXAxis;
    float tYAxis;
    float tAngle;

    uint8_t tGear;
    uint8_t tSubAction;
    // int64: 与 task_plan_msg.task_id 对齐——原 uint8 会把任务 1000 截断成 232
    int64_t task_id;
};

class TaskPlanCore {
public:
    // ---- 主循环发布门控读取的状态(与旧实现同名的公有成员) ----
    bool recived_cloud_task = false;
    int mCurTaskNum = 0;
    int mExecuteTaskNum = -1;
    PlanStatusIn mPathPlanStatus;

    // 远程指车目标点(地图系), OnRunningMsg 写 / task1000 分支读
    double RunningtXAxis = 0.0;
    double RunningtYAxis = 0.0;
    double RunningtAngle = 0.0;

    TaskPlanCore() {};
    ~TaskPlanCore() {};

    // config_file: config.yaml 路径(文件不可读时 YAML 抛异常, 原行为保留)
    void InitParameter(const char *config_file);

    // ---- ROS 层推入的输入 ----
    // 命名划分: 输入镜像写入类入口用 pnc 惯例的 Set*(可含轻逻辑,
    // 如 SetCanData 自动模式重武装、SetTaskInfo 任务装载); 需要 core
    // 同步对外发事件的入口用 canbus_core 惯例的 On*(事件经 TaskPlanSink
    // 逐条回传 ROS 层)。
    void SetPathPlanStatus(const PlanStatusIn &path_plan_t);
    void SetCanData(const CanStateIn &can_data);
    // 挂钩/托盘位置限值快照(ROS 层每圈热读 rosparam 后整体推入, 见
    // PositionLimitsIn): 供 OperationStateJudge 判 hook 到位/退销
    void SetPositionLimits(const PositionLimitsIn &position_limits);
    void SetNavigationData(const NavStateIn &navigation_msg_t);
    void SetRemoteSignal(const RemoteSignalIn &remote_signal);
    // task_file_dir: rosparam "task_file"(param/ 目录),
    // 每次任务装载时由 ROS 层读取
    void SetTaskInfo(const TaskInfoIn &task_info,
                     const std::string &task_file_dir);
    void OnRunningMsg(const RunningTargetIn &target,
                      const std::string &task_file_dir);
    void OnCommandMsg(const CommandIn &cmd, TaskPlanSink sink);

    // ---- 周期作业 ----
    // 20Hz: 手自动模式判定 + 块推进状态机
    void TaskPlanProcess(TaskPlanSink sink);
    void PublishTaskPlanMsg(float max_vehicle_speed, TaskPlanSink sink);
    void RunHeartBeat(const HeartBeatInputs &in, TaskPlanSink sink);

    void clearTaskPool();  // 云端停车指令清空全部任务

    // ---- 事件发生时刻的输出快照(ROS 层逐字段拷贝成消息) ----
    const TaskPlanMsgOut &GetTaskPlanMsg() const { return mTaskPlanData; }
    const TaskStatusOut &GetTaskStatus() const { return mTaskStatus; }
    const HeartBeatOut &GetHeartBeat() const { return mHeartBeatData; }
    const CommandFbOut &GetCommandFb() const { return mCommandFbData; }
    const RunningFbOut &GetRunningFb() const { return mRunningFbData; }

private:
    void ClearTASKINFO_S(TASKINFO_S &tTaskInfo);
    // 文件缺失/字段坏 → printf 告警并返回空表(拒绝该任务), 不抛异常
    std::vector<TASKINFO_S> ParseTaskFile(std::string file_name,
                                          int64_t task_id,
                                          const std::string &task_file_dir);
    bool OperationStateJudge(TASKINFO_S tTask);
    bool JudgeArrivedDestination(TASKINFO_S tTask);
    void TaskManage(int &tCurTaskNum, std::vector<TASKINFO_S> tTaskList,
                    TaskPlanSink sink);

    void emitPublish(TaskPlanSink sink, TaskPlanEventType type);
    void emitParamInt(TaskPlanSink sink, const char *key, int value);
    void emitParamDouble(TaskPlanSink sink, const char *key, double value);
    void emitLogInfo(TaskPlanSink sink, const char *text);

    int mTimerCount = 0;         // 块推进去抖计数(上限 500)
    std::vector<TASKINFO_S> mTaskPool;

    TaskInfoIn mTaskInfoMsg;     // 最近一次任务消息(回显 + task1000 伪造基底)
    CanStateIn mCanData;
    PositionLimitsIn mPositionLimits;  // hook/pallet 判定线(默认=config.cfg 默认值)
    NavStateIn mNavData;
    RemoteSignalIn mRemoteSignal;
    std::vector<TASKINFO_S> mTaskList;

    // 历史例外: 旧 comply 同名成员, 依"不重命名既有标识符"铁律保留无前缀
    std::string cloudFeedbackStatus = "IDLE";  // 心跳上报状态字

    // 输出快照
    TaskPlanMsgOut mTaskPlanData;
    TaskStatusOut mTaskStatus;
    HeartBeatOut mHeartBeatData;
    CommandFbOut mCommandFbData;
    RunningFbOut mRunningFbData;

    // 心跳时间戳的秒内计数(原实现为回调内 static, 语义一致)
    int mHbCount = 0;
    int mHbSecondLast = 0;

    PubAlgor pubalgor;  // 历史例外: 旧 comply 同名成员, 保留无前缀
    // 仅加载校验, 无读者(保留加载副作用: 配置坏即启动失败)
    YAML::Node mConfigFile;
};

#endif  // TASK_PLAN_CORE_H
