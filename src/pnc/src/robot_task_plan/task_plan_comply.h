#ifndef TASK_PLAN_COMPLY_H
#define TASK_PLAN_COMPLY_H

#include <jsoncpp/json/json.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <vector>

#include "../common/pubalgor/pubalgor.h"
#include "robot/TaskInfo.h"
#include "robot/TaskStatus.h"
#include "robot/can_msg.h"
#include "robot/path_plan_status.h"
#include "robot/task_plan_msg.h"
#include "robot/RemoteSignal.h"
#include "robot/navigation_msg.h"
#include "robot/v2nHeartBeat.h"
#include "robot/v2nHeartBeatValue.h"
#include "robot/v2nKeyPoint.h"
#include "robot/v2nCommandFeedback.h"
#include "robot/v2nRunningFeedback.h"
#include "robot/v2nFeedbackValue.h"
#include "robot/HeadingPoint.h"
#include "robot/NotifyMsg.h"
#include "robot/NotifyObject.h"
#include "robot/NotifyValue.h"
#include "robot/RunningMsg.h"
#include "robot/RunningValue.h"
#include "robot/WarningMsg.h"
#include "robot/WarningValue.h"
#include "robot/CommandMsg.h"
#include "robot/CommandValue.h"

#include "ros/ros.h"
#include "yaml-cpp/yaml.h"
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "frame_transform.h"

typedef struct taskinfo
{
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
    uint8_t task_id;
}TASKINFO_S;

class TaskPlanComply
{
public:
 bool recived_cloud_task = false;
    TaskPlanComply() {};
    ~TaskPlanComply() {};

    void SetPathPlanStatus(robot::path_plan_status path_plan_t);
    void SetCanData(robot::can_msg can_data);
    void SetNavigationData(robot::navigation_msg navigation_msg_t);
    void SetTaskInfo(robot::TaskInfo task_info);
    void SetRemoteSignal(robot::RemoteSignal remote_signal);

    void InitParameter();
    void TaskPlanProcess();
    void PublishTaskPlanMsg(ros::Publisher tPub1, ros::Publisher tPub2);

    void clearTaskPool();

    double RunningtXAxis = 0.0;
    double RunningtYAxis = 0.0;
    double RunningtAngle = 0.0;

    int RunningFlag = 0;
    int RunningStatus = 0;
    int CommandFlag = 0;
    int CommandStatus = 0;

    std::string cloudFeedbackStatus = "IDLE";
    std::string CommandId = "";

    robot::TaskInfo mTaskInfoMsg;
    robot::navigation_msg mNavData;
    robot::can_msg mCanData;
    std::vector<TASKINFO_S> mTaskList;
    int mCurTaskNum = 0;
    int mExecuteTaskNum = -1;
    // int mLastTaskNUm  = 0;
    robot::path_plan_status mPathPlanStatus;

private:
    void ClearTASKINFO_S(TASKINFO_S& tTaskInfo);
    std::vector<TASKINFO_S> ParseTaskFile(std::string file_name, int task_id);
    bool OperationStateJudge(TASKINFO_S tTask);
    bool JudgeArrivedDestination(TASKINFO_S tTask);
    bool IsChangeTask(robot::TaskInfo task_info_last, robot::TaskInfo task_info_new);
    void TaskManage(int& tCurTaskNum, vector<TASKINFO_S> tTaskList);

private:
    int mTimerCount;
    
    std::vector<TASKINFO_S> mTaskPool;

    bool wait_task = false;
    // msg-receive
    
    robot::RemoteSignal mRemoteSignal;

    // msg-send
    robot::task_plan_msg mTaskPlanData;
    robot::TaskStatus mTaskStatus;
    // class
    PubAlgor pubalgor;
    YAML::Node mConfigFile;
   
};

#endif
