# 独立消息：不改变既有 task_plan_msg/path_plan_msg 的 MD5。
# task_plan -> planning：任务许可及20Hz存活；反向同类型消息：本代路径校验结果。
std_msgs/Header header
uint64 revision
uint64 task_key
uint64 fence_version
uint64 route_version
uint32 start_index
uint32 stop_index
string fence_file
bool stop
bool truncated
string reason
bool completed
int8 alarm
