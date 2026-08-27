string ts
string deviceId
string type
CommandValue values

#发送端：自检请求 0-忽略 1-自检
#接收端：自检结果 0-失败 1-通过
int32 ready

#发送端：任务指令 0-停止 1-暂停 2-运行
#接收端：任务响应 0-停止 1-暂停 2-运行 3-完成
int32 commandState
#发送端：期望任务ID
#接收端：执行任务ID
int32 commandID

#发送端：限速请求 0-无请求 1-请求限速
#接收端：限速响应 0-未响应 1-已响应
int32 limit

#发送端：期望速度(m/s)
#接收端：限制速度(m/s)
float32 speedCommand
