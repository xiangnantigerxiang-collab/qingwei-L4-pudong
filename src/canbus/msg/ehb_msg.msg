# EHB(电子液压制动)状态消息 — 依据 docs/ehb-can.md(ehb-can.xls 保真整理)「发送信号需求」三报文+「接收信号需求」两报文及补充项
# 方向:EHB 发送三帧(整车接收)+ 整车/VCU(SA=0x58)→EHB 帧六帧(canbus 在 can0 旁听解析,不发送);
#   RecvCanData 解析入 mEHBMsg,T1 10Hz 发布于 /ehb_msg(/can_msg 09-07 起由 T2 20Hz 发布,两话题相位不锁)
# 位序 Intel(字节1~8,字节内位1~8,位1=最低位,ehb-can.md 2.2 显式整理约定);以下各帧均 DLC=8
# 发送方向(帧1~3,周期 100ms):0x08FB670E EHB_STORAGE 蓄能(64359-01~20,20 信号)
#   / 0x08FB680E EHB_AUTOBRAKE 主动制动(64360-1~18,18 信号) / 0x08FB690E EHB_EPB 驻车(64361-1~14,14 信号)
# 接收方向(帧4:20ms,帧5:100ms,补充项周期未提供):0x08FB1458 Vehicle_Brake_Request(64276-01~04,4 信号)
#   / 0x08FB1558 Vehicle_Parking_Request(64277-01~03,3 信号) / 补充项 0x0CFD0358 车速 / 0x0CFD0058
#   高压上电 / 0x0CFD0158 制动踏板 / 0x08FD0258 档位(ASCII R,N,D,P);「其他液压管路压力」ID=暂无,未纳入
# 注1:压力类为原始值(反馈×0.1 MPa;请求 VHL_ATB_BrakePressRequest×0.04 MPa;无效均 0xFE);1bit 故障位统一 0:未发生故障 1:发生故障
# 注2:xls 隐藏列 PGN/优先级/源地址与 ID 拆分不一致(ehb-can.md I01),以 D 列报文 ID 为准——
#   参数编号前缀 64359/64360/64361/64276/64277 与 ID 计算所得 PGN 自洽,佐证 D 列 ID
# 注3:接收方向四个补充项信号原表名为中文(车辆车速/点火钥匙状态(改为:高压上电状态)/制动踏板状态/车辆当前档位),
#   本文件英文名(VHL_VehicleSpeed/VHL_HvPowerState/VHL_BrakePedalStatus/VHL_VehicleGear)系整理命名,非原表

# ---- 帧1 0x08FB670E EHB_STORAGE 蓄能系统(7.5-8.6 十个故障位;8.7-8.8 未分配) ----
uint8 EHB_STO_StorageSystemStatus             # 1.1-1.2 EHB蓄能系统工作状态 00:加压关 01:加压开 10:错误 11:无效
uint8 EHB_STO_StorageSystemFault              # 1.3-1.4 EHB蓄能系统故障状态 00:无故障 01:警示 10:停车 11:无效
uint8 EHB_STO_StorageSystemLowPressureWarn    # 1.5-1.6 EHB蓄能系统低压警示 00:无警示 01:警示 10:停车 11:无效
uint8 EHB_STO_BrakeFluidPosition              # 1.7-1.8 制动液位信号状态（储液罐） 00:正常 01:过低 10:错误 11:无效
uint8 EHB_STO_PressureSensor1RawValue         # 2.1-2.8 液压传感器1原始压力值(raw×0.1 MPa,0xFE 无效)
uint8 EHB_STO_PressureSensor2RawValue         # 3.1-3.8 液压传感器2原始压力值(raw×0.1 MPa,0xFE 无效)
uint8 EHB_STO_PressureSensor3RawValue         # 4.1-4.8 液压传感器3原始压力值(raw×0.1 MPa,0xFE 无效)
uint8 EHB_STO_HighPressureAccumulatorA        # 5.1-5.8 高压蓄能器压力A(raw×0.1 MPa,0xFE 无效)
uint8 EHB_STO_HighPressureAccumulatorB        # 6.1-6.8 高压蓄能器压力B(raw×0.1 MPa,0xFE 无效)
uint8 EHB_STO_FailureNum                      # 7.1-7.4 蓄能系统当前故障数量(0~13;无效 0xE,AA 列另写 0xFE 系原表冲突 I04)
bool EHB_STO_FAU_PowerSupplyVoltageHigh       # 7.5 电源电压过高
bool EHB_STO_FAU_PowerSupplyVoltageLow        # 7.6 电源电压过低
bool EHB_STO_FAU_SensorSupplyError            # 7.7 液压传感器电源异常
bool EHB_STO_FAU_Sensor1Error                 # 7.8 1号液压传感器信号异常
bool EHB_STO_FAU_Sensor2Error                 # 8.1 2号液压传感器信号异常
bool EHB_STO_FAU_Sensor3Error                 # 8.2 3号液压传感器信号异常
bool EHB_STO_FAU_SensorJudgmentFailure        # 8.3 液压传感器信号无法解析
bool EHB_STO_FAU_MotorOpenLoad                # 8.4 蓄能电机断路
bool EHB_STO_FAU_MotorCannotStop              # 8.5 蓄能电机无法关闭
bool EHB_STO_FAU_MotorWorkTimeout             # 8.6 蓄能电机加压超时

# ---- 帧2 0x08FB680E EHB_AUTOBRAKE 主动制动(3.1-4.7 十五个故障位,位序按 ehb-can.md 逐行锚定;2.3-2.4 未分配) ----
uint8 EHB_ATB_ActualBrakePressure             # 1.1-1.8 主动制动系统实际压力(raw×0.1 MPa,0xFE 无效)
uint8 EHB_ATB_AutoBrakeSystemStatus           # 2.1-2.2 主动制动系统当前状态 0x0:正常 0x1:抑制 0x2:故障
uint8 EHB_ATB_FailureNum                      # 2.5-2.8 主动制动系统当前故障数量(0~13;无效 0xE,I04)
bool EHB_ATB_FAU_PowerSupplyVoltageHigh       # 3.1 电源电压过高
bool EHB_ATB_FAU_PowerSupplyVoltageLow        # 3.2 电源电压过低
bool EHB_ATB_FAU_CanBusOff                    # 3.3 CAN BUS OFF
bool EHB_ATB_FAU_SensorSupplyError            # 3.4 传感器电源故障
bool EHB_ATB_FAU_MotorDriverError             # 3.5 电机驱动故障
bool EHB_ATB_FAU_BrakePressSensorError        # 3.6 制动液压信号异常
bool EHB_ATB_FAU_LosOfEHBSTOCom               # 3.7 EHB蓄能器通讯丢失
bool EHB_ATB_FAU_MotorCommuteFail             # 3.8 电机校准失败
bool EHB_ATB_FAU_MotorOpenLoad                # 4.1 电机开路
bool EHB_ATB_FAU_MotorShort                   # 4.2 电机短路
bool EHB_ATB_FAU_MotorStall                   # 4.3 电机堵转
bool EHB_ATB_FAU_ValveBodyError               # 4.4 阀体故障
bool EHB_ATB_FAU_LosOfBrakeReqSignal          # 4.5 制动请求信号丢失
bool EHB_ATB_FAU_BrakeReqRcError              # 4.6 制动请求rollcounter错误
bool EHB_ATB_FAU_BrakeReqCsError              # 4.7 制动请求checksum错误

# ---- 帧3 0x08FB690E EHB_EPB 电子驻车(4.1-5.2 十个故障位;1.6-1.8/3.5-3.8/5.3-8.8 未分配) ----
uint8 EHB_EPB_SystemStatus                    # 1.1-1.2 驻车系统工作状态 00:正常 01:警示 10:停车 11:无效
uint8 EHB_EPB_ParkingStatus                   # 1.3-1.5 驻车系统驻车状态 0x0:已夹紧 0x1:已释放 0x2:夹紧中 0x3:释放中 0x4~0x6:未知(无效0b111)
uint8 EHB_EPB_ParkingPressure                 # 2.1-2.8 驻车系统液压(raw×0.1 MPa,0xFE 无效)
uint8 EHB_EPB_FailureNum                      # 3.1-3.4 驻车系统当前故障数(0~13;无效 0xE,I04)
bool EHB_EPB_FAU_PowerSupplyVoltageHigh       # 4.1 电源电压过高
bool EHB_EPB_FAU_PowerSupplyVoltageLow        # 4.2 电源电压过低
bool EHB_EPB_FAU_NoValueError                 # 4.3 常开电磁阀异常
bool EHB_EPB_FAU_NcValueError                 # 4.4 常闭电磁阀异常
bool EHB_EPB_FAU_SensorSupplyError            # 4.5 传感器电源异常
bool EHB_EPB_FAU_ParkingPresSensorError       # 4.6 驻车系统液压信号异常
bool EHB_EPB_FAU_ParkingPressureLow           # 4.7 驻车液压过低
bool EHB_EPB_FAU_LosOfParkingReqSignal        # 4.8 驻车请求信号丢失
bool EHB_EPB_FAU_ParkingReqRcError            # 5.1 驻车请求rollcounter错误
bool EHB_EPB_FAU_ParkingReqCsError            # 5.2 驻车请求checksum错误

# ---- 接收方向帧4 0x08FB1458 Vehicle_Brake_Request 制动请求(VCU SA=0x58→EHB,20ms,旁听解析;2.3-7.4 未分配) ----
uint8 VHL_ATB_BrakePressRequest               # 1.1-1.8 请求制动力大小(raw×0.04=0~8 MPa,0xFE 无效)
uint8 VHL_ATB_BrakePressRequestFlag           # 2.1-2.2 制动请求标志 00:无请求 01:有请求 10:未定义 11:无效(AC 列 0x11 越界系原表冲突 I03)
uint8 VHL_ATB_RollingCounter                  # 7.5-7.8 滚动计数器(0~15;声明 8bit 与位跨 4bit 冲突 I02,按位置 4bit 解析为显式实现假设)
uint8 VHL_ATB_CheckSum                        # 8.1-8.8 校验和 =(Byte0..Byte6 求和) XOR 0xFF(求和宽度未定义 I06;仅存值不校验)

# ---- 接收方向帧5 0x08FB1558 Vehicle_Parking_Request 驻车请求(VCU SA=0x58→EHB,100ms,旁听解析;1.3-7.4 未分配) ----
uint8 VHL_EPB_ParkingRequest                  # 1.1-1.2 驻车请求 0:无请求 1:请求驻车 2:请求释放 3:无效(I03 同上)
uint8 VHL_EPB_RollingCounter                  # 7.5-7.8 滚动计数器(0~15;I02 同上)
uint8 VHL_EPB_CheckSum                        # 8.1-8.8 校验和(同帧4,仅存值不校验)

# ---- 接收方向补充项(VCU SA=0x58→EHB;无报文头,周期未提供;原表信号名为中文,英文系整理命名,见头注3) ----
uint16 VHL_VehicleSpeed                       # 0x0CFD0358 5.1-6.8 车辆车速(raw×0.1=0~250.0 km/h,0xFFFF 忽略;64771-03)
uint8 VHL_HvPowerState                        # 0x0CFD0058 3.1-3.8 高压上电状态(原表「点火钥匙状态(改为:高压上电状态)」) 0x00:高压断电 0x01:高压上电 0xFF:忽略;64768-03
uint8 VHL_BrakePedalStatus                    # 0x0CFD0158 4.1-4.8 制动踏板状态(0-100%,0xFF 忽略;64769-05)
uint16 VHL_VehicleGear                        # 0x08FD0258 5.1-6.8 车辆当前档位 ASCII(R,N,D,P),0xFFFF 忽略;单字符两字节摆放未定义(I09),存原始 16bit 小端拼接;64770-04
