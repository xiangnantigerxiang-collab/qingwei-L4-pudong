# EHB CAN 通信矩阵：面向编程大模型的保真整理

> 来源为 `ehb-can.xls`。本文区分 **原表记录**、**显式规范化**、**整理推导** 和 **待确认问题**。原表本身存在冲突：保留冲突两侧的原值，不把推测改写成协议事实。本文不是经设备方确认的修订协议。

## 1. 来源、范围与阅读顺序

源文件：`ehb-can.xls`；大小：`213504` 字节；SHA-256：`3ebdb6be23daca54af07329d112f115cd989365ab3881ec7e2e113358fb676e0`。单独阅读本 Markdown 即可获取全部单元格文字/数值、图片内容转写及协议疑点；原始图片、完整结构审计、规范化 JSON 和源文件随完整资料包保存。

| 原始顺序 | 工作表 | 可见性 | 有值单元格 | 合并区域数 |
| --- | --- | --- | --- | --- |
| 1 | 封面签字 | 隐藏 | 25 | 2 |
| 2 | 发送信号需求 | 可见 | 743 | 9 |
| 3 | 接收信号需求 | 可见 | 216 | 8 |
| 4 | 其他 | 隐藏 | 1 | 1 |
| 5 | 模板版本管理 | 隐藏 | 24 | 0 |

合计 **1,009 个有值单元格**，其中 741 个文本单元格、268 个数值单元格；包含 2 个仅由空格组成的文本单元格。另有 2,507 个显式空白/带样式空白单元格、20 个合并区域、3 张嵌入 PNG、2 项数据验证和 1 个定义名称。未检测到公式单元格、单元格批注记录或单元格超链接记录。

协议条目合计：**5 个命名报文、59 个对应信号**，另有 **5 个补充接收项目**（其中 1 项仅为待定义占位），共 64 个信号/待定义项目；共出现 9 个不同的具体 CAN ID。发送信号 52 个；两个命名接收报文共 7 个信号。

建议编程模型先读第 2 节的字段约定和第 6 节的疑点，再使用第 3～5 节；需要确认任一字面量、空格、类型或来源时，查第 10 节逐行原始单元格账本。第 7～9 节保留隐藏表、附图、表格结构和历史引用，不要将模板痕迹当作新增协议。

## 2. 编程读取约定与原始字段映射

### 2.1 不得隐式补全的事项

`null` 表示源单元格未填写，不等于 0、默认值、无效值或“不适用”。原有 `TBD`、`暂无`、`0b00`、`0x11`、`0~0xC8` 等文本保留。原始大小写、拼写和参数编号均保留；例如 `EHB_EPB_FAU_NoValueError` 不改写为 `...ValveError`，`LosOf` 不改为 `LossOf`。参数编号 `64359-01` 等保留为字符串，不自动生成整数 SPN。

规范化记录仅把 S/V/W 列可明确解析的数值文本转为数值，把 T/U 列位置转成字符串；所有原始存储类型仍可在第 10 节找到。例如发送!S45 原为字符串 `"2"`，接收!V14 原为字符串 `"0.1"`。信号的所有 O～AE 列均有对应字段；原表没有填写的字段也显式列为 null。

方向以工作表名称及“控制器=EHB”为依据，组织为 EHB 视角的 TX/RX；这不意味着 AE“源节点”已经填写。不得将整车侧程序的发送方向与 EHB 视角的发送方向混淆。命名报文分组依据独立报文头行及其后信号行，不能对整个工作表无条件向下填充。

### 2.2 位位置与派生字段

原表使用 `1.1`、`1.8`、`7.5` 等位置记法，字节次序列填写 `intel`。本文为便于程序处理，**显式采用**“字节 1～8、字节内位 1～8，位 1 对应最低位”的整理约定；原表未另外给出位编号文字定义。此约定需在协议实现验收时确认，不能把它说成额外原文。

```text
start_lsb0 = (start_byte_1based - 1) * 8 + (start_bit_1based - 1)
end_lsb0   = (end_byte_1based   - 1) * 8 + (end_bit_1based   - 1)
span_bits  = end_lsb0 - start_lsb0 + 1
length_consistent = (span_bits == declared_length_bits)
```

`derived_layout` 仅保存上述计算结果，不替代 S 列声明长度。两个滚动计数器同时保留 `declared_length_bits=8`、`span_bits=4`、`length_consistent=false`。`1.1` 是位置记法，不是小数长度或已经可直接使用的 DBC start bit。

### 2.3 原表 A～AE 列完整映射

| 列 | 原始标题 | 结构化键名 | 原表可见性 |
| --- | --- | --- | --- |
| A | 控制器 | controller | 可见 |
| B | 序号 | sequence | 可见 |
| C | 报文名称 | message_name | 可见 |
| D | 报文ID | can_id_literal | 可见 |
| E | PGN | pgn_cell | 两张信号表中隐藏 |
| F | 优先级 | priority_cell | 两张信号表中隐藏 |
| G | 数据页 | data_page_cell | 两张信号表中隐藏 |
| H | PDU格式（Hex） | pdu_format_hex_cell | 两张信号表中隐藏 |
| I | 特定PDU（Hex） | pdu_specific_hex_cell | 两张信号表中隐藏 |
| J | 源地址(Hex) | source_address_hex_cell | 两张信号表中隐藏 |
| K | 自定义 | custom_cell | 两张信号表中隐藏 |
| L | 报文发送类型 | send_type | 可见 |
| M | 报文周期(ms) | cycle_ms | 可见 |
| N | 报文数据长度（byte） | dlc_bytes | 可见 |
| O | 信号名称 | signal_name | 可见 |
| P | 信号描述 | description | 可见 |
| Q | 可疑参数编号 | parameter_no | 可见 |
| R | 字节次序 | byte_order | 可见 |
| S | 信号长度（bit） | declared_length_bits | 可见 |
| T | 起始位 | start | 可见 |
| U | 终止位 | end | 可见 |
| V | 精度 | factor | 可见 |
| W | 偏移量 | offset | 可见 |
| X | 最小值—最大值（logical value） | logical_range | 可见 |
| Y | 最小值—最大值（physical value） | physical_range | 可见 |
| Z | 信号单位 | unit | 可见 |
| AA | 信号值描述 | value_description | 可见 |
| AB | 默认值（Hex） | default_literal | 可见 |
| AC | 无效值（Hex） | invalid_literal | 可见 |
| AD | 备注 | remark | 可见 |
| AE | 源节点 | source_node | 可见 |

接收表 D3 的标题另含换行：`报文ID\nXX=（00～FF）`；保留这一原文，不据此把实际 ID 末字节自动替换成任意地址。两表 A1 均为“整车通信网络信号矩阵”并带尾随空格；B2 为 `Message Information 报文信息`，N2 为 `Signal Information 信号信息`。标题和尾随空格的精确值见原始账本。

## 3. 五个命名报文及全部信号

| 整理键 | EHB 方向 | 报文名 | 原始 CAN ID | 周期 ms | 数据字节数 | 信号行/数量 |
| --- | --- | --- | --- | --- | --- | --- |
| TX-01 | TX | EHB_STORAGE | 0x08FB670E | 100 | 8 | 5～24 / 20 |
| TX-02 | TX | EHB_AUTOBRAKE | 0x08FB680E | 100 | 8 | 26～43 / 18 |
| TX-03 | TX | EHB_EPB | 0x08FB690E | 100 | 8 | 45～58 / 14 |
| RX-01 | RX | Vehicle_Brake_Request | 0x08FB1458 | 20 | 8 | 5～8 / 4 |
| RX-02 | RX | Vehicle_Parking_Request | 0x08FB1558 | 100 | 8 | 10～12 / 3 |

### 3.1. EHB_STORAGE（0x08FB670E）

来源：`发送信号需求` 第 4 行报文头；后续第 5～24 行为该报文信号。下列报文信息包含原来隐藏的 E～K 列；命名报文均涉及 I01。

```json
{
  "record_key": "TX-01",
  "source_sheet": "发送信号需求",
  "source_row": 4,
  "direction_from_ehb": "TX",
  "controller": "EHB",
  "sequence": 1,
  "message_name": "EHB_STORAGE",
  "can_id_literal": "0x08FB670E",
  "pgn_cell": 65381,
  "priority_cell": 6,
  "data_page_cell": 0,
  "pdu_format_hex_cell": "FF",
  "pdu_specific_hex_cell": 65,
  "source_address_hex_cell": "2D",
  "custom_cell": "自定义",
  "send_type": "Periodic",
  "cycle_ms": 100,
  "dlc_bytes": 8,
  "remark": null,
  "source_node": null,
  "issue_refs": [
    "I01"
  ]
}
```

**完整信号记录（JSONL，每行一个信号；所有空值均显式保留）**

```jsonl
{"source_row":5,"signal_name":"EHB_STO_StorageSystemStatus","description":"EHB蓄能系统工作状态","parameter_no":"64359-01","byte_order":"intel","declared_length_bits":2,"start":"1.1","end":"1.2","factor":null,"offset":null,"logical_range":"0~0x3","physical_range":"0~3","unit":"bit","value_description":"00: 加压关\n01: 加压开\n10: 错误\n11: 无效","default_literal":"0b00","invalid_literal":"0b11","remark":null,"source_node":null,"derived_layout":{"start_lsb0":0,"end_lsb0":1,"span_bits":2,"length_consistent":true},"issue_refs":[]}
{"source_row":6,"signal_name":"EHB_STO_StorageSystemFault","description":"EHB蓄能系统故障状态","parameter_no":"64359-02","byte_order":"intel","declared_length_bits":2,"start":"1.3","end":"1.4","factor":null,"offset":null,"logical_range":"0~0x3","physical_range":"0~3","unit":"bit","value_description":"00: 无故障\n01: 警示\n10: 停车\n11: 无效","default_literal":"0b00","invalid_literal":"0b11","remark":null,"source_node":null,"derived_layout":{"start_lsb0":2,"end_lsb0":3,"span_bits":2,"length_consistent":true},"issue_refs":[]}
{"source_row":7,"signal_name":"EHB_STO_StorageSystemLowPressureWarn","description":"EHB蓄能系统低压警示","parameter_no":"64359-03","byte_order":"intel","declared_length_bits":2,"start":"1.5","end":"1.6","factor":null,"offset":null,"logical_range":"0~0x3","physical_range":"0~3","unit":"bit","value_description":"00: 无警示\n01: 警示\n10: 停车\n11: 无效","default_literal":"0b00","invalid_literal":"0b11","remark":null,"source_node":null,"derived_layout":{"start_lsb0":4,"end_lsb0":5,"span_bits":2,"length_consistent":true},"issue_refs":[]}
{"source_row":8,"signal_name":"EHB_STO_BrakeFluidPosition","description":"制动液位信号状态（储液罐）","parameter_no":"64359-04","byte_order":"intel","declared_length_bits":2,"start":"1.7","end":"1.8","factor":null,"offset":null,"logical_range":"0~0x3","physical_range":"0~3","unit":"bit","value_description":"00: 正常\n01: 过低\n10: 错误\n11: 无效","default_literal":"0b00","invalid_literal":"0b11","remark":null,"source_node":null,"derived_layout":{"start_lsb0":6,"end_lsb0":7,"span_bits":2,"length_consistent":true},"issue_refs":[]}
{"source_row":9,"signal_name":"EHB_STO_PressureSensor1RawValue","description":"液压传感器1原始压力值","parameter_no":"64359-05","byte_order":"intel","declared_length_bits":8,"start":"2.1","end":"2.8","factor":0.1,"offset":0,"logical_range":"0~0xC8","physical_range":"0~20","unit":"MPa","value_description":"0~0xC8: 液压传感器1原始压力值\n0xFE: 无效","default_literal":"0b00","invalid_literal":"0xFE","remark":null,"source_node":null,"derived_layout":{"start_lsb0":8,"end_lsb0":15,"span_bits":8,"length_consistent":true},"issue_refs":[]}
{"source_row":10,"signal_name":"EHB_STO_PressureSensor2RawValue","description":"液压传感器2原始压力值","parameter_no":"64359-06","byte_order":"intel","declared_length_bits":8,"start":"3.1","end":"3.8","factor":0.1,"offset":0,"logical_range":"0~0xC8","physical_range":"0~20","unit":"MPa","value_description":"0~0xC8: 液压传感器2原始压力值\n0xFE: 无效","default_literal":"0b00","invalid_literal":"0xFE","remark":null,"source_node":null,"derived_layout":{"start_lsb0":16,"end_lsb0":23,"span_bits":8,"length_consistent":true},"issue_refs":[]}
{"source_row":11,"signal_name":"EHB_STO_PressureSensor3RawValue","description":"液压传感器3原始压力值","parameter_no":"64359-07","byte_order":"intel","declared_length_bits":8,"start":"4.1","end":"4.8","factor":0.1,"offset":0,"logical_range":"0~0xC8","physical_range":"0~20","unit":"MPa","value_description":"0~0xC8: 液压传感器3原始压力值\n0xFE: 无效","default_literal":"0b00","invalid_literal":"0xFE","remark":null,"source_node":null,"derived_layout":{"start_lsb0":24,"end_lsb0":31,"span_bits":8,"length_consistent":true},"issue_refs":[]}
{"source_row":12,"signal_name":"EHB_STO_HighPressureAccumulatorA","description":"高压蓄能器压力A","parameter_no":"64359-08","byte_order":"intel","declared_length_bits":8,"start":"5.1","end":"5.8","factor":0.1,"offset":0,"logical_range":"0~0xC8","physical_range":"0~20","unit":"MPa","value_description":"0~0xC8: 高压蓄能器A压力值\n0xFE: 无效","default_literal":"0b00","invalid_literal":"0xFE","remark":null,"source_node":null,"derived_layout":{"start_lsb0":32,"end_lsb0":39,"span_bits":8,"length_consistent":true},"issue_refs":[]}
{"source_row":13,"signal_name":"EHB_STO_HighPressureAccumulatorB","description":"高压蓄能器压力B","parameter_no":"64359-09","byte_order":"intel","declared_length_bits":8,"start":"6.1","end":"6.8","factor":0.1,"offset":0,"logical_range":"0~0xC8","physical_range":"0~20","unit":"MPa","value_description":"0~0xC8: 高压蓄能器B压力值\n0xFE: 无效","default_literal":"0b00","invalid_literal":"0xFE","remark":null,"source_node":null,"derived_layout":{"start_lsb0":40,"end_lsb0":47,"span_bits":8,"length_consistent":true},"issue_refs":[]}
{"source_row":14,"signal_name":"EHB_STO_FailureNum","description":"蓄能系统当前故障数量","parameter_no":"64359-10","byte_order":"intel","declared_length_bits":4,"start":"7.1","end":"7.4","factor":1,"offset":0,"logical_range":"0~0xD","physical_range":"0~13","unit":"count","value_description":"0~0xD: 系统当前故障数\n0xFE: 无效","default_literal":"0b00","invalid_literal":"0xE","remark":null,"source_node":null,"derived_layout":{"start_lsb0":48,"end_lsb0":51,"span_bits":4,"length_consistent":true},"issue_refs":["I04"]}
{"source_row":15,"signal_name":"EHB_STO_FAU_PowerSupplyVoltageHigh","description":"电源电压过高","parameter_no":"64359-11","byte_order":"intel","declared_length_bits":1,"start":"7.5","end":"7.5","factor":null,"offset":null,"logical_range":"0~0x1","physical_range":"0~1","unit":"bit","value_description":"0: 未发生故障\n1: 发生故障","default_literal":"0b00","invalid_literal":null,"remark":null,"source_node":null,"derived_layout":{"start_lsb0":52,"end_lsb0":52,"span_bits":1,"length_consistent":true},"issue_refs":[]}
{"source_row":16,"signal_name":"EHB_STO_FAU_PowerSupplyVoltageLow","description":"电源电压过低","parameter_no":"64359-12","byte_order":"intel","declared_length_bits":1,"start":"7.6","end":"7.6","factor":null,"offset":null,"logical_range":"0~0x1","physical_range":"0~1","unit":"bit","value_description":"0: 未发生故障\n1: 发生故障","default_literal":"0b00","invalid_literal":null,"remark":null,"source_node":null,"derived_layout":{"start_lsb0":53,"end_lsb0":53,"span_bits":1,"length_consistent":true},"issue_refs":[]}
{"source_row":17,"signal_name":"EHB_STO_FAU_SensorSupplyError","description":"液压传感器电源异常","parameter_no":"64359-13","byte_order":"intel","declared_length_bits":1,"start":"7.7","end":"7.7","factor":null,"offset":null,"logical_range":"0~0x1","physical_range":"0~1","unit":"bit","value_description":"0: 未发生故障\n1: 发生故障","default_literal":"0b00","invalid_literal":null,"remark":null,"source_node":null,"derived_layout":{"start_lsb0":54,"end_lsb0":54,"span_bits":1,"length_consistent":true},"issue_refs":[]}
{"source_row":18,"signal_name":"EHB_STO_FAU_Sensor1Error","description":"1号液压传感器信号异常","parameter_no":"64359-14","byte_order":"intel","declared_length_bits":1,"start":"7.8","end":"7.8","factor":null,"offset":null,"logical_range":"0~0x1","physical_range":"0~1","unit":"bit","value_description":"0: 未发生故障\n1: 发生故障","default_literal":"0b00","invalid_literal":null,"remark":null,"source_node":null,"derived_layout":{"start_lsb0":55,"end_lsb0":55,"span_bits":1,"length_consistent":true},"issue_refs":[]}
{"source_row":19,"signal_name":"EHB_STO_FAU_Sensor2Error","description":"2号液压传感器信号异常","parameter_no":"64359-15","byte_order":"intel","declared_length_bits":1,"start":"8.1","end":"8.1","factor":null,"offset":null,"logical_range":"0~0x1","physical_range":"0~1","unit":"bit","value_description":"0: 未发生故障\n1: 发生故障","default_literal":"0b00","invalid_literal":null,"remark":null,"source_node":null,"derived_layout":{"start_lsb0":56,"end_lsb0":56,"span_bits":1,"length_consistent":true},"issue_refs":[]}
{"source_row":20,"signal_name":"EHB_STO_FAU_Sensor3Error","description":"3号液压传感器信号异常","parameter_no":"64359-16","byte_order":"intel","declared_length_bits":1,"start":"8.2","end":"8.2","factor":null,"offset":null,"logical_range":"0~0x1","physical_range":"0~1","unit":"bit","value_description":"0: 未发生故障\n1: 发生故障","default_literal":"0b00","invalid_literal":null,"remark":null,"source_node":null,"derived_layout":{"start_lsb0":57,"end_lsb0":57,"span_bits":1,"length_consistent":true},"issue_refs":[]}
{"source_row":21,"signal_name":"EHB_STO_FAU_SensorJudgmentFailure","description":"液压传感器信号无法解析","parameter_no":"64359-17","byte_order":"intel","declared_length_bits":1,"start":"8.3","end":"8.3","factor":null,"offset":null,"logical_range":"0~0x1","physical_range":"0~1","unit":"bit","value_description":"0: 未发生故障\n1: 发生故障","default_literal":"0b00","invalid_literal":null,"remark":null,"source_node":null,"derived_layout":{"start_lsb0":58,"end_lsb0":58,"span_bits":1,"length_consistent":true},"issue_refs":[]}
{"source_row":22,"signal_name":"EHB_STO_FAU_MotorOpenLoad","description":"蓄能电机断路","parameter_no":"64359-18","byte_order":"intel","declared_length_bits":1,"start":"8.4","end":"8.4","factor":null,"offset":null,"logical_range":"0~0x1","physical_range":"0~1","unit":"bit","value_description":"0: 未发生故障\n1: 发生故障","default_literal":"0b00","invalid_literal":null,"remark":null,"source_node":null,"derived_layout":{"start_lsb0":59,"end_lsb0":59,"span_bits":1,"length_consistent":true},"issue_refs":[]}
{"source_row":23,"signal_name":"EHB_STO_FAU_MotorCannotStop","description":"蓄能电机无法关闭","parameter_no":"64359-19","byte_order":"intel","declared_length_bits":1,"start":"8.5","end":"8.5","factor":null,"offset":null,"logical_range":"0~0x1","physical_range":"0~1","unit":"bit","value_description":"0: 未发生故障\n1: 发生故障","default_literal":"0b00","invalid_literal":null,"remark":null,"source_node":null,"derived_layout":{"start_lsb0":60,"end_lsb0":60,"span_bits":1,"length_consistent":true},"issue_refs":[]}
{"source_row":24,"signal_name":"EHB_STO_FAU_MotorWorkTimeout","description":"蓄能电机加压超时","parameter_no":"64359-20","byte_order":"intel","declared_length_bits":1,"start":"8.6","end":"8.6","factor":null,"offset":null,"logical_range":"0~0x1","physical_range":"0~1","unit":"bit","value_description":"0: 未发生故障\n1: 发生故障","default_literal":"0b00","invalid_literal":null,"remark":null,"source_node":null,"derived_layout":{"start_lsb0":61,"end_lsb0":61,"span_bits":1,"length_consistent":true},"issue_refs":[]}
```

按原始起止位置计算的未分配 LSB0 位索引（仅用于覆盖校验，不代表默认填充值）：`[62,63]`。仅按原表起止位置统计，不等同于已确认的保留位编码；计数器长度冲突仍保留。

### 3.2. EHB_AUTOBRAKE（0x08FB680E）

来源：`发送信号需求` 第 25 行报文头；后续第 26～43 行为该报文信号。下列报文信息包含原来隐藏的 E～K 列；命名报文均涉及 I01。

```json
{
  "record_key": "TX-02",
  "source_sheet": "发送信号需求",
  "source_row": 25,
  "direction_from_ehb": "TX",
  "controller": "EHB",
  "sequence": 2,
  "message_name": "EHB_AUTOBRAKE",
  "can_id_literal": "0x08FB680E",
  "pgn_cell": 65382,
  "priority_cell": 6,
  "data_page_cell": 0,
  "pdu_format_hex_cell": "FF",
  "pdu_specific_hex_cell": 66,
  "source_address_hex_cell": "2D",
  "custom_cell": "自定义",
  "send_type": "Periodic",
  "cycle_ms": 100,
  "dlc_bytes": 8,
  "remark": null,
  "source_node": null,
  "issue_refs": [
    "I01"
  ]
}
```

**完整信号记录（JSONL，每行一个信号；所有空值均显式保留）**

```jsonl
{"source_row":26,"signal_name":"EHB_ATB_ActualBrakePressure","description":"主动制动系统实际压力","parameter_no":"64360-1","byte_order":"intel","declared_length_bits":8,"start":"1.1","end":"1.8","factor":0.1,"offset":0,"logical_range":"0~0xC8","physical_range":"0~20","unit":"MPa","value_description":"0~0xC8: 主动制动系统实际压力\n0xFE: 无效","default_literal":"0b00","invalid_literal":"0xFE","remark":null,"source_node":null,"derived_layout":{"start_lsb0":0,"end_lsb0":7,"span_bits":8,"length_consistent":true},"issue_refs":[]}
{"source_row":27,"signal_name":"EHB_ATB_AutoBrakeSystemStatus","description":"主动制动系统当前状态","parameter_no":"64360-2","byte_order":"intel","declared_length_bits":2,"start":"2.1","end":"2.2","factor":1,"offset":0,"logical_range":"0~0x3","physical_range":"0~3","unit":"bit","value_description":"0x0: 正常\n0x1: 抑制\n0x2: 故障","default_literal":"0b00","invalid_literal":"0b11","remark":null,"source_node":null,"derived_layout":{"start_lsb0":8,"end_lsb0":9,"span_bits":2,"length_consistent":true},"issue_refs":[]}
{"source_row":28,"signal_name":"EHB_ATB_FailureNum","description":"主动制动系统当前故障数量","parameter_no":"64360-3","byte_order":"intel","declared_length_bits":4,"start":"2.5","end":"2.8","factor":1,"offset":0,"logical_range":"0~0xD","physical_range":"0~13","unit":"count","value_description":"0~0xD: 系统当前故障数\n0xFE: 无效","default_literal":"0b00","invalid_literal":"0xE","remark":null,"source_node":null,"derived_layout":{"start_lsb0":12,"end_lsb0":15,"span_bits":4,"length_consistent":true},"issue_refs":["I04"]}
{"source_row":29,"signal_name":"EHB_ATB_FAU_PowerSupplyVoltageHigh","description":"电源电压过高","parameter_no":"64360-4","byte_order":"intel","declared_length_bits":1,"start":"3.1","end":"3.1","factor":null,"offset":null,"logical_range":"0~0x1","physical_range":"0~1","unit":"bit","value_description":"0: 未发生故障\n1: 发生故障","default_literal":"0b00","invalid_literal":null,"remark":null,"source_node":null,"derived_layout":{"start_lsb0":16,"end_lsb0":16,"span_bits":1,"length_consistent":true},"issue_refs":[]}
{"source_row":30,"signal_name":"EHB_ATB_FAU_PowerSupplyVoltageLow","description":"电源电压过低","parameter_no":"64360-5","byte_order":"intel","declared_length_bits":1,"start":"3.2","end":"3.2","factor":null,"offset":null,"logical_range":"0~0x1","physical_range":"0~1","unit":"bit","value_description":"0: 未发生故障\n1: 发生故障","default_literal":"0b00","invalid_literal":null,"remark":null,"source_node":null,"derived_layout":{"start_lsb0":17,"end_lsb0":17,"span_bits":1,"length_consistent":true},"issue_refs":[]}
{"source_row":31,"signal_name":"EHB_ATB_FAU_CanBusOff","description":"CAN BUS OFF","parameter_no":"64360-6","byte_order":"intel","declared_length_bits":1,"start":"3.3","end":"3.3","factor":null,"offset":null,"logical_range":"0~0x1","physical_range":"0~1","unit":"bit","value_description":"0: 未发生故障\n1: 发生故障","default_literal":"0b00","invalid_literal":null,"remark":null,"source_node":null,"derived_layout":{"start_lsb0":18,"end_lsb0":18,"span_bits":1,"length_consistent":true},"issue_refs":[]}
{"source_row":32,"signal_name":"EHB_ATB_FAU_SensorSupplyError","description":"传感器电源故障","parameter_no":"64360-7","byte_order":"intel","declared_length_bits":1,"start":"3.4","end":"3.4","factor":null,"offset":null,"logical_range":"0~0x1","physical_range":"0~1","unit":"bit","value_description":"0: 未发生故障\n1: 发生故障","default_literal":"0b00","invalid_literal":null,"remark":null,"source_node":null,"derived_layout":{"start_lsb0":19,"end_lsb0":19,"span_bits":1,"length_consistent":true},"issue_refs":[]}
{"source_row":33,"signal_name":"EHB_ATB_FAU_MotorDriverError","description":"电机驱动故障","parameter_no":"64360-8","byte_order":"intel","declared_length_bits":1,"start":"3.5","end":"3.5","factor":null,"offset":null,"logical_range":"0~0x1","physical_range":"0~1","unit":"bit","value_description":"0: 未发生故障\n1: 发生故障","default_literal":"0b00","invalid_literal":null,"remark":null,"source_node":null,"derived_layout":{"start_lsb0":20,"end_lsb0":20,"span_bits":1,"length_consistent":true},"issue_refs":[]}
{"source_row":34,"signal_name":"EHB_ATB_FAU_BrakePressSensorError","description":"制动液压信号异常","parameter_no":"64360-9","byte_order":"intel","declared_length_bits":1,"start":"3.6","end":"3.6","factor":null,"offset":null,"logical_range":"0~0x1","physical_range":"0~1","unit":"bit","value_description":"0: 未发生故障\n1: 发生故障","default_literal":"0b00","invalid_literal":null,"remark":null,"source_node":null,"derived_layout":{"start_lsb0":21,"end_lsb0":21,"span_bits":1,"length_consistent":true},"issue_refs":[]}
{"source_row":35,"signal_name":"EHB_ATB_FAU_LosOfEHBSTOCom","description":"EHB蓄能器通讯丢失","parameter_no":"64360-10","byte_order":"intel","declared_length_bits":1,"start":"3.7","end":"3.7","factor":null,"offset":null,"logical_range":"0~0x1","physical_range":"0~1","unit":"bit","value_description":"0: 未发生故障\n1: 发生故障","default_literal":"0b00","invalid_literal":null,"remark":null,"source_node":null,"derived_layout":{"start_lsb0":22,"end_lsb0":22,"span_bits":1,"length_consistent":true},"issue_refs":[]}
{"source_row":36,"signal_name":"EHB_ATB_FAU_MotorCommuteFail","description":"电机校准失败","parameter_no":"64360-11","byte_order":"intel","declared_length_bits":1,"start":"3.8","end":"3.8","factor":null,"offset":null,"logical_range":"0~0x1","physical_range":"0~1","unit":"bit","value_description":"0: 未发生故障\n1: 发生故障","default_literal":"0b00","invalid_literal":null,"remark":null,"source_node":null,"derived_layout":{"start_lsb0":23,"end_lsb0":23,"span_bits":1,"length_consistent":true},"issue_refs":[]}
{"source_row":37,"signal_name":"EHB_ATB_FAU_MotorOpenLoad","description":"电机开路","parameter_no":"64360-12","byte_order":"intel","declared_length_bits":1,"start":"4.1","end":"4.1","factor":null,"offset":null,"logical_range":"0~0x1","physical_range":"0~1","unit":"bit","value_description":"0: 未发生故障\n1: 发生故障","default_literal":"0b00","invalid_literal":null,"remark":null,"source_node":null,"derived_layout":{"start_lsb0":24,"end_lsb0":24,"span_bits":1,"length_consistent":true},"issue_refs":[]}
{"source_row":38,"signal_name":"EHB_ATB_FAU_MotorShort","description":"电机短路","parameter_no":"64360-13","byte_order":"intel","declared_length_bits":1,"start":"4.2","end":"4.2","factor":null,"offset":null,"logical_range":"0~0x1","physical_range":"0~1","unit":"bit","value_description":"0: 未发生故障\n1: 发生故障","default_literal":"0b00","invalid_literal":null,"remark":null,"source_node":null,"derived_layout":{"start_lsb0":25,"end_lsb0":25,"span_bits":1,"length_consistent":true},"issue_refs":[]}
{"source_row":39,"signal_name":"EHB_ATB_FAU_MotorStall","description":"电机堵转","parameter_no":"64360-14","byte_order":"intel","declared_length_bits":1,"start":"4.3","end":"4.3","factor":null,"offset":null,"logical_range":"0~0x1","physical_range":"0~1","unit":"bit","value_description":"0: 未发生故障\n1: 发生故障","default_literal":"0b00","invalid_literal":null,"remark":null,"source_node":null,"derived_layout":{"start_lsb0":26,"end_lsb0":26,"span_bits":1,"length_consistent":true},"issue_refs":[]}
{"source_row":40,"signal_name":"EHB_ATB_FAU_ValveBodyError","description":"阀体故障","parameter_no":"64360-15","byte_order":"intel","declared_length_bits":1,"start":"4.4","end":"4.4","factor":null,"offset":null,"logical_range":"0~0x1","physical_range":"0~1","unit":"bit","value_description":"0: 未发生故障\n1: 发生故障","default_literal":"0b00","invalid_literal":null,"remark":null,"source_node":null,"derived_layout":{"start_lsb0":27,"end_lsb0":27,"span_bits":1,"length_consistent":true},"issue_refs":[]}
{"source_row":41,"signal_name":"EHB_ATB_FAU_LosOfBrakeReqSignal","description":"制动请求信号丢失","parameter_no":"64360-16","byte_order":"intel","declared_length_bits":1,"start":"4.5","end":"4.5","factor":null,"offset":null,"logical_range":"0~0x1","physical_range":"0~1","unit":"bit","value_description":"0: 未发生故障\n1: 发生故障","default_literal":"0b00","invalid_literal":null,"remark":null,"source_node":null,"derived_layout":{"start_lsb0":28,"end_lsb0":28,"span_bits":1,"length_consistent":true},"issue_refs":[]}
{"source_row":42,"signal_name":"EHB_ATB_FAU_BrakeReqRcError","description":"制动请求rollcounter错误","parameter_no":"64360-17","byte_order":"intel","declared_length_bits":1,"start":"4.6","end":"4.6","factor":null,"offset":null,"logical_range":"0~0x1","physical_range":"0~1","unit":"bit","value_description":"0: 未发生故障\n1: 发生故障","default_literal":"0b00","invalid_literal":null,"remark":null,"source_node":null,"derived_layout":{"start_lsb0":29,"end_lsb0":29,"span_bits":1,"length_consistent":true},"issue_refs":[]}
{"source_row":43,"signal_name":"EHB_ATB_FAU_BrakeReqCsError","description":"制动请求checksum错误","parameter_no":"64360-18","byte_order":"intel","declared_length_bits":1,"start":"4.7","end":"4.7","factor":null,"offset":null,"logical_range":"0~0x1","physical_range":"0~1","unit":"bit","value_description":"0: 未发生故障\n1: 发生故障","default_literal":"0b00","invalid_literal":null,"remark":null,"source_node":null,"derived_layout":{"start_lsb0":30,"end_lsb0":30,"span_bits":1,"length_consistent":true},"issue_refs":[]}
```

按原始起止位置计算的未分配 LSB0 位索引（仅用于覆盖校验，不代表默认填充值）：`[10,11,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63]`。仅按原表起止位置统计，不等同于已确认的保留位编码；计数器长度冲突仍保留。

### 3.3. EHB_EPB（0x08FB690E）

来源：`发送信号需求` 第 44 行报文头；后续第 45～58 行为该报文信号。下列报文信息包含原来隐藏的 E～K 列；命名报文均涉及 I01。

```json
{
  "record_key": "TX-03",
  "source_sheet": "发送信号需求",
  "source_row": 44,
  "direction_from_ehb": "TX",
  "controller": "EHB",
  "sequence": 3,
  "message_name": "EHB_EPB",
  "can_id_literal": "0x08FB690E",
  "pgn_cell": 65226,
  "priority_cell": 6,
  "data_page_cell": 0,
  "pdu_format_hex_cell": "FE",
  "pdu_specific_hex_cell": "CA",
  "source_address_hex_cell": "2D",
  "custom_cell": "J1939",
  "send_type": "Periodic",
  "cycle_ms": 100,
  "dlc_bytes": 8,
  "remark": null,
  "source_node": null,
  "issue_refs": [
    "I01"
  ]
}
```

**完整信号记录（JSONL，每行一个信号；所有空值均显式保留）**

```jsonl
{"source_row":45,"signal_name":"EHB_EPB_SystemStatus","description":"驻车系统工作状态","parameter_no":"64361-1","byte_order":"intel","declared_length_bits":2,"start":"1.1","end":"1.2","factor":1,"offset":0,"logical_range":"0x0-0x3","physical_range":"0-3","unit":"bit","value_description":"00: 正常\n01: 警示\n10: 停车\n11: 无效","default_literal":"0b00","invalid_literal":"0b11","remark":null,"source_node":null,"derived_layout":{"start_lsb0":0,"end_lsb0":1,"span_bits":2,"length_consistent":true},"issue_refs":[]}
{"source_row":46,"signal_name":"EHB_EPB_ParkingStatus","description":"驻车系统驻车状态","parameter_no":"64361-2","byte_order":"intel","declared_length_bits":3,"start":"1.3","end":"1.5","factor":1,"offset":0,"logical_range":"0x0-0x7","physical_range":"0-7","unit":"bit","value_description":"0x0: 已夹紧\n0x1: 已释放\n0x2:  夹紧中\n0x3: 释放中\n0x4~0x6: 未知","default_literal":"0b00","invalid_literal":"0b111","remark":null,"source_node":null,"derived_layout":{"start_lsb0":2,"end_lsb0":4,"span_bits":3,"length_consistent":true},"issue_refs":[]}
{"source_row":47,"signal_name":"EHB_EPB_ParkingPressure","description":"驻车系统液压","parameter_no":"64361-3","byte_order":"intel","declared_length_bits":8,"start":"2.1","end":"2.8","factor":0.1,"offset":0,"logical_range":"0~0xC8","physical_range":"0~20","unit":"MPa","value_description":"0~0xC8: 驻车系统液压\n0xFE: 无效","default_literal":"0b00","invalid_literal":"0xFE","remark":null,"source_node":null,"derived_layout":{"start_lsb0":8,"end_lsb0":15,"span_bits":8,"length_consistent":true},"issue_refs":[]}
{"source_row":48,"signal_name":"EHB_EPB_FailureNum","description":"驻车系统当前故障数","parameter_no":"64361-4","byte_order":"intel","declared_length_bits":4,"start":"3.1","end":"3.4","factor":1,"offset":0,"logical_range":"0~0xD","physical_range":"0~13","unit":"count","value_description":"0~0xD: 系统当前故障数\n0xFE: 无效","default_literal":"0b00","invalid_literal":"0xE","remark":null,"source_node":null,"derived_layout":{"start_lsb0":16,"end_lsb0":19,"span_bits":4,"length_consistent":true},"issue_refs":["I04"]}
{"source_row":49,"signal_name":"EHB_EPB_FAU_PowerSupplyVoltageHigh","description":"电源电压过高","parameter_no":"64361-5","byte_order":"intel","declared_length_bits":1,"start":"4.1","end":"4.1","factor":null,"offset":null,"logical_range":"0~0x1","physical_range":"0~1","unit":"bit","value_description":"0: 未发生故障\n1: 发生故障","default_literal":"0b00","invalid_literal":null,"remark":null,"source_node":null,"derived_layout":{"start_lsb0":24,"end_lsb0":24,"span_bits":1,"length_consistent":true},"issue_refs":[]}
{"source_row":50,"signal_name":"EHB_EPB_FAU_PowerSupplyVoltageLow","description":"电源电压过低","parameter_no":"64361-6","byte_order":"intel","declared_length_bits":1,"start":"4.2","end":"4.2","factor":null,"offset":null,"logical_range":"0~0x1","physical_range":"0~1","unit":"bit","value_description":"0: 未发生故障\n1: 发生故障","default_literal":"0b00","invalid_literal":null,"remark":null,"source_node":null,"derived_layout":{"start_lsb0":25,"end_lsb0":25,"span_bits":1,"length_consistent":true},"issue_refs":[]}
{"source_row":51,"signal_name":"EHB_EPB_FAU_NoValueError","description":"常开电磁阀异常","parameter_no":"64361-7","byte_order":"intel","declared_length_bits":1,"start":"4.3","end":"4.3","factor":null,"offset":null,"logical_range":"0~0x1","physical_range":"0~1","unit":"bit","value_description":"0: 未发生故障\n1: 发生故障","default_literal":"0b00","invalid_literal":null,"remark":null,"source_node":null,"derived_layout":{"start_lsb0":26,"end_lsb0":26,"span_bits":1,"length_consistent":true},"issue_refs":[]}
{"source_row":52,"signal_name":"EHB_EPB_FAU_NcValueError","description":"常闭电磁阀异常","parameter_no":"64361-8","byte_order":"intel","declared_length_bits":1,"start":"4.4","end":"4.4","factor":null,"offset":null,"logical_range":"0~0x1","physical_range":"0~1","unit":"bit","value_description":"0: 未发生故障\n1: 发生故障","default_literal":"0b00","invalid_literal":null,"remark":null,"source_node":null,"derived_layout":{"start_lsb0":27,"end_lsb0":27,"span_bits":1,"length_consistent":true},"issue_refs":[]}
{"source_row":53,"signal_name":"EHB_EPB_FAU_SensorSupplyError","description":"传感器电源异常","parameter_no":"64361-9","byte_order":"intel","declared_length_bits":1,"start":"4.5","end":"4.5","factor":null,"offset":null,"logical_range":"0~0x1","physical_range":"0~1","unit":"bit","value_description":"0: 未发生故障\n1: 发生故障","default_literal":"0b00","invalid_literal":null,"remark":null,"source_node":null,"derived_layout":{"start_lsb0":28,"end_lsb0":28,"span_bits":1,"length_consistent":true},"issue_refs":[]}
{"source_row":54,"signal_name":"EHB_EPB_FAU_ParkingPresSensorError","description":"驻车系统液压信号异常","parameter_no":"64361-10","byte_order":"intel","declared_length_bits":1,"start":"4.6","end":"4.6","factor":null,"offset":null,"logical_range":"0~0x1","physical_range":"0~1","unit":"bit","value_description":"0: 未发生故障\n1: 发生故障","default_literal":"0b00","invalid_literal":null,"remark":null,"source_node":null,"derived_layout":{"start_lsb0":29,"end_lsb0":29,"span_bits":1,"length_consistent":true},"issue_refs":[]}
{"source_row":55,"signal_name":"EHB_EPB_FAU_ParkingPressureLow","description":"驻车液压过低","parameter_no":"64361-11","byte_order":"intel","declared_length_bits":1,"start":"4.7","end":"4.7","factor":null,"offset":null,"logical_range":"0~0x1","physical_range":"0~1","unit":"bit","value_description":"0: 未发生故障\n1: 发生故障","default_literal":"0b00","invalid_literal":null,"remark":null,"source_node":null,"derived_layout":{"start_lsb0":30,"end_lsb0":30,"span_bits":1,"length_consistent":true},"issue_refs":[]}
{"source_row":56,"signal_name":"EHB_EPB_FAU_LosOfParkingReqSignal","description":"驻车请求信号丢失","parameter_no":"64361-12","byte_order":"intel","declared_length_bits":1,"start":"4.8","end":"4.8","factor":null,"offset":null,"logical_range":"0~0x1","physical_range":"0~1","unit":"bit","value_description":"0: 未发生故障\n1: 发生故障","default_literal":"0b00","invalid_literal":null,"remark":null,"source_node":null,"derived_layout":{"start_lsb0":31,"end_lsb0":31,"span_bits":1,"length_consistent":true},"issue_refs":[]}
{"source_row":57,"signal_name":"EHB_EPB_FAU_ParkingReqRcError","description":"驻车请求rollcounter错误","parameter_no":"64361-13","byte_order":"intel","declared_length_bits":1,"start":"5.1","end":"5.1","factor":null,"offset":null,"logical_range":"0~0x1","physical_range":"0~1","unit":"bit","value_description":"0: 未发生故障\n1: 发生故障","default_literal":"0b00","invalid_literal":null,"remark":null,"source_node":null,"derived_layout":{"start_lsb0":32,"end_lsb0":32,"span_bits":1,"length_consistent":true},"issue_refs":[]}
{"source_row":58,"signal_name":"EHB_EPB_FAU_ParkingReqCsError","description":"驻车请求checksum错误","parameter_no":"64361-14","byte_order":"intel","declared_length_bits":1,"start":"5.2","end":"5.2","factor":null,"offset":null,"logical_range":"0~0x1","physical_range":"0~1","unit":"bit","value_description":"0: 未发生故障\n1: 发生故障","default_literal":"0b00","invalid_literal":null,"remark":null,"source_node":null,"derived_layout":{"start_lsb0":33,"end_lsb0":33,"span_bits":1,"length_consistent":true},"issue_refs":[]}
```

按原始起止位置计算的未分配 LSB0 位索引（仅用于覆盖校验，不代表默认填充值）：`[5,6,7,20,21,22,23,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63]`。仅按原表起止位置统计，不等同于已确认的保留位编码；计数器长度冲突仍保留。

### 3.4. Vehicle_Brake_Request（0x08FB1458）

来源：`接收信号需求` 第 4 行报文头；后续第 5～8 行为该报文信号。下列报文信息包含原来隐藏的 E～K 列；命名报文均涉及 I01。

```json
{
  "record_key": "RX-01",
  "source_sheet": "接收信号需求",
  "source_row": 4,
  "direction_from_ehb": "RX",
  "controller": "EHB",
  "sequence": 1,
  "message_name": "Vehicle_Brake_Request",
  "can_id_literal": "0x08FB1458",
  "pgn_cell": 65381,
  "priority_cell": 6,
  "data_page_cell": 0,
  "pdu_format_hex_cell": "FF",
  "pdu_specific_hex_cell": 65,
  "source_address_hex_cell": "2D",
  "custom_cell": "自定义",
  "send_type": "Periodic",
  "cycle_ms": 20,
  "dlc_bytes": 8,
  "remark": "遵循SAE J1939协议，保留位以0xFF填充",
  "source_node": null,
  "issue_refs": [
    "I01"
  ]
}
```

**完整信号记录（JSONL，每行一个信号；所有空值均显式保留）**

```jsonl
{"source_row":5,"signal_name":"VHL_ATB_BrakePressRequest","description":"请求制动力大小","parameter_no":"64276-01","byte_order":"intel","declared_length_bits":8,"start":"1.1","end":"1.8","factor":0.04,"offset":0,"logical_range":"0~0xC8","physical_range":"0~8","unit":"MPa","value_description":"0~0xC8: 请求制动力大小\n0xFE: 无效","default_literal":"0x00","invalid_literal":"0xFE","remark":null,"source_node":null,"derived_layout":{"start_lsb0":0,"end_lsb0":7,"span_bits":8,"length_consistent":true},"issue_refs":[]}
{"source_row":6,"signal_name":"VHL_ATB_BrakePressRequestFlag","description":"制动请求标志","parameter_no":"64276-02","byte_order":"intel","declared_length_bits":2,"start":"2.1","end":"2.2","factor":1,"offset":0,"logical_range":"0~0x3","physical_range":"0~3","unit":"bit","value_description":"00: 无请求\n01: 有请求\n10: 未定义\n11: 无效","default_literal":"0x00","invalid_literal":"0x11","remark":null,"source_node":null,"derived_layout":{"start_lsb0":8,"end_lsb0":9,"span_bits":2,"length_consistent":true},"issue_refs":["I03"]}
{"source_row":7,"signal_name":"VHL_ATB_RollingCounter","description":"滚动计数器","parameter_no":"64276-03","byte_order":"intel","declared_length_bits":8,"start":"7.5","end":"7.8","factor":1,"offset":0,"logical_range":"0-0xF","physical_range":"0-15","unit":null,"value_description":null,"default_literal":"0x00","invalid_literal":null,"remark":null,"source_node":null,"derived_layout":{"start_lsb0":52,"end_lsb0":55,"span_bits":4,"length_consistent":false},"issue_refs":["I02","I07"]}
{"source_row":8,"signal_name":"VHL_ATB_CheckSum","description":"校验和","parameter_no":"64276-04","byte_order":"intel","declared_length_bits":8,"start":"8.1","end":"8.8","factor":1,"offset":0,"logical_range":"0-0xFF","physical_range":"0-256","unit":null,"value_description":"Checksum=(Byte0+Byte1+Byte2+……+Byte6) XOR 0xFF ","default_literal":"0x00","invalid_literal":null,"remark":null,"source_node":null,"derived_layout":{"start_lsb0":56,"end_lsb0":63,"span_bits":8,"length_consistent":true},"issue_refs":["I05","I06"]}
```

按原始起止位置计算的未分配 LSB0 位索引（仅用于覆盖校验，不代表默认填充值）：`[10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51]`。仅按原表起止位置统计，不等同于已确认的保留位编码；计数器长度冲突仍保留。

### 3.5. Vehicle_Parking_Request（0x08FB1558）

来源：`接收信号需求` 第 9 行报文头；后续第 10～12 行为该报文信号。下列报文信息包含原来隐藏的 E～K 列；命名报文均涉及 I01。

```json
{
  "record_key": "RX-02",
  "source_sheet": "接收信号需求",
  "source_row": 9,
  "direction_from_ehb": "RX",
  "controller": "EHB",
  "sequence": 2,
  "message_name": "Vehicle_Parking_Request",
  "can_id_literal": "0x08FB1558",
  "pgn_cell": 65382,
  "priority_cell": 6,
  "data_page_cell": 0,
  "pdu_format_hex_cell": "FF",
  "pdu_specific_hex_cell": 66,
  "source_address_hex_cell": "2D",
  "custom_cell": "自定义",
  "send_type": "Periodic",
  "cycle_ms": 100,
  "dlc_bytes": 8,
  "remark": "遵循SAE J1939协议，保留位以0xFF填充",
  "source_node": null,
  "issue_refs": [
    "I01"
  ]
}
```

**完整信号记录（JSONL，每行一个信号；所有空值均显式保留）**

```jsonl
{"source_row":10,"signal_name":"VHL_EPB_ParkingRequest","description":"驻车请求","parameter_no":"64277-01","byte_order":"intel","declared_length_bits":2,"start":"1.1","end":"1.2","factor":1,"offset":0,"logical_range":"0~0x3","physical_range":"0~3","unit":null,"value_description":"0: 无请求\n1: 请求驻车\n2: 请求释放\n3: 无效","default_literal":"0x00","invalid_literal":"0x11","remark":null,"source_node":null,"derived_layout":{"start_lsb0":0,"end_lsb0":1,"span_bits":2,"length_consistent":true},"issue_refs":["I03"]}
{"source_row":11,"signal_name":"VHL_EPB_RollingCounter","description":"滚动计数器","parameter_no":"64277-02","byte_order":"intel","declared_length_bits":8,"start":"7.5","end":"7.8","factor":1,"offset":0,"logical_range":"0-0xF","physical_range":"0-15","unit":null,"value_description":null,"default_literal":"0x00","invalid_literal":null,"remark":null,"source_node":null,"derived_layout":{"start_lsb0":52,"end_lsb0":55,"span_bits":4,"length_consistent":false},"issue_refs":["I02","I07"]}
{"source_row":12,"signal_name":"VHL_EPB_CheckSum","description":"校验和","parameter_no":"64277-03","byte_order":"intel","declared_length_bits":8,"start":"8.1","end":"8.8","factor":1,"offset":0,"logical_range":"0-0xFF","physical_range":"0-256","unit":null,"value_description":"Checksum=(Byte0+Byte1+Byte2+……+Byte6) XOR 0xFF ","default_literal":"0x00","invalid_literal":null,"remark":null,"source_node":null,"derived_layout":{"start_lsb0":56,"end_lsb0":63,"span_bits":8,"length_consistent":true},"issue_refs":["I05","I06"]}
```

按原始起止位置计算的未分配 LSB0 位索引（仅用于覆盖校验，不代表默认填充值）：`[2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51]`。仅按原表起止位置统计，不等同于已确认的保留位编码；计数器长度冲突仍保留。

## 4. 补充接收项目：原始分组行与五个条目

### 4.1 第 13 行“仍然需要的项目”

以下是独立保留的原始行，**不是**为第 14～18 行构造的共同报文头。尤其不要把其中的周期 100、PGN 65226、PDU FE/CA 或源地址 2D 填充到这些不同 ID 上。

```json
{
  "sheet": "接收信号需求",
  "row": 13,
  "cells": {
    "A": "EHB",
    "E": 65226,
    "F": 6,
    "G": 0,
    "H": "FE",
    "I": "CA",
    "J": "2D",
    "K": "J1939",
    "L": "Periodic",
    "M": 100,
    "N": "TBD",
    "O": "仍然需要的项目",
    "AD": "遵循SAE J1939协议，保留位以0xFF填充"
  }
}
```

### 4.2. 车辆车速

来源：`接收信号需求!第 14 行`。报文名称、周期、DLC 等未提供的字段保持 null；D 列 `暂无` 不视为有效 CAN ID。

```json
{
  "message": {
    "record_key": "RX-S01",
    "source_sheet": "接收信号需求",
    "source_row": 14,
    "direction_from_ehb": "RX",
    "controller": null,
    "sequence": null,
    "message_name": null,
    "can_id_literal": "0x0CFD0358",
    "pgn_cell": null,
    "priority_cell": null,
    "data_page_cell": null,
    "pdu_format_hex_cell": null,
    "pdu_specific_hex_cell": null,
    "source_address_hex_cell": null,
    "custom_cell": null,
    "send_type": null,
    "cycle_ms": null,
    "dlc_bytes": null,
    "remark": null,
    "source_node": null,
    "issue_refs": [
      "I08"
    ]
  },
  "signal": {
    "source_row": 14,
    "signal_name": "车辆车速",
    "description": null,
    "parameter_no": "64771-03",
    "byte_order": "intel",
    "declared_length_bits": 16,
    "start": "5.1",
    "end": "6.8",
    "factor": 0.1,
    "offset": null,
    "logical_range": "0~0x9C4",
    "physical_range": "0~250.0",
    "unit": "km/h",
    "value_description": "0-250.0 km/h；0xFFFF：忽略",
    "default_literal": "0x00",
    "invalid_literal": "0xFFFF",
    "remark": null,
    "source_node": null,
    "derived_layout": {
      "start_lsb0": 32,
      "end_lsb0": 47,
      "span_bits": 16,
      "length_consistent": true
    },
    "issue_refs": [
      "I08"
    ]
  }
}
```

### 4.3. 其他液压管路的压力（如果有）

来源：`接收信号需求!第 15 行`。报文名称、周期、DLC 等未提供的字段保持 null；D 列 `暂无` 不视为有效 CAN ID。

```json
{
  "message": {
    "record_key": "RX-S02",
    "source_sheet": "接收信号需求",
    "source_row": 15,
    "direction_from_ehb": "RX",
    "controller": null,
    "sequence": null,
    "message_name": null,
    "can_id_literal": "暂无",
    "pgn_cell": null,
    "priority_cell": null,
    "data_page_cell": null,
    "pdu_format_hex_cell": null,
    "pdu_specific_hex_cell": null,
    "source_address_hex_cell": null,
    "custom_cell": null,
    "send_type": null,
    "cycle_ms": null,
    "dlc_bytes": null,
    "remark": null,
    "source_node": null,
    "issue_refs": [
      "I08"
    ]
  },
  "signal": {
    "source_row": 15,
    "signal_name": "其他液压管路的压力（如果有）",
    "description": null,
    "parameter_no": null,
    "byte_order": null,
    "declared_length_bits": null,
    "start": null,
    "end": null,
    "factor": null,
    "offset": null,
    "logical_range": null,
    "physical_range": null,
    "unit": null,
    "value_description": null,
    "default_literal": null,
    "invalid_literal": null,
    "remark": null,
    "source_node": null,
    "derived_layout": null,
    "issue_refs": [
      "I08"
    ]
  }
}
```

### 4.4. 点火钥匙状态 (改为：高压上电状态)

来源：`接收信号需求!第 16 行`。报文名称、周期、DLC 等未提供的字段保持 null；D 列 `暂无` 不视为有效 CAN ID。

```json
{
  "message": {
    "record_key": "RX-S03",
    "source_sheet": "接收信号需求",
    "source_row": 16,
    "direction_from_ehb": "RX",
    "controller": null,
    "sequence": null,
    "message_name": null,
    "can_id_literal": "0x0CFD0058",
    "pgn_cell": null,
    "priority_cell": null,
    "data_page_cell": null,
    "pdu_format_hex_cell": null,
    "pdu_specific_hex_cell": null,
    "source_address_hex_cell": null,
    "custom_cell": null,
    "send_type": null,
    "cycle_ms": null,
    "dlc_bytes": null,
    "remark": null,
    "source_node": null,
    "issue_refs": [
      "I08"
    ]
  },
  "signal": {
    "source_row": 16,
    "signal_name": "点火钥匙状态 (改为：高压上电状态)",
    "description": null,
    "parameter_no": "64768-03",
    "byte_order": "intel",
    "declared_length_bits": 8,
    "start": "3.1",
    "end": "3.8",
    "factor": 1,
    "offset": null,
    "logical_range": null,
    "physical_range": null,
    "unit": null,
    "value_description": "0x00=高压断电；\n0x01=高压上电；\n0xFF：忽略",
    "default_literal": null,
    "invalid_literal": null,
    "remark": null,
    "source_node": null,
    "derived_layout": {
      "start_lsb0": 16,
      "end_lsb0": 23,
      "span_bits": 8,
      "length_consistent": true
    },
    "issue_refs": [
      "I08"
    ]
  }
}
```

### 4.5. 制动踏板状态

来源：`接收信号需求!第 17 行`。报文名称、周期、DLC 等未提供的字段保持 null；D 列 `暂无` 不视为有效 CAN ID。

```json
{
  "message": {
    "record_key": "RX-S04",
    "source_sheet": "接收信号需求",
    "source_row": 17,
    "direction_from_ehb": "RX",
    "controller": null,
    "sequence": null,
    "message_name": null,
    "can_id_literal": "0x0CFD0158",
    "pgn_cell": null,
    "priority_cell": null,
    "data_page_cell": null,
    "pdu_format_hex_cell": null,
    "pdu_specific_hex_cell": null,
    "source_address_hex_cell": null,
    "custom_cell": null,
    "send_type": null,
    "cycle_ms": null,
    "dlc_bytes": null,
    "remark": null,
    "source_node": null,
    "issue_refs": [
      "I08"
    ]
  },
  "signal": {
    "source_row": 17,
    "signal_name": "制动踏板状态",
    "description": null,
    "parameter_no": "64769-05",
    "byte_order": "intel",
    "declared_length_bits": 8,
    "start": "4.1",
    "end": "4.8",
    "factor": 1,
    "offset": null,
    "logical_range": "0~0x64",
    "physical_range": "0-100",
    "unit": "%",
    "value_description": "0-100；FF=忽略",
    "default_literal": null,
    "invalid_literal": null,
    "remark": null,
    "source_node": null,
    "derived_layout": {
      "start_lsb0": 24,
      "end_lsb0": 31,
      "span_bits": 8,
      "length_consistent": true
    },
    "issue_refs": [
      "I08"
    ]
  }
}
```

### 4.6. 车辆当前档位

来源：`接收信号需求!第 18 行`。报文名称、周期、DLC 等未提供的字段保持 null；D 列 `暂无` 不视为有效 CAN ID。

```json
{
  "message": {
    "record_key": "RX-S05",
    "source_sheet": "接收信号需求",
    "source_row": 18,
    "direction_from_ehb": "RX",
    "controller": null,
    "sequence": null,
    "message_name": null,
    "can_id_literal": "0x08FD0258",
    "pgn_cell": null,
    "priority_cell": null,
    "data_page_cell": null,
    "pdu_format_hex_cell": null,
    "pdu_specific_hex_cell": null,
    "source_address_hex_cell": null,
    "custom_cell": null,
    "send_type": null,
    "cycle_ms": null,
    "dlc_bytes": null,
    "remark": null,
    "source_node": null,
    "issue_refs": [
      "I08"
    ]
  },
  "signal": {
    "source_row": 18,
    "signal_name": "车辆当前档位",
    "description": null,
    "parameter_no": "64770-04",
    "byte_order": "intel",
    "declared_length_bits": 16,
    "start": "5.1",
    "end": "6.8",
    "factor": null,
    "offset": null,
    "logical_range": null,
    "physical_range": null,
    "unit": null,
    "value_description": "ASCII    （R, N, D, P）；0xFFFF：忽略",
    "default_literal": null,
    "invalid_literal": null,
    "remark": null,
    "source_node": null,
    "derived_layout": {
      "start_lsb0": 32,
      "end_lsb0": 47,
      "span_bits": 16,
      "length_consistent": true
    },
    "issue_refs": [
      "I08",
      "I09"
    ]
  }
}
```

## 5. 原表编码、单位和校验说明

### 5.1 压力、故障数量、状态与忽略值

发送表中压力类信号的精度为 0.1 MPa、偏移量 0、逻辑范围 0～0xC8、物理范围 0～20 MPa，默认字面量为 0b00，无效字面量 0xFE。制动压力请求 `VHL_ATB_BrakePressRequest` 的精度则为 **0.04 MPa**，偏移量 0，逻辑范围同为 0～0xC8，但物理范围为 **0～8 MPa**，默认 0x00，无效 0xFE。不要混用反馈压力与请求压力的精度。来源：发送!第 9～13、26、47 行；接收!第 5 行。

只有在精度和偏移量均已明确时，才可按常见线性候选关系 `physical = raw * factor + offset` 组织换算；这是对所列数值范围的一致性解释。未填的精度/偏移量保持未知，不因为“通常如此”而补成 1/0；无效值、忽略值、未定义值应先按原始描述区分，不能参与正常物理量换算。

三个故障数量信号均写范围 0～13，但其 0xFE/0xE 无效码存在 I04。1 bit 故障标志的原文是“0: 未发生故障；1: 发生故障”，精度、偏移量和无效码多为空；这些空白均已保留。其他 2/3 bit 状态必须逐条使用各自的 `value_description`，不能套用同一枚举。

补充接收信号中的 `0xFFFF：忽略`、`0xFF：忽略`、`FF=忽略` 原样保留；它们不能一律翻译成设备故障。车速 AC14 同时填写 0xFFFF，AA14 明确写忽略。高压上电状态和制动踏板状态虽在 AA 列写有忽略码，但 AC 列为空，结构化记录没有擅自回填。

### 5.2 保留位

`接收信号需求!AD4`、`AD9`、`AD13` 的原文均为：

```text
遵循SAE J1939协议，保留位以0xFF填充
```

这条说明明确出现在两个请求报文的报文头，以及第 13 行补充分组记录。两个请求报文的保留位应保留该要求，但已定义信号必须按自身值覆盖相应位；不能把全部报文置零，也不能把有效信号的默认值改成全 1。第 13 行对后续补充项目的适用范围需确认。发送表报文头未写同样备注，不能把该填充规则无条件推广到三个发送报文。滚动计数器位长冲突解决前，不提供可直接发送的最终字节布局。

### 5.3 两个请求报文的校验和原式

来源：`接收信号需求!AA8` 和 `AA12`；两处内容完全相同，下行保留尾随空格：

```text
Checksum=(Byte0+Byte1+Byte2+……+Byte6) XOR 0xFF 
```

按本文位置约定，校验和位于第 8 字节（8.1～8.8），原式使用 `Byte0`～`Byte6` 指代此前七个字节。原文没有 CRC 多项式，也未定义中间求和宽度；不能替换成 CRC，也不能未经说明补上溢出规则。字段默认 0x00、精度 1、偏移 0，长度/范围和求和溢出问题分别见 I05、I06。滚动计数器的默认值、范围、长度冲突和行为缺项见 I02、I07。

## 6. 原表冲突与待确认项（不静默修订）

### I01 · 报文 ID 与隐藏的报文分解栏不一致

来源：发送!D4:J4、D25:J25、D44:J44；接收!D4:J4、D9:J9。其中“发送”“接收”分别指 `发送信号需求`、`接收信号需求`。

**原表事实/检查结果：** 五个命名报文的原始 ID 拆分结果与 E 列 PGN、F 列优先级、H 列 PDU 格式、I 列特定 PDU、J 列源地址存在差异。G 列数据页均为 0。下文给出原值与按明确位掩码计算的结果；计算值不是修订值。

**编程处理边界：** 保留两套原始证据。不得依据隐藏栏重建并覆盖 ID，也不得反向覆盖这些栏。需设备方确认哪组定义有效。

### I02 · 滚动计数器长度冲突

来源：接收!S7:U7、X7:Y7；S11:U11、X11:Y11。其中“发送”“接收”分别指 `发送信号需求`、`接收信号需求`。

**原表事实/检查结果：** 声明长度为 8 bit，但起止位置为 7.5～7.8，仅覆盖 4 bit；逻辑范围 0-0xF、物理范围 0-15。

**编程处理边界：** 不得把 8 自动改为 4。按起点 52 和声明长度 8 编码会延伸到第 8 字节并与校验和重叠；需确认真实长度与位置。

### I03 · 2 位请求信号的无效值越界

来源：接收!S6、AA6、AC6；S10、AA10、AC10。其中“发送”“接收”分别指 `发送信号需求`、`接收信号需求`。

**原表事实/检查结果：** VHL_ATB_BrakePressRequestFlag 和 VHL_EPB_ParkingRequest 的声明长度均为 2 bit；AC 列均写 0x11，即十六进制 17，无法放入 2 位。AA6 写 11: 无效；AA10 写 3: 无效。

**编程处理边界：** 候选含义可能是二进制 11（数值 3），但未经确认不得替换。尤其禁止简单截断：0x11 & 0x3 = 1，会变为有请求或请求驻车。

### I04 · 4 位故障数量信号存在两种无效值

来源：发送!S14、AA14、AC14；S28、AA28、AC28；S48、AA48、AC48。其中“发送”“接收”分别指 `发送信号需求`、`接收信号需求`。

**原表事实/检查结果：** 三个故障数量信号均为 4 bit，AA 列写“0xFE: 无效”，AC 列写“0xE”。0xFE 无法由 4 位表示。

**编程处理边界：** 原文两处都保留，不能静默改写 AA，也不能把该信号扩成 8 位。需确认无效码。

### I05 · 校验和物理范围超过 8 位最大值

来源：接收!S8、X8:Y8；S12、X12:Y12。其中“发送”“接收”分别指 `发送信号需求`、`接收信号需求`。

**原表事实/检查结果：** 两个校验和信号长度为 8 bit，逻辑范围为 0-0xFF，物理范围原文却为 0-256；8 位最大整数为 255。

**编程处理边界：** 保留原文 0-256 并提示范围矛盾，不擅自改为 0-255。

### I06 · 校验和求和溢出规则未写明

来源：接收!AA8、AA12。其中“发送”“接收”分别指 `发送信号需求`、`接收信号需求`。

**原表事实/检查结果：** 原式为 Checksum=(Byte0+Byte1+Byte2+……+Byte6) XOR 0xFF，未明确求和中间值宽度、溢出/截断规则。

**编程处理边界：** 不得把未经确认的取低 8 位规则写成原表要求。若采用候选 (sum(bytes[0:7]) & 0xFF) ^ 0xFF，必须标为待确认实现假设。此式不是 CRC 定义。

### I07 · 滚动计数器推进规则不完整

来源：接收!AB7、AB11、X7:Y7、X11:Y11。其中“发送”“接收”分别指 `发送信号需求`、`接收信号需求`。

**原表事实/检查结果：** 原表给出默认值 0x00 和范围 0～15，但未给出每次发送是否递增、回绕规则、重启恢复、丢帧容忍或异常判定阈值。

**编程处理边界：** 默认值可以原样保留；其余行为不得由“RollingCounter”名称自动推定。

### I08 · 补充接收项目缺少独立报文元数据

来源：接收!A13:AE18。其中“发送”“接收”分别指 `发送信号需求`、`接收信号需求`。

**原表事实/检查结果：** 第 13 行为“仍然需要的项目”，含周期 100、长度 TBD、PGN 65226 等模板式信息；第 14、16～18 行各写了独立 ID，第 15 行 ID 为“暂无”。这些行缺少独立报文名称、周期、DLC 等字段。

**编程处理边界：** 第 13 行作为独立原始记录保留，不向下填充到四个不同 ID。其保留位备注对后续项目的适用范围也需确认。缺失字段保持 null；“暂无”“TBD”不转换为 0。

### I09 · 档位的 16 位 ASCII 编码细节不完整

来源：接收!S18:U18、AA18。其中“发送”“接收”分别指 `发送信号需求`、`接收信号需求`。

**原表事实/检查结果：** 车辆当前档位长度为 16 bit、位置 5.1～6.8，值描述为“ASCII    （R, N, D, P）；0xFFFF：忽略”。

**编程处理边界：** 保留原文。需确认单字符在两个字节中的摆放、填充字节及完整有效编码；不得擅自指定 D\0 或 \0D。

### I10 · 列标题与单元格字面量进制不统一

来源：发送/接收!AB3:AC3；发送!I4、I25；接收!I4、I9。其中“发送”“接收”分别指 `发送信号需求`、`接收信号需求`。

**原表事实/检查结果：** AB/AC 表头标为 Hex，但许多单元格显式写 0b00、0b11、0b111；I 列表头为 Hex，但 65、66 存储为 Excel 数值。

**编程处理边界：** 0b 前缀按二进制字面量保留，0x 按十六进制保留。数值单元格 65、66 与其“Hex”表头一起保留，不把 65 擅自转成 0x41。

### I11 · 缺失通用实现参数

来源：整个工作簿。其中“发送”“接收”分别指 `发送信号需求`、`接收信号需求`。

**原表事实/检查结果：** 原表未明确提供 CAN 波特率、接口/通道、总线终端配置、超时阈值、完整安全状态机、制动/驻车仲裁、异常恢复、所有字段的有符号性以及位编号文字定义。

**编程处理边界：** 不得补写 250/500 kbit/s 等具体值；本文的 LSB0 是显式整理约定，不是原表额外声明。控制代码部署前需确认相关参数。

### I12 · 名称、忽略值、保留位及模板信息的语义边界

来源：发送!O35、O51:O52、O56；接收!AA14、AA16:AA18；模板版本管理；嵌入资源。其中“发送”“接收”分别指 `发送信号需求`、`接收信号需求`。

**原表事实/检查结果：** 存在 LosOf、NoValueError、NcValueError 等原始拼写；补充信号中写“忽略”而非统一“故障”；部分字段空白；模板版本、外部 DTC 引用与嵌入 DTC 图片不等同于当前 EHB 协议版本或新增报文。

**编程处理边界：** 保留原始拼写和术语；未知字段不填默认值；发送报文空余位不得套用接收请求的 0xFF 备注；不得从历史模板资源生成未经定义的 DTC/DM1 报文。

### 6.1 I01 的逐报文计算核对

以下只展示一个明确写出的 29 位标识符拆分算法，用于核对原表，不作为原表字段的替代。原表的 SAE J1939 备注是来源事实；位掩码及结果是本文的整理推导。此处九个 ID 算出的 PF 均为 FB 或 FD，计算 PGN 时采用下列式子。

```text
priority = (can_id >> 26) & 0x7
data_page = (can_id >> 24) & 0x1
pf = (can_id >> 16) & 0xFF
ps = (can_id >> 8) & 0xFF
source_address = can_id & 0xFF
pgn_for_these_ids = (can_id >> 8) & 0x3FFFF
```

| 报文 | 原 ID | 原 PGN | 计算 PGN | 优先级 原/算 | PF 原/算 | PS 原/算 | 源地址 原/算 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| EHB_STORAGE | 0x08FB670E | 65381 | 64359 | 6 / 2 | FF / FB | 65 / 67 | 2D / 0E |
| EHB_AUTOBRAKE | 0x08FB680E | 65382 | 64360 | 6 / 2 | FF / FB | 66 / 68 | 2D / 0E |
| EHB_EPB | 0x08FB690E | 65226 | 64361 | 6 / 2 | FE / FB | CA / 69 | 2D / 0E |
| Vehicle_Brake_Request | 0x08FB1458 | 65381 | 64276 | 6 / 2 | FF / FB | 65 / 14 | 2D / 58 |
| Vehicle_Parking_Request | 0x08FB1558 | 65382 | 64277 | 6 / 2 | FF / FB | 66 / 15 | 2D / 58 |

五个命名报文的原优先级均为 6，而上述 ID 均算得 2。原 G 列数据页 0 与计算一致。原 Q 列参数编号前缀分别为 64359、64360、64361、64276、64277，与上述 ID 计算的 PGN 对应，但这只能作为核对线索，不能据此把参数编号改成标准整数 SPN，也不能判定隐藏列必须被删除。

| 补充接收项目 | 原 ID | 计算优先级 | 计算 PGN | 计算 PF / PS / 源地址 |
| --- | --- | --- | --- | --- |
| 车辆车速 | 0x0CFD0358 | 3 | 64771 | FD / 03 / 58 |
| 点火钥匙状态 (改为：高压上电状态) | 0x0CFD0058 | 3 | 64768 | FD / 00 / 58 |
| 制动踏板状态 | 0x0CFD0158 | 3 | 64769 | FD / 01 / 58 |
| 车辆当前档位 | 0x08FD0258 | 2 | 64770 | FD / 02 / 58 |

## 7. 隐藏工作表的全部内容

### 7.1 封面签字

原工作表隐藏。编号、归档号、文档名称、编制/校对/审核/批准及日期均为未填模板；不得从其他元数据自动补填。两个只有空格的单元格也是原始内容，完整空格见第 10 节。图片对象另见第 8 节。

| 单元格 | 原始内容（JSON 字符串保留空格） |
| --- | --- |
| A6 | "编号: " |
| J6 | "归档号:" |
| A7 | "                    " |
| A9 | "     " |
| A17 | "文档名称：             " |
| B26 | "编    制：" |
| D26 | "日期：" |
| F26 | "年" |
| H26 | "月" |
| J26 | "日" |
| B27 | "校  对：" |
| D27 | "日期：" |
| F27 | "年" |
| H27 | "月" |
| J27 | "日" |
| B28 | "审    核：" |
| D28 | "日期：" |
| F28 | "年" |
| H28 | "月" |
| J28 | "日" |
| B29 | "批    准：" |
| D29 | "日期：" |
| F29 | "年" |
| H29 | "月" |
| J29 | "日" |

### 7.2 其他

原工作表隐藏；仅 `A1` 有值，合并区域为 `A1:F1`：

```text
补充信息（本调查表中未包含的信息）
```

### 7.3 模板版本管理

原工作表隐藏。下表是**调查表模板版本**，不是已确认的 EHB 协议版本；日期按源表数字保留。版本说明提到但本文件中不存在的概述要求、故障代码表、网络管理调查、问题交互表等，不在本文件中臆造。

| 调查表版本 | 变更内容 | 修改者 | 时间 |
| --- | --- | --- | --- |
| V1.0 | 初版 | 高瑶瑶 | 20180928 |
| V1.1 | 1.在概述要求sheet中：添加12-16条要求：关于网络管理调查<br>2.删除最大延迟时间的注解<br>3.删除各个控制器分配的SPN及PGN<br> | 高瑶瑶 | 20181010 |
| V1.2 | 1.高亮必填栏<br>2.故障代码表中添加可疑参数SPN、故障设置条件、恢复条件、可能的故障原因等项信息。<br>3.添加测试可支持项（采纳隐藏） | 高瑶瑶 | 20181211 |
| V1.3 | 1.添加签字封面；<br>2.修改文本红色蓝色字体； | 高瑶瑶 | 20190404 |
| V1.4 | 1.根据恒润提供模板对概述要求及软硬件信息内容进行优化；<br>2.添加网络管理调查信息； <br>3.删除J1939-73调查表，单独新建文件<br>4.添加问题交互表，降低邮件交流的信息遗失 | 高瑶瑶 | 20200601 |

## 8. 嵌入图片：原件、转写及关联状态

三张图片均从源文件嵌入资源中逐字节提取，已核对 PNG 数据块校验值。以下图片相对路径在完整资料包中有效；仅使用 Markdown 时，下面的文字转写仍可独立读取。没有把图片中的内容自动并入 EHB 命名报文。

| 图片 | 尺寸 | 字节数 | 工作簿中的关联 | SHA-256 |
| --- | --- | --- | --- | --- |
| embedded_image_1.png | 886×354 | 49010 | 嵌入图像资源，未被当前五张工作表中的图片对象引用 | e6c2874b7ae05f549f113deca93c2aceaf1da51b9dfbc465a6cea706c478698c |
| embedded_image_2.png | 365×305 | 70230 | 封面签字：图片 3；替代文本 DFM-D；锚点 E8:G11 | 82e6b237d06a94ff8c7c33a67d06656c300f5a16188ddb1ab8ce0807065b3c80 |
| embedded_image_3.png | 262×51 | 5854 | 封面签字：图片 4；锚点 C36:I40 | 34420e09e4cdeb1b6afdcf4e3ad76cd79fcf443442f6418d4f791f6e1cc9168e |

### 8.1 嵌入图片 1：DTC 位定义图

![原文件嵌入的 DTC 位定义图](ehb-can_assets/embedded_image_1.png)

该图在嵌入图像资源表中存在，但当前五张工作表的图片对象未引用它。不能因此认定它是当前 EHB 的有效 DTC 发送报文布局，也不能遗漏图中的信息。下面逐栏转写图中文字，换行用 `<br>` 表示：

| 图中栏目 | 图中文字 |
| --- | --- |
| 总标题 | DTC |
| 字节 3 | SPN 低 8 位有效位<br>（第 8 位为最高有效位） |
| 字节 4 | SPN 第 2 字节<br>（第 8 位为最高有效位） |
| 字节 5 | SPN 高 3 位有效位<br>与 FMI 有效位<br>（第 8 位为 SPN 的最高有效位及第 5 位为 FMI 的最高有效位） |
| 字节 6 | 上部仅写“字节 6”，没有额外解释文字。 |

图下部的位分区为：SPN 跨字节 3 全部 8 位、字节 4 全部 8 位和字节 5 的位 8～6；FMI 为字节 5 的位 5～1；字节 6 的位 8 标为纵向排列的 `C`、`M`（CM），位 7～1 标为 OC。各字节底部均按从左到右 `8 7 6 5 4 3 2 1` 标位。图中未给出这些字段对应的当前 EHB 报文 ID、周期或校验规则。

### 8.2 嵌入图片 2：封面图标

![封面图标 DFM](ehb-can_assets/embedded_image_2.png)

视觉内容：上方红色圆形图形标记，下方黑色大字 `DFM`（M 上方带红色三角形）。当前对象名为“图片 3”，替代文本/图片名元数据为 `DFM-D`；位于隐藏表“封面签字”，锚点 E8:G11。

### 8.3 嵌入图片 3：封面页脚标识

![封面页脚标识](ehb-can_assets/embedded_image_3.png)

图片中文字完整转写：`东风特种装备事业部`；`DONGFENG SPECIAL EQUIPMENT DIVISION`。左侧有红色圆形图形标记。当前对象名为“图片 4”，位于隐藏表“封面签字”，锚点 C36:I40，超出该表有值单元格的末行，不能仅按单元格使用区域截取而遗漏。

## 9. 表格结构、样式提示、元数据和外部引用

### 9.1 隐藏行列与合并区域

“发送信号需求”和“接收信号需求”的 E～K 列均隐藏，已完整读取。发送表第 62～67、69～74 行隐藏；这些行无文本/数值，仅含空白和样式记录。接收表无隐藏行。其余三张表为整表隐藏。

**封面签字**：合并区域 `J6:K6`、`A17:K17`。

**发送信号需求**：合并区域 `B2:M2`、`N2:AD2`、`A62:A67`、`B62:N67`、`A69:A74`、`B69:N74`、`D5:D24`、`D26:D43`、`D45:D58`。

**接收信号需求**：合并区域 `B2:M2`、`N2:AD2`、`A22:A27`、`B22:N27`、`A29:A34`、`B29:N34`、`D5:D8`、`D10:D12`。

**其他**：合并区域 `A1:F1`。

**模板版本管理**：合并区域 无。

D5:D24、D26:D43、D45:D58 等合并区域的锚点本身没有重复填写 ID，报文 ID 实际位于其前一行的报文头。正文分组保留这一结构，不把合并空白当成新报文或丢失 ID。

### 9.2 非默认文字颜色

源表存在红色文字/数字，未提供“红色=删除/无效”的统一图例，因此没有按颜色筛除内容。下表列出全部 104 个包含非默认颜色内容的单元格及着色部分；其颜色索引均为 10（红色）。完整富文本字体分段、样式和底纹等保存在 `ehb-can.audit.json` 与源 XLS，不以颜色判断协议优先级。

| 工作表 | 单元格 | 着色部分（JSON） |
| --- | --- | --- |
| 发送信号需求 | D4 | [{"text":"FB670E","colour_index":10}] |
| 发送信号需求 | Q5 | [{"text":"64359-01","colour_index":10}] |
| 发送信号需求 | Q6 | [{"text":"64359-02","colour_index":10}] |
| 发送信号需求 | Q7 | [{"text":"64359-03","colour_index":10}] |
| 发送信号需求 | Q8 | [{"text":"64359-04","colour_index":10}] |
| 发送信号需求 | Q9 | [{"text":"64359-05","colour_index":10}] |
| 发送信号需求 | Q10 | [{"text":"64359-06","colour_index":10}] |
| 发送信号需求 | Q11 | [{"text":"64359-07","colour_index":10}] |
| 发送信号需求 | Q12 | [{"text":"64359-08","colour_index":10}] |
| 发送信号需求 | Q13 | [{"text":"64359-09","colour_index":10}] |
| 发送信号需求 | Q14 | [{"text":"64359-10","colour_index":10}] |
| 发送信号需求 | Q15 | [{"text":"64359-11","colour_index":10}] |
| 发送信号需求 | Q16 | [{"text":"64359-12","colour_index":10}] |
| 发送信号需求 | Q17 | [{"text":"64359-13","colour_index":10}] |
| 发送信号需求 | Q18 | [{"text":"64359-14","colour_index":10}] |
| 发送信号需求 | Q19 | [{"text":"64359-15","colour_index":10}] |
| 发送信号需求 | Q20 | [{"text":"64359-16","colour_index":10}] |
| 发送信号需求 | Q21 | [{"text":"64359-17","colour_index":10}] |
| 发送信号需求 | Q22 | [{"text":"64359-18","colour_index":10}] |
| 发送信号需求 | Q23 | [{"text":"64359-19","colour_index":10}] |
| 发送信号需求 | Q24 | [{"text":"64359-20","colour_index":10}] |
| 发送信号需求 | D25 | [{"text":"FB680E","colour_index":10}] |
| 发送信号需求 | E25 | [{"value":65382,"colour_index":10}] |
| 发送信号需求 | I25 | [{"value":66,"colour_index":10}] |
| 发送信号需求 | J25 | [{"text":"2D","colour_index":10}] |
| 发送信号需求 | Q26 | [{"text":"64360-1","colour_index":10}] |
| 发送信号需求 | Q27 | [{"text":"64360-2","colour_index":10}] |
| 发送信号需求 | Q28 | [{"text":"64360-3","colour_index":10}] |
| 发送信号需求 | Q29 | [{"text":"64360-4","colour_index":10}] |
| 发送信号需求 | Q30 | [{"text":"64360-5","colour_index":10}] |
| 发送信号需求 | Q31 | [{"text":"64360-6","colour_index":10}] |
| 发送信号需求 | Q32 | [{"text":"64360-7","colour_index":10}] |
| 发送信号需求 | Q33 | [{"text":"64360-8","colour_index":10}] |
| 发送信号需求 | Q34 | [{"text":"64360-9","colour_index":10}] |
| 发送信号需求 | Q35 | [{"text":"64360-10","colour_index":10}] |
| 发送信号需求 | Q36 | [{"text":"64360-11","colour_index":10}] |
| 发送信号需求 | Q37 | [{"text":"64360-12","colour_index":10}] |
| 发送信号需求 | Q38 | [{"text":"64360-13","colour_index":10}] |
| 发送信号需求 | Q39 | [{"text":"64360-14","colour_index":10}] |
| 发送信号需求 | Q40 | [{"text":"64360-15","colour_index":10}] |
| 发送信号需求 | Q41 | [{"text":"64360-16","colour_index":10}] |
| 发送信号需求 | Q42 | [{"text":"64360-17","colour_index":10}] |
| 发送信号需求 | Q43 | [{"text":"64360-18","colour_index":10}] |
| 发送信号需求 | D44 | [{"text":"FB690E","colour_index":10}] |
| 发送信号需求 | Q45 | [{"text":"64361-1","colour_index":10}] |
| 发送信号需求 | Q46 | [{"text":"64361-2","colour_index":10}] |
| 发送信号需求 | Q47 | [{"text":"64361-3","colour_index":10}] |
| 发送信号需求 | Q48 | [{"text":"64361-4","colour_index":10}] |
| 发送信号需求 | Q49 | [{"text":"64361-5","colour_index":10}] |
| 发送信号需求 | Q50 | [{"text":"64361-6","colour_index":10}] |
| 发送信号需求 | Q51 | [{"text":"64361-7","colour_index":10}] |
| 发送信号需求 | Q52 | [{"text":"64361-8","colour_index":10}] |
| 发送信号需求 | Q53 | [{"text":"64361-9","colour_index":10}] |
| 发送信号需求 | Q54 | [{"text":"64361-10","colour_index":10}] |
| 发送信号需求 | Q55 | [{"text":"64361-11","colour_index":10}] |
| 发送信号需求 | Q56 | [{"text":"64361-12","colour_index":10}] |
| 发送信号需求 | Q57 | [{"text":"64361-13","colour_index":10}] |
| 发送信号需求 | Q58 | [{"text":"64361-14","colour_index":10}] |
| 接收信号需求 | D4 | [{"text":"FB1","colour_index":10},{"text":"4","colour_index":10},{"text":"58","colour_index":10}] |
| 接收信号需求 | D9 | [{"text":"FB1558","colour_index":10}] |
| 接收信号需求 | E9 | [{"value":65382,"colour_index":10}] |
| 接收信号需求 | I9 | [{"value":66,"colour_index":10}] |
| 接收信号需求 | J9 | [{"text":"2D","colour_index":10}] |
| 接收信号需求 | D14 | [{"text":"0x0CFD0358","colour_index":10}] |
| 接收信号需求 | O14 | [{"text":"车辆车速","colour_index":10}] |
| 接收信号需求 | R14 | [{"text":"intel","colour_index":10}] |
| 接收信号需求 | S14 | [{"value":16,"colour_index":10}] |
| 接收信号需求 | T14 | [{"value":5.1,"colour_index":10}] |
| 接收信号需求 | U14 | [{"value":6.8,"colour_index":10}] |
| 接收信号需求 | V14 | [{"text":"0.1","colour_index":10}] |
| 接收信号需求 | X14 | [{"text":"0~0x9C4","colour_index":10}] |
| 接收信号需求 | Y14 | [{"text":"0~250.0","colour_index":10}] |
| 接收信号需求 | Z14 | [{"text":"km/h","colour_index":10}] |
| 接收信号需求 | AA14 | [{"text":"0-250.0 km/h","colour_index":10},{"text":"；","colour_index":10},{"text":"0xFFFF","colour_index":10},{"text":"：忽略","colour_index":10}] |
| 接收信号需求 | AB14 | [{"text":"0x00","colour_index":10}] |
| 接收信号需求 | AC14 | [{"text":"0xFFFF","colour_index":10}] |
| 接收信号需求 | D15 | [{"text":"暂无","colour_index":10}] |
| 接收信号需求 | O15 | [{"text":"其他液压管路的压力（如果有）","colour_index":10}] |
| 接收信号需求 | D16 | [{"text":"0x0CFD0058","colour_index":10}] |
| 接收信号需求 | O16 | [{"text":"点火钥匙状态","colour_index":10},{"text":" (","colour_index":10},{"text":"改为：高压上电状态","colour_index":10},{"text":")","colour_index":10}] |
| 接收信号需求 | R16 | [{"text":"intel","colour_index":10}] |
| 接收信号需求 | S16 | [{"value":8,"colour_index":10}] |
| 接收信号需求 | T16 | [{"value":3.1,"colour_index":10}] |
| 接收信号需求 | U16 | [{"value":3.8,"colour_index":10}] |
| 接收信号需求 | V16 | [{"value":1,"colour_index":10}] |
| 接收信号需求 | AA16 | [{"text":"0x00=","colour_index":10},{"text":"高压断电；\\n","colour_index":10},{"text":"0x01=","colour_index":10},{"text":"高压上电；\\n","colour_index":10},{"text":"0xFF","colour_index":10},{"text":"：忽略","colour_index":10}] |
| 接收信号需求 | D17 | [{"text":"0x0CFD0158","colour_index":10}] |
| 接收信号需求 | O17 | [{"text":"制动踏板状态","colour_index":10}] |
| 接收信号需求 | R17 | [{"text":"intel","colour_index":10}] |
| 接收信号需求 | S17 | [{"value":8,"colour_index":10}] |
| 接收信号需求 | T17 | [{"value":4.1,"colour_index":10}] |
| 接收信号需求 | U17 | [{"value":4.8,"colour_index":10}] |
| 接收信号需求 | V17 | [{"value":1,"colour_index":10}] |
| 接收信号需求 | X17 | [{"text":"0~0x64","colour_index":10}] |
| 接收信号需求 | Y17 | [{"text":"0-100","colour_index":10}] |
| 接收信号需求 | Z17 | [{"text":"%","colour_index":10}] |
| 接收信号需求 | AA17 | [{"text":"0-100","colour_index":10},{"text":"；","colour_index":10},{"text":"FF=","colour_index":10},{"text":"忽略","colour_index":10}] |
| 接收信号需求 | D18 | [{"text":"0x","colour_index":10},{"text":"08","colour_index":10},{"text":"FD0258","colour_index":10}] |
| 接收信号需求 | O18 | [{"text":"车辆当前档位","colour_index":10}] |
| 接收信号需求 | R18 | [{"text":"intel","colour_index":10}] |
| 接收信号需求 | S18 | [{"value":16,"colour_index":10}] |
| 接收信号需求 | T18 | [{"value":5.1,"colour_index":10}] |
| 接收信号需求 | U18 | [{"value":6.8,"colour_index":10}] |
| 接收信号需求 | AA18 | [{"text":"ASCII    ","colour_index":10},{"text":"（","colour_index":10},{"text":"R, N, D, P","colour_index":10},{"text":"）；","colour_index":10},{"text":"0xFFFF","colour_index":10},{"text":"：忽略","colour_index":10}] |

### 9.3 数据验证

两项下拉验证的原始列表内容为 `"Tx\u0000 Rx"`，列表项为 `"Tx"` 和 `" Rx"`（后一项带前导空格）。这些只是 AE 源节点列的表格填写约束，不构成已填写的源节点或报文方向。提示/错误标题及文本的存储值均为单个 NUL 字符 `"\u0000"`，不是额外用户可见说明。

```json
[
  {
    "sheet": "发送信号需求",
    "options": 786819,
    "strings": [
      "\u0000",
      "\u0000",
      "\u0000",
      "\u0000"
    ],
    "formula1_tokens_hex": "170600547800205278",
    "formula2_tokens_hex": "",
    "list_literal": "Tx\u0000 Rx",
    "list_choices": [
      "Tx",
      " Rx"
    ],
    "ranges": [
      "AE66:AE72",
      "AE33:AE33"
    ]
  },
  {
    "sheet": "接收信号需求",
    "options": 786819,
    "strings": [
      "\u0000",
      "\u0000",
      "\u0000",
      "\u0000"
    ],
    "formula1_tokens_hex": "170600547800205278",
    "formula2_tokens_hex": "",
    "list_literal": "Tx\u0000 Rx",
    "list_choices": [
      "Tx",
      " Rx"
    ],
    "ranges": [
      "AE26:AE32"
    ]
  }
]
```

### 9.4 文档属性

这些是源文件内嵌属性，不是上传时间，也不能据此判定 EHB 协议发布时间。文档标题为 `diagnostic questionnaire`；作者字段为空；最后保存者为 `SONG`；应用程序为 `Microsoft Excel`。工作簿另有写入者记录 `Song`（大小写按两处原值分别保留，记录后有填充空格）。存储时间转为 UTC 后：最后打印 2009-09-03 11:37:31，创建 1996-12-17 01:32:42，最后保存 2023-12-30 09:20:27。全部已读取属性如下，属性 ID 与原始类型同时保留：

```json
[
  {
    "stream": "\u0005SummaryInformation",
    "property_id": 1,
    "variant_type": 2,
    "value": 936
  },
  {
    "stream": "\u0005SummaryInformation",
    "property_id": 2,
    "variant_type": 30,
    "value": "diagnostic questionnaire"
  },
  {
    "stream": "\u0005SummaryInformation",
    "property_id": 4,
    "variant_type": 30,
    "value": ""
  },
  {
    "stream": "\u0005SummaryInformation",
    "property_id": 8,
    "variant_type": 30,
    "value": "SONG"
  },
  {
    "stream": "\u0005SummaryInformation",
    "property_id": 18,
    "variant_type": 30,
    "value": "Microsoft Excel"
  },
  {
    "stream": "\u0005SummaryInformation",
    "property_id": 11,
    "variant_type": 64,
    "value": {
      "filetime": 128964514510000000,
      "utc": "2009-09-03T11:37:31+00:00"
    }
  },
  {
    "stream": "\u0005SummaryInformation",
    "property_id": 12,
    "variant_type": 64,
    "value": {
      "filetime": 124952599620000000,
      "utc": "1996-12-17T01:32:42+00:00"
    }
  },
  {
    "stream": "\u0005SummaryInformation",
    "property_id": 13,
    "variant_type": 64,
    "value": {
      "filetime": 133484016270000000,
      "utc": "2023-12-30T09:20:27+00:00"
    }
  },
  {
    "stream": "\u0005SummaryInformation",
    "property_id": 19,
    "variant_type": 3,
    "value": 0
  },
  {
    "stream": "\u0005DocumentSummaryInformation",
    "property_id": 1,
    "variant_type": 2,
    "value": 936
  },
  {
    "stream": "\u0005DocumentSummaryInformation",
    "property_id": 14,
    "variant_type": 30,
    "value": ""
  },
  {
    "stream": "\u0005DocumentSummaryInformation",
    "property_id": 15,
    "variant_type": 30,
    "value": ""
  },
  {
    "stream": "\u0005DocumentSummaryInformation",
    "property_id": 23,
    "variant_type": 3,
    "value": 786432
  },
  {
    "stream": "\u0005DocumentSummaryInformation",
    "property_id": 11,
    "variant_type": 11,
    "value": false
  },
  {
    "stream": "\u0005DocumentSummaryInformation",
    "property_id": 16,
    "variant_type": 11,
    "value": false
  },
  {
    "stream": "\u0005DocumentSummaryInformation",
    "property_id": 19,
    "variant_type": 11,
    "value": false
  },
  {
    "stream": "\u0005DocumentSummaryInformation",
    "property_id": 22,
    "variant_type": 11,
    "value": false
  },
  {
    "stream": "\u0005DocumentSummaryInformation",
    "property_id": 13,
    "variant_type": 4126,
    "value": {
      "raw_hex": "0500000009000000b7e2c3e6c7a9d7d6000d000000b7a2cbcdd0c5bac5d0e8c7f3000d000000bdd3cad5d0c5bac5d0e8c7f30005000000c6e4cbfb000d000000c4a3b0e5b0e6b1beb9dcc0ed00",
      "decoded": [
        "封面签字",
        "发送信号需求",
        "接收信号需求",
        "其他",
        "模板版本管理"
      ]
    }
  },
  {
    "stream": "\u0005DocumentSummaryInformation",
    "property_id": 12,
    "variant_type": 4108,
    "value": {
      "raw_hex": "020000001e00000007000000b9a4d7f7b1ed000300000005000000",
      "decoded": [
        "工作表",
        5
      ]
    }
  }
]
```

### 9.5 外部工作簿引用及定义名称 DTC

源文件仍存储一个历史外部工作簿路径和其 11 个工作表名，并另有当前工作簿自引用记录。外部路径中的控制字符是 Excel 的编码内容，下面以 JSON 转义原样保留；本次未访问、未读取该外部路径，也没有把外部文件的内容当成本文件内容。

```json
[
  {
    "kind": "external_workbook",
    "sheet_count": 11,
    "encoded_path_raw": "\u0001\u0001@10.4.9.25\u0003Team\u0003项目\u0003S15\u0003S15+A16\u0003PEPS\u0003S15一键式PEPS资料----诊断列表、下线和售后流程、更新后的OPEN ISSUE\u0003DFMC S15+A16一键式PEPS诊断列表_V1.0_ 20120705(DFMC).xls",
    "sheet_names": [
      "Rev.History",
      "CAN Diag Msg ID",
      "Diag Services",
      "PID",
      "LID",
      "CID",
      "RID",
      "DTC",
      "Bootloader流程",
      "REF1 DTCStatus",
      "REF2 DiagConfiguration"
    ]
  },
  {
    "kind": "self_reference",
    "sheet_count": 5
  }
]
```

原表还含工作簿级定义名称 `DTC`。其引用的外部工作表索引值为 65535，不对应当前五张工作表中的有效下标；本次无法将该名称解析成当前有效工作表内容，因而仅保留定义信息，不扩展为协议报文。引用区域标记为 A1:I10。

```json
{
  "defined_names": [
    {
      "name": "DTC",
      "scope_sheet_index_1based": 0,
      "flags": 0,
      "formula_tokens_hex": "3b00000000090000000800",
      "externsheet_index": 0,
      "referenced_area": "A1:I10"
    }
  ],
  "externsheet_records": [
    {
      "supbook_index": 0,
      "first_sheet_index": 65535,
      "last_sheet_index": 65535
    },
    {
      "supbook_index": 1,
      "first_sheet_index": 0,
      "last_sheet_index": 0
    }
  ]
}
```

## 10. 逐行原始单元格账本（全部 1,009 个有值单元格）

以下是**原始值层**，没有向下填充、类型规范化、默认值补全、大小写修正或去除空白。每一行 JSONL 的 `sheet` 指工作表，`row` 是 1 起始行号，`cells` 的键是原始列字母，例如 `{"row":7,"cells":{"S":8}}` 对应 S7=8。字符串中的 `\n`、尾随空格及纯空格全部保留；数字仍为 JSON 数字。没有出现的单元格表示没有文本/数值，不代表零；样式空白另保存在审计文件。

### 原始账本 · 封面签字

```jsonl
{"sheet":"封面签字","row":6,"cells":{"A":"编号: ","J":"归档号:"}}
{"sheet":"封面签字","row":7,"cells":{"A":"                    "}}
{"sheet":"封面签字","row":9,"cells":{"A":"     "}}
{"sheet":"封面签字","row":17,"cells":{"A":"文档名称：             "}}
{"sheet":"封面签字","row":26,"cells":{"B":"编    制：","D":"日期：","F":"年","H":"月","J":"日"}}
{"sheet":"封面签字","row":27,"cells":{"B":"校  对：","D":"日期：","F":"年","H":"月","J":"日"}}
{"sheet":"封面签字","row":28,"cells":{"B":"审    核：","D":"日期：","F":"年","H":"月","J":"日"}}
{"sheet":"封面签字","row":29,"cells":{"B":"批    准：","D":"日期：","F":"年","H":"月","J":"日"}}
```

### 原始账本 · 发送信号需求

```jsonl
{"sheet":"发送信号需求","row":1,"cells":{"A":"整车通信网络信号矩阵                                                        "}}
{"sheet":"发送信号需求","row":2,"cells":{"A":"控制器","B":"Message Information 报文信息","N":"Signal Information 信号信息"}}
{"sheet":"发送信号需求","row":3,"cells":{"B":"序号","C":"报文名称","D":"报文ID","E":"PGN","F":"优先级","G":"数据页","H":"PDU格式（Hex）","I":"特定PDU（Hex）","J":"源地址(Hex)","K":"自定义","L":"报文发送类型","M":"报文周期(ms)","N":"报文数据长度（byte）","O":"信号名称","P":"信号描述","Q":"可疑参数编号","R":"字节次序","S":"信号长度（bit）","T":"起始位","U":"终止位","V":"精度","W":"偏移量","X":"最小值—最大值（logical value）","Y":"最小值—最大值（physical value）","Z":"信号单位","AA":"信号值描述","AB":"默认值（Hex）","AC":"无效值（Hex）","AD":"备注","AE":"源节点"}}
{"sheet":"发送信号需求","row":4,"cells":{"A":"EHB","B":1,"C":"EHB_STORAGE","D":"0x08FB670E","E":65381,"F":6,"G":0,"H":"FF","I":65,"J":"2D","K":"自定义","L":"Periodic","M":100,"N":8}}
{"sheet":"发送信号需求","row":5,"cells":{"O":"EHB_STO_StorageSystemStatus","P":"EHB蓄能系统工作状态","Q":"64359-01","R":"intel","S":2,"T":1.1,"U":1.2,"X":"0~0x3","Y":"0~3","Z":"bit","AA":"00: 加压关\n01: 加压开\n10: 错误\n11: 无效","AB":"0b00","AC":"0b11"}}
{"sheet":"发送信号需求","row":6,"cells":{"O":"EHB_STO_StorageSystemFault","P":"EHB蓄能系统故障状态","Q":"64359-02","R":"intel","S":2,"T":1.3,"U":1.4,"X":"0~0x3","Y":"0~3","Z":"bit","AA":"00: 无故障\n01: 警示\n10: 停车\n11: 无效","AB":"0b00","AC":"0b11"}}
{"sheet":"发送信号需求","row":7,"cells":{"O":"EHB_STO_StorageSystemLowPressureWarn","P":"EHB蓄能系统低压警示","Q":"64359-03","R":"intel","S":2,"T":1.5,"U":1.6,"X":"0~0x3","Y":"0~3","Z":"bit","AA":"00: 无警示\n01: 警示\n10: 停车\n11: 无效","AB":"0b00","AC":"0b11"}}
{"sheet":"发送信号需求","row":8,"cells":{"O":"EHB_STO_BrakeFluidPosition","P":"制动液位信号状态（储液罐）","Q":"64359-04","R":"intel","S":2,"T":1.7,"U":1.8,"X":"0~0x3","Y":"0~3","Z":"bit","AA":"00: 正常\n01: 过低\n10: 错误\n11: 无效","AB":"0b00","AC":"0b11"}}
{"sheet":"发送信号需求","row":9,"cells":{"O":"EHB_STO_PressureSensor1RawValue","P":"液压传感器1原始压力值","Q":"64359-05","R":"intel","S":8,"T":2.1,"U":2.8,"V":0.1,"W":0,"X":"0~0xC8","Y":"0~20","Z":"MPa","AA":"0~0xC8: 液压传感器1原始压力值\n0xFE: 无效","AB":"0b00","AC":"0xFE"}}
{"sheet":"发送信号需求","row":10,"cells":{"O":"EHB_STO_PressureSensor2RawValue","P":"液压传感器2原始压力值","Q":"64359-06","R":"intel","S":8,"T":3.1,"U":3.8,"V":0.1,"W":0,"X":"0~0xC8","Y":"0~20","Z":"MPa","AA":"0~0xC8: 液压传感器2原始压力值\n0xFE: 无效","AB":"0b00","AC":"0xFE"}}
{"sheet":"发送信号需求","row":11,"cells":{"O":"EHB_STO_PressureSensor3RawValue","P":"液压传感器3原始压力值","Q":"64359-07","R":"intel","S":8,"T":4.1,"U":4.8,"V":0.1,"W":0,"X":"0~0xC8","Y":"0~20","Z":"MPa","AA":"0~0xC8: 液压传感器3原始压力值\n0xFE: 无效","AB":"0b00","AC":"0xFE"}}
{"sheet":"发送信号需求","row":12,"cells":{"O":"EHB_STO_HighPressureAccumulatorA","P":"高压蓄能器压力A","Q":"64359-08","R":"intel","S":8,"T":5.1,"U":5.8,"V":0.1,"W":0,"X":"0~0xC8","Y":"0~20","Z":"MPa","AA":"0~0xC8: 高压蓄能器A压力值\n0xFE: 无效","AB":"0b00","AC":"0xFE"}}
{"sheet":"发送信号需求","row":13,"cells":{"O":"EHB_STO_HighPressureAccumulatorB","P":"高压蓄能器压力B","Q":"64359-09","R":"intel","S":8,"T":6.1,"U":6.8,"V":0.1,"W":0,"X":"0~0xC8","Y":"0~20","Z":"MPa","AA":"0~0xC8: 高压蓄能器B压力值\n0xFE: 无效","AB":"0b00","AC":"0xFE"}}
{"sheet":"发送信号需求","row":14,"cells":{"O":"EHB_STO_FailureNum","P":"蓄能系统当前故障数量","Q":"64359-10","R":"intel","S":4,"T":7.1,"U":7.4,"V":1,"W":0,"X":"0~0xD","Y":"0~13","Z":"count","AA":"0~0xD: 系统当前故障数\n0xFE: 无效","AB":"0b00","AC":"0xE"}}
{"sheet":"发送信号需求","row":15,"cells":{"O":"EHB_STO_FAU_PowerSupplyVoltageHigh","P":"电源电压过高","Q":"64359-11","R":"intel","S":1,"T":7.5,"U":7.5,"X":"0~0x1","Y":"0~1","Z":"bit","AA":"0: 未发生故障\n1: 发生故障","AB":"0b00"}}
{"sheet":"发送信号需求","row":16,"cells":{"O":"EHB_STO_FAU_PowerSupplyVoltageLow","P":"电源电压过低","Q":"64359-12","R":"intel","S":1,"T":7.6,"U":7.6,"X":"0~0x1","Y":"0~1","Z":"bit","AA":"0: 未发生故障\n1: 发生故障","AB":"0b00"}}
{"sheet":"发送信号需求","row":17,"cells":{"O":"EHB_STO_FAU_SensorSupplyError","P":"液压传感器电源异常","Q":"64359-13","R":"intel","S":1,"T":7.7,"U":7.7,"X":"0~0x1","Y":"0~1","Z":"bit","AA":"0: 未发生故障\n1: 发生故障","AB":"0b00"}}
{"sheet":"发送信号需求","row":18,"cells":{"O":"EHB_STO_FAU_Sensor1Error","P":"1号液压传感器信号异常","Q":"64359-14","R":"intel","S":1,"T":7.8,"U":7.8,"X":"0~0x1","Y":"0~1","Z":"bit","AA":"0: 未发生故障\n1: 发生故障","AB":"0b00"}}
{"sheet":"发送信号需求","row":19,"cells":{"O":"EHB_STO_FAU_Sensor2Error","P":"2号液压传感器信号异常","Q":"64359-15","R":"intel","S":1,"T":8.1,"U":8.1,"X":"0~0x1","Y":"0~1","Z":"bit","AA":"0: 未发生故障\n1: 发生故障","AB":"0b00"}}
{"sheet":"发送信号需求","row":20,"cells":{"O":"EHB_STO_FAU_Sensor3Error","P":"3号液压传感器信号异常","Q":"64359-16","R":"intel","S":1,"T":8.2,"U":8.2,"X":"0~0x1","Y":"0~1","Z":"bit","AA":"0: 未发生故障\n1: 发生故障","AB":"0b00"}}
{"sheet":"发送信号需求","row":21,"cells":{"O":"EHB_STO_FAU_SensorJudgmentFailure","P":"液压传感器信号无法解析","Q":"64359-17","R":"intel","S":1,"T":8.3,"U":8.3,"X":"0~0x1","Y":"0~1","Z":"bit","AA":"0: 未发生故障\n1: 发生故障","AB":"0b00"}}
{"sheet":"发送信号需求","row":22,"cells":{"O":"EHB_STO_FAU_MotorOpenLoad","P":"蓄能电机断路","Q":"64359-18","R":"intel","S":1,"T":8.4,"U":8.4,"X":"0~0x1","Y":"0~1","Z":"bit","AA":"0: 未发生故障\n1: 发生故障","AB":"0b00"}}
{"sheet":"发送信号需求","row":23,"cells":{"O":"EHB_STO_FAU_MotorCannotStop","P":"蓄能电机无法关闭","Q":"64359-19","R":"intel","S":1,"T":8.5,"U":8.5,"X":"0~0x1","Y":"0~1","Z":"bit","AA":"0: 未发生故障\n1: 发生故障","AB":"0b00"}}
{"sheet":"发送信号需求","row":24,"cells":{"O":"EHB_STO_FAU_MotorWorkTimeout","P":"蓄能电机加压超时","Q":"64359-20","R":"intel","S":1,"T":8.6,"U":8.6,"X":"0~0x1","Y":"0~1","Z":"bit","AA":"0: 未发生故障\n1: 发生故障","AB":"0b00"}}
{"sheet":"发送信号需求","row":25,"cells":{"A":"EHB","B":2,"C":"EHB_AUTOBRAKE","D":"0x08FB680E","E":65382,"F":6,"G":0,"H":"FF","I":66,"J":"2D","K":"自定义","L":"Periodic","M":100,"N":8}}
{"sheet":"发送信号需求","row":26,"cells":{"O":"EHB_ATB_ActualBrakePressure","P":"主动制动系统实际压力","Q":"64360-1","R":"intel","S":8,"T":1.1,"U":1.8,"V":0.1,"W":0,"X":"0~0xC8","Y":"0~20","Z":"MPa","AA":"0~0xC8: 主动制动系统实际压力\n0xFE: 无效","AB":"0b00","AC":"0xFE"}}
{"sheet":"发送信号需求","row":27,"cells":{"O":"EHB_ATB_AutoBrakeSystemStatus","P":"主动制动系统当前状态","Q":"64360-2","R":"intel","S":2,"T":2.1,"U":2.2,"V":1,"W":0,"X":"0~0x3","Y":"0~3","Z":"bit","AA":"0x0: 正常\n0x1: 抑制\n0x2: 故障","AB":"0b00","AC":"0b11"}}
{"sheet":"发送信号需求","row":28,"cells":{"O":"EHB_ATB_FailureNum","P":"主动制动系统当前故障数量","Q":"64360-3","R":"intel","S":4,"T":2.5,"U":2.8,"V":1,"W":0,"X":"0~0xD","Y":"0~13","Z":"count","AA":"0~0xD: 系统当前故障数\n0xFE: 无效","AB":"0b00","AC":"0xE"}}
{"sheet":"发送信号需求","row":29,"cells":{"O":"EHB_ATB_FAU_PowerSupplyVoltageHigh","P":"电源电压过高","Q":"64360-4","R":"intel","S":1,"T":3.1,"U":3.1,"X":"0~0x1","Y":"0~1","Z":"bit","AA":"0: 未发生故障\n1: 发生故障","AB":"0b00"}}
{"sheet":"发送信号需求","row":30,"cells":{"O":"EHB_ATB_FAU_PowerSupplyVoltageLow","P":"电源电压过低","Q":"64360-5","R":"intel","S":1,"T":3.2,"U":3.2,"X":"0~0x1","Y":"0~1","Z":"bit","AA":"0: 未发生故障\n1: 发生故障","AB":"0b00"}}
{"sheet":"发送信号需求","row":31,"cells":{"O":"EHB_ATB_FAU_CanBusOff","P":"CAN BUS OFF","Q":"64360-6","R":"intel","S":1,"T":3.3,"U":3.3,"X":"0~0x1","Y":"0~1","Z":"bit","AA":"0: 未发生故障\n1: 发生故障","AB":"0b00"}}
{"sheet":"发送信号需求","row":32,"cells":{"O":"EHB_ATB_FAU_SensorSupplyError","P":"传感器电源故障","Q":"64360-7","R":"intel","S":1,"T":3.4,"U":3.4,"X":"0~0x1","Y":"0~1","Z":"bit","AA":"0: 未发生故障\n1: 发生故障","AB":"0b00"}}
{"sheet":"发送信号需求","row":33,"cells":{"O":"EHB_ATB_FAU_MotorDriverError","P":"电机驱动故障","Q":"64360-8","R":"intel","S":1,"T":3.5,"U":3.5,"X":"0~0x1","Y":"0~1","Z":"bit","AA":"0: 未发生故障\n1: 发生故障","AB":"0b00"}}
{"sheet":"发送信号需求","row":34,"cells":{"O":"EHB_ATB_FAU_BrakePressSensorError","P":"制动液压信号异常","Q":"64360-9","R":"intel","S":1,"T":3.6,"U":3.6,"X":"0~0x1","Y":"0~1","Z":"bit","AA":"0: 未发生故障\n1: 发生故障","AB":"0b00"}}
{"sheet":"发送信号需求","row":35,"cells":{"O":"EHB_ATB_FAU_LosOfEHBSTOCom","P":"EHB蓄能器通讯丢失","Q":"64360-10","R":"intel","S":1,"T":3.7,"U":3.7,"X":"0~0x1","Y":"0~1","Z":"bit","AA":"0: 未发生故障\n1: 发生故障","AB":"0b00"}}
{"sheet":"发送信号需求","row":36,"cells":{"O":"EHB_ATB_FAU_MotorCommuteFail","P":"电机校准失败","Q":"64360-11","R":"intel","S":1,"T":3.8,"U":3.8,"X":"0~0x1","Y":"0~1","Z":"bit","AA":"0: 未发生故障\n1: 发生故障","AB":"0b00"}}
{"sheet":"发送信号需求","row":37,"cells":{"O":"EHB_ATB_FAU_MotorOpenLoad","P":"电机开路","Q":"64360-12","R":"intel","S":1,"T":4.1,"U":4.1,"X":"0~0x1","Y":"0~1","Z":"bit","AA":"0: 未发生故障\n1: 发生故障","AB":"0b00"}}
{"sheet":"发送信号需求","row":38,"cells":{"O":"EHB_ATB_FAU_MotorShort","P":"电机短路","Q":"64360-13","R":"intel","S":1,"T":4.2,"U":4.2,"X":"0~0x1","Y":"0~1","Z":"bit","AA":"0: 未发生故障\n1: 发生故障","AB":"0b00"}}
{"sheet":"发送信号需求","row":39,"cells":{"O":"EHB_ATB_FAU_MotorStall","P":"电机堵转","Q":"64360-14","R":"intel","S":1,"T":4.3,"U":4.3,"X":"0~0x1","Y":"0~1","Z":"bit","AA":"0: 未发生故障\n1: 发生故障","AB":"0b00"}}
{"sheet":"发送信号需求","row":40,"cells":{"O":"EHB_ATB_FAU_ValveBodyError","P":"阀体故障","Q":"64360-15","R":"intel","S":1,"T":4.4,"U":4.4,"X":"0~0x1","Y":"0~1","Z":"bit","AA":"0: 未发生故障\n1: 发生故障","AB":"0b00"}}
{"sheet":"发送信号需求","row":41,"cells":{"O":"EHB_ATB_FAU_LosOfBrakeReqSignal","P":"制动请求信号丢失","Q":"64360-16","R":"intel","S":1,"T":4.5,"U":4.5,"X":"0~0x1","Y":"0~1","Z":"bit","AA":"0: 未发生故障\n1: 发生故障","AB":"0b00"}}
{"sheet":"发送信号需求","row":42,"cells":{"O":"EHB_ATB_FAU_BrakeReqRcError","P":"制动请求rollcounter错误","Q":"64360-17","R":"intel","S":1,"T":4.6,"U":4.6,"X":"0~0x1","Y":"0~1","Z":"bit","AA":"0: 未发生故障\n1: 发生故障","AB":"0b00"}}
{"sheet":"发送信号需求","row":43,"cells":{"O":"EHB_ATB_FAU_BrakeReqCsError","P":"制动请求checksum错误","Q":"64360-18","R":"intel","S":1,"T":4.7,"U":4.7,"X":"0~0x1","Y":"0~1","Z":"bit","AA":"0: 未发生故障\n1: 发生故障","AB":"0b00"}}
{"sheet":"发送信号需求","row":44,"cells":{"A":"EHB","B":3,"C":"EHB_EPB","D":"0x08FB690E","E":65226,"F":6,"G":0,"H":"FE","I":"CA","J":"2D","K":"J1939","L":"Periodic","M":100,"N":8}}
{"sheet":"发送信号需求","row":45,"cells":{"O":"EHB_EPB_SystemStatus","P":"驻车系统工作状态","Q":"64361-1","R":"intel","S":"2","T":1.1,"U":1.2,"V":"1","W":"0","X":"0x0-0x3","Y":"0-3","Z":"bit","AA":"00: 正常\n01: 警示\n10: 停车\n11: 无效","AB":"0b00","AC":"0b11"}}
{"sheet":"发送信号需求","row":46,"cells":{"O":"EHB_EPB_ParkingStatus","P":"驻车系统驻车状态","Q":"64361-2","R":"intel","S":3,"T":"1.3","U":1.5,"V":"1","W":"0","X":"0x0-0x7","Y":"0-7","Z":"bit","AA":"0x0: 已夹紧\n0x1: 已释放\n0x2:  夹紧中\n0x3: 释放中\n0x4~0x6: 未知","AB":"0b00","AC":"0b111"}}
{"sheet":"发送信号需求","row":47,"cells":{"O":"EHB_EPB_ParkingPressure","P":"驻车系统液压","Q":"64361-3","R":"intel","S":8,"T":2.1,"U":2.8,"V":0.1,"W":0,"X":"0~0xC8","Y":"0~20","Z":"MPa","AA":"0~0xC8: 驻车系统液压\n0xFE: 无效","AB":"0b00","AC":"0xFE"}}
{"sheet":"发送信号需求","row":48,"cells":{"O":"EHB_EPB_FailureNum","P":"驻车系统当前故障数","Q":"64361-4","R":"intel","S":4,"T":3.1,"U":3.4,"V":1,"W":0,"X":"0~0xD","Y":"0~13","Z":"count","AA":"0~0xD: 系统当前故障数\n0xFE: 无效","AB":"0b00","AC":"0xE"}}
{"sheet":"发送信号需求","row":49,"cells":{"O":"EHB_EPB_FAU_PowerSupplyVoltageHigh","P":"电源电压过高","Q":"64361-5","R":"intel","S":1,"T":4.1,"U":4.1,"X":"0~0x1","Y":"0~1","Z":"bit","AA":"0: 未发生故障\n1: 发生故障","AB":"0b00"}}
{"sheet":"发送信号需求","row":50,"cells":{"O":"EHB_EPB_FAU_PowerSupplyVoltageLow","P":"电源电压过低","Q":"64361-6","R":"intel","S":1,"T":4.2,"U":4.2,"X":"0~0x1","Y":"0~1","Z":"bit","AA":"0: 未发生故障\n1: 发生故障","AB":"0b00"}}
{"sheet":"发送信号需求","row":51,"cells":{"O":"EHB_EPB_FAU_NoValueError","P":"常开电磁阀异常","Q":"64361-7","R":"intel","S":1,"T":4.3,"U":4.3,"X":"0~0x1","Y":"0~1","Z":"bit","AA":"0: 未发生故障\n1: 发生故障","AB":"0b00"}}
{"sheet":"发送信号需求","row":52,"cells":{"O":"EHB_EPB_FAU_NcValueError","P":"常闭电磁阀异常","Q":"64361-8","R":"intel","S":1,"T":4.4,"U":4.4,"X":"0~0x1","Y":"0~1","Z":"bit","AA":"0: 未发生故障\n1: 发生故障","AB":"0b00"}}
{"sheet":"发送信号需求","row":53,"cells":{"O":"EHB_EPB_FAU_SensorSupplyError","P":"传感器电源异常","Q":"64361-9","R":"intel","S":1,"T":4.5,"U":4.5,"X":"0~0x1","Y":"0~1","Z":"bit","AA":"0: 未发生故障\n1: 发生故障","AB":"0b00"}}
{"sheet":"发送信号需求","row":54,"cells":{"O":"EHB_EPB_FAU_ParkingPresSensorError","P":"驻车系统液压信号异常","Q":"64361-10","R":"intel","S":1,"T":4.6,"U":4.6,"X":"0~0x1","Y":"0~1","Z":"bit","AA":"0: 未发生故障\n1: 发生故障","AB":"0b00"}}
{"sheet":"发送信号需求","row":55,"cells":{"O":"EHB_EPB_FAU_ParkingPressureLow","P":"驻车液压过低","Q":"64361-11","R":"intel","S":1,"T":4.7,"U":4.7,"X":"0~0x1","Y":"0~1","Z":"bit","AA":"0: 未发生故障\n1: 发生故障","AB":"0b00"}}
{"sheet":"发送信号需求","row":56,"cells":{"O":"EHB_EPB_FAU_LosOfParkingReqSignal","P":"驻车请求信号丢失","Q":"64361-12","R":"intel","S":1,"T":4.8,"U":4.8,"X":"0~0x1","Y":"0~1","Z":"bit","AA":"0: 未发生故障\n1: 发生故障","AB":"0b00"}}
{"sheet":"发送信号需求","row":57,"cells":{"O":"EHB_EPB_FAU_ParkingReqRcError","P":"驻车请求rollcounter错误","Q":"64361-13","R":"intel","S":1,"T":5.1,"U":5.1,"X":"0~0x1","Y":"0~1","Z":"bit","AA":"0: 未发生故障\n1: 发生故障","AB":"0b00"}}
{"sheet":"发送信号需求","row":58,"cells":{"O":"EHB_EPB_FAU_ParkingReqCsError","P":"驻车请求checksum错误","Q":"64361-14","R":"intel","S":1,"T":5.2,"U":5.2,"X":"0~0x1","Y":"0~1","Z":"bit","AA":"0: 未发生故障\n1: 发生故障","AB":"0b00"}}
```

### 原始账本 · 接收信号需求

```jsonl
{"sheet":"接收信号需求","row":1,"cells":{"A":"整车通信网络信号矩阵                                                        "}}
{"sheet":"接收信号需求","row":2,"cells":{"A":"控制器","B":"Message Information 报文信息","N":"Signal Information 信号信息"}}
{"sheet":"接收信号需求","row":3,"cells":{"B":"序号","C":"报文名称","D":"报文ID\nXX=（00～FF）","E":"PGN","F":"优先级","G":"数据页","H":"PDU格式（Hex）","I":"特定PDU（Hex）","J":"源地址(Hex)","K":"自定义","L":"报文发送类型","M":"报文周期(ms)","N":"报文数据长度（byte）","O":"信号名称","P":"信号描述","Q":"可疑参数编号","R":"字节次序","S":"信号长度（bit）","T":"起始位","U":"终止位","V":"精度","W":"偏移量","X":"最小值—最大值（logical value）","Y":"最小值—最大值（physical value）","Z":"信号单位","AA":"信号值描述","AB":"默认值（Hex）","AC":"无效值（Hex）","AD":"备注","AE":"源节点"}}
{"sheet":"接收信号需求","row":4,"cells":{"A":"EHB","B":1,"C":"Vehicle_Brake_Request","D":"0x08FB1458","E":65381,"F":6,"G":0,"H":"FF","I":65,"J":"2D","K":"自定义","L":"Periodic","M":20,"N":8,"AD":"遵循SAE J1939协议，保留位以0xFF填充"}}
{"sheet":"接收信号需求","row":5,"cells":{"O":"VHL_ATB_BrakePressRequest","P":"请求制动力大小","Q":"64276-01","R":"intel","S":8,"T":1.1,"U":1.8,"V":0.04,"W":0,"X":"0~0xC8","Y":"0~8","Z":"MPa","AA":"0~0xC8: 请求制动力大小\n0xFE: 无效","AB":"0x00","AC":"0xFE"}}
{"sheet":"接收信号需求","row":6,"cells":{"O":"VHL_ATB_BrakePressRequestFlag","P":"制动请求标志","Q":"64276-02","R":"intel","S":2,"T":2.1,"U":2.2,"V":1,"W":0,"X":"0~0x3","Y":"0~3","Z":"bit","AA":"00: 无请求\n01: 有请求\n10: 未定义\n11: 无效","AB":"0x00","AC":"0x11"}}
{"sheet":"接收信号需求","row":7,"cells":{"O":"VHL_ATB_RollingCounter","P":"滚动计数器","Q":"64276-03","R":"intel","S":8,"T":7.5,"U":7.8,"V":1,"W":0,"X":"0-0xF","Y":"0-15","AB":"0x00"}}
{"sheet":"接收信号需求","row":8,"cells":{"O":"VHL_ATB_CheckSum","P":"校验和","Q":"64276-04","R":"intel","S":8,"T":8.1,"U":8.8,"V":1,"W":0,"X":"0-0xFF","Y":"0-256","AA":"Checksum=(Byte0+Byte1+Byte2+……+Byte6) XOR 0xFF ","AB":"0x00"}}
{"sheet":"接收信号需求","row":9,"cells":{"A":"EHB","B":2,"C":"Vehicle_Parking_Request","D":"0x08FB1558","E":65382,"F":6,"G":0,"H":"FF","I":66,"J":"2D","K":"自定义","L":"Periodic","M":100,"N":8,"AD":"遵循SAE J1939协议，保留位以0xFF填充"}}
{"sheet":"接收信号需求","row":10,"cells":{"O":"VHL_EPB_ParkingRequest","P":"驻车请求","Q":"64277-01","R":"intel","S":2,"T":1.1,"U":1.2,"V":1,"W":0,"X":"0~0x3","Y":"0~3","AA":"0: 无请求\n1: 请求驻车\n2: 请求释放\n3: 无效","AB":"0x00","AC":"0x11"}}
{"sheet":"接收信号需求","row":11,"cells":{"O":"VHL_EPB_RollingCounter","P":"滚动计数器","Q":"64277-02","R":"intel","S":8,"T":7.5,"U":7.8,"V":1,"W":0,"X":"0-0xF","Y":"0-15","AB":"0x00"}}
{"sheet":"接收信号需求","row":12,"cells":{"O":"VHL_EPB_CheckSum","P":"校验和","Q":"64277-03","R":"intel","S":8,"T":8.1,"U":8.8,"V":1,"W":0,"X":"0-0xFF","Y":"0-256","AA":"Checksum=(Byte0+Byte1+Byte2+……+Byte6) XOR 0xFF ","AB":"0x00"}}
{"sheet":"接收信号需求","row":13,"cells":{"A":"EHB","E":65226,"F":6,"G":0,"H":"FE","I":"CA","J":"2D","K":"J1939","L":"Periodic","M":100,"N":"TBD","O":"仍然需要的项目","AD":"遵循SAE J1939协议，保留位以0xFF填充"}}
{"sheet":"接收信号需求","row":14,"cells":{"D":"0x0CFD0358","O":"车辆车速","Q":"64771-03","R":"intel","S":16,"T":5.1,"U":6.8,"V":"0.1","X":"0~0x9C4","Y":"0~250.0","Z":"km/h","AA":"0-250.0 km/h；0xFFFF：忽略","AB":"0x00","AC":"0xFFFF"}}
{"sheet":"接收信号需求","row":15,"cells":{"D":"暂无","O":"其他液压管路的压力（如果有）"}}
{"sheet":"接收信号需求","row":16,"cells":{"D":"0x0CFD0058","O":"点火钥匙状态 (改为：高压上电状态)","Q":"64768-03","R":"intel","S":8,"T":3.1,"U":3.8,"V":1,"AA":"0x00=高压断电；\n0x01=高压上电；\n0xFF：忽略"}}
{"sheet":"接收信号需求","row":17,"cells":{"D":"0x0CFD0158","O":"制动踏板状态","Q":"64769-05","R":"intel","S":8,"T":4.1,"U":4.8,"V":1,"X":"0~0x64","Y":"0-100","Z":"%","AA":"0-100；FF=忽略"}}
{"sheet":"接收信号需求","row":18,"cells":{"D":"0x08FD0258","O":"车辆当前档位","Q":"64770-04","R":"intel","S":16,"T":5.1,"U":6.8,"AA":"ASCII    （R, N, D, P）；0xFFFF：忽略"}}
```

### 原始账本 · 其他

```jsonl
{"sheet":"其他","row":1,"cells":{"A":"补充信息（本调查表中未包含的信息）"}}
```

### 原始账本 · 模板版本管理

```jsonl
{"sheet":"模板版本管理","row":1,"cells":{"A":"调查表版本","B":"变更内容","C":"修改者","D":"时间"}}
{"sheet":"模板版本管理","row":2,"cells":{"A":"V1.0","B":"初版","C":"高瑶瑶","D":20180928}}
{"sheet":"模板版本管理","row":3,"cells":{"A":"V1.1","B":"1.在概述要求sheet中：添加12-16条要求：关于网络管理调查\n2.删除最大延迟时间的注解\n3.删除各个控制器分配的SPN及PGN\n","C":"高瑶瑶","D":20181010}}
{"sheet":"模板版本管理","row":4,"cells":{"A":"V1.2","B":"1.高亮必填栏\n2.故障代码表中添加可疑参数SPN、故障设置条件、恢复条件、可能的故障原因等项信息。\n3.添加测试可支持项（采纳隐藏）","C":"高瑶瑶","D":20181211}}
{"sheet":"模板版本管理","row":5,"cells":{"A":"V1.3","B":"1.添加签字封面；\n2.修改文本红色蓝色字体；","C":"高瑶瑶","D":20190404}}
{"sheet":"模板版本管理","row":6,"cells":{"A":"V1.4","B":"1.根据恒润提供模板对概述要求及软硬件信息内容进行优化；\n2.添加网络管理调查信息； \n3.删除J1939-73调查表，单独新建文件\n4.添加问题交互表，降低邮件交流的信息遗失","C":"高瑶瑶","D":20200601}}
```

## 11. 完整性与一致性校验结果

校验的是提取/整理后的内容是否与源文件一致，以及已知矛盾是否继续可见；**不是**将源协议认证为正确，也不表示设备实测通过。

```json
{
  "source_sha256": "3ebdb6be23daca54af07329d112f115cd989365ab3881ec7e2e113358fb676e0",
  "source_size_bytes": 213504,
  "sheet_count": 5,
  "hidden_sheet_count": 3,
  "nonempty_value_cells": 1009,
  "string_cells": 741,
  "numeric_cells": 268,
  "whitespace_only_cells": [
    [
      "封面签字",
      "A7",
      20
    ],
    [
      "封面签字",
      "A9",
      5
    ]
  ],
  "explicit_blank_cells": 2507,
  "merged_ranges": 20,
  "shared_strings": 367,
  "all_shared_strings_referenced": true,
  "named_messages": 5,
  "named_message_signals": 59,
  "supplemental_items": 5,
  "total_signal_or_pending_items": 64,
  "tx_signals": 52,
  "rx_named_signals": 7,
  "position_length_conflicts": [
    {
      "sheet": "接收信号需求",
      "row": 7
    },
    {
      "sheet": "接收信号需求",
      "row": 11
    }
  ],
  "formula_cells": 0,
  "cell_comment_records": 0,
  "cell_hyperlink_records": 0,
  "data_validations": 2,
  "defined_names": 1,
  "embedded_pngs": 3,
  "png_crc_verified": true,
  "raw_ledger_cell_roundtrip_equal": true,
  "normalized_signal_fields_verified": true,
  "source_protocol_conflicts_resolved": false
}
```

原始账本已按工作表、单元格地址、数值/文本类型逐项回读比对；64 个信号/待定义项目的所有 O～AE 字段已逐项与原单元格核对。367 条共享字符串均被实际单元格引用，741 次字符串引用全部解析，跨记录中文字符串未截断。已按起止位置检查所有有位定义的信号：两处位长冲突仍明确保留；按起止位置计算未发现命名报文内的信号区间重叠，这不消除两处计数器按“声明 8 位”编码时产生的重叠风险。

资料包内 `ehb-can.normalized.json` 对应第 3～6 节结构化记录；`ehb-can.raw-cells.jsonl` 对应第 10 节；`ehb-can.audit.json` 另外保存行列/空白记录、全部字体和单元格样式、富文本分段、格式串、合并范围、图形结构和外部引用原始记录，并附全部 2,555 条 BIFF 记录及其他 OLE 流的原始十六进制载荷；`ehb-can.validation.json` 保存上述检查结果。原始 XLS 与三个原始 PNG 也一并保留，供复核非文本外观。

**供编程模型执行的最终边界：** 可以据此建立协议配置、读取原表字段、生成待确认问题和带显式假设的离线解析测试；存在冲突或缺项的字段必须继续携带问题编号，不得悄悄选择一方、通过位掩码隐藏错误，或声称已经获得可直接用于实车制动/驻车控制的完整协议。
