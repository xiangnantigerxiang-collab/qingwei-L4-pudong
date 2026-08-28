# qingwei L4 monitor — 自动驾驶可视化(Web)

替代 `src/simview`(C++ simviewer + rviz)的浏览器端 3D 可视化。对标 `hmi/`
的架构:纯 Python stdlib + rospy(可选),**零 pip 依赖**,部署 = 拷贝目录。

- 车辆位置/朝向/速度、感知障碍物、规划/路由路径、停车点、托盘位置、
  地图三线、2 路 2D 补盲激光、**4 路 3D 感知点云**(rviz 从未显示过的增强)
- 3D 视角(OrbitControls 鼠标旋转/缩放/平移)、俯视 2D / 3D 环视 / 跟随车辆
  三个相机预设、每层独立开关(默认态照抄 robot.rviz)
- Three.js r147 vendored(`static/three.min.js` + `OrbitControls.js`,
  MIT,离线无 CDN;选 r147 因其是最后保有非 module examples/js 的版本)

## 部署(车载)

```bash
scp -r monitor/ nvidia@<orin-ip>:/home/nvidia/qingwei-L4-No2/
# 车端:
bash ~/qingwei-L4-No2/monitor/monitor.sh     # 默认 0.0.0.0:8081
```

浏览器打开 `http://<orin-ip>:8081`。与 hmi(8080) 并存互不影响;
改端口 `MONITOR_PORT=8082 bash monitor/monitor.sh`。

## 功能与 simview 对照

| simview/rviz 功能 | monitor 对应 | 语义差异 |
|---|---|---|
| 车辆黄框+红点+4行文字 | 相同(2.3/-0.8×±0.73m) | yaw=(90°-heading) 一致 |
| 障碍物青色 CUBE | 半透明+抬到 h/2 | dx/dy 互换一致;半透明为有意改进 |
| 规划绿线/路由黄线 | 相同 | 路径 x/y 不等长整帧丢弃,一致 |
| 停车点红箭头 | 相同 | stopAngle 有值时用于朝向(C++ 忽略恒指东,增强) |
| 托盘红点 | 相同 | 不含 C++ 里从未参与渲染的 hook_xg-2.8 死偏移 |
| 地图白中心线+紫双边 | 相同(±1m,>0.1m 抽稀) | 文件拷至 monitor/map/,不再依赖 simview 包 |
| 2D 补盲 LaserScan×2 | 按通道着色(左蓝右绿) | intensity 已解码暂未参与着色(rviz 为 intensity 灰度);**需配安装外参**(下节) |
| 3D 感知点云 | 新增(4 路,车体坐标) | rviz 未显示过 |
| 图层开关 | checkbox 面板 | 默认态照抄 robot.rviz(map/planning/grid 关) |
| TopDownOrtho 相机 | "俯视 2D"预设 | 跟随模式为增强 |

不订阅的话题:`/planning/obstacles`(死话题,advertise 但从不发布)、
`/palletpos` 以外的 C++ 注释掉的绘制(车轮/托盘矩形)。

## 2D 补盲激光外参标定(上实车做一次)

lakibeam 两路的 frame_id 都叫 `laser`,工程内无任何静态 tf——rviz 里这两路
从未正确落位。monitor 在服务端对每路做外参变换,配置在
`monitor_config.py` 的 `SCAN_EXTRINSICS`:

```python
"/back_left_scan":  {"x": 0.0, "y": 0.0, "yaw_deg": 0.0},
```

`x/y` 为传感器在车体系的位置(米),`yaw_deg` 为安装朝向偏转。标定方法:
车前/车后放已知距离的障碍物,对照页面点云弧的位置调外参直到吻合。

## CPU / 带宽调参(`monitor_config.py` 的 CLOUD 段)

4 路 PointCloud2 typed 订阅空转约 12-40MB/s 反序列化,是 CPU 大头,故:
- **活动门控**:cloud.bin 超 30s 无浏览器拉取自动退订,有请求再重订
- **节拍解析**:每路每 1s 解析一次(`parse_interval`)
- **跨步预抽稀**:解包前每 N 点取 1(`stride`,默认 2)
- **体素抽稀**:0.2m 格(`voxel`),每路上限 8000 点

总带宽约 4×8000×12B ≈ 380KB/帧@1Hz。若 Orin 占用偏高:调大 stride/
parse_interval,或调粗 voxel。实测方法:`top` 看 monitor 进程,
hmi 面板本身也有 CPU 显示。

## 设计否决记录

- **SSE/WebSocket 推送**:二进制过 SSE 需 base64(+33%),WebSocket 非
  stdlib,破坏与 hmi 的同构原则;1~2Hz fetch 对诊断视角足够
- **gzip**:点云噪声态数据压缩率仅 ~20-30%,Orin 上 400KB gzip 耗几十 ms
  CPU,得不偿失;带宽不够直接调粗 voxel
- **路径版本号门控**:/plan_path_msg 10Hz receding horizon 每帧必变,
  门控省不了流量反引入陈旧视图 bug;快照直带全量(cm 精度,~5-10KB)

## 排障:页面能开但状态点灰色/无数据(08-28 实车案例)

两个状态点(master/ros)只有收到快照数据才会变色——**都保持灰色 =
快照从未成功返回,或 3D 初始化失败拖死了轮询**(旧版 buildScene 抛异常
会使 init 中断、轮询不启动)。排查顺序:

```bash
# 1. 车端服务侧(区分服务端/浏览器侧):
curl -s -m 5 http://127.0.0.1:8081/api/snapshot | head -c 300; echo
#   有 JSON(ros_available/master_ok/...) -> 服务端正常, 看 2/3
#   卡住/空/异常                        -> 服务端问题, 看 4

# 2. 浏览器 F12 控制台红字 + 页面左上红色错误框(新版会自报):
#   "3D 初始化失败,已切换 2D 俯视渲染: ..." -> 浏览器 WebGL 不可用,
#      新版已降级为完整 2D 可视化,3D 需换浏览器/开硬件加速
#   "JS 错误: ..."                       -> 直接给出错误与行号
#   顶部红色横幅"服务连接断开"            -> /api/snapshot 不通, 看 4

# 3. roscore 未启动是正常降级(非故障): master 点红色 + 黄色横幅
#    "ROS master 失联", ROS 点绿色; 起车端栈后自动恢复

# 4. 部署新鲜度(旧副本缺修复):
grep -c "getBigUint64" monitor/static/index.html   # 应为 0
grep -c "3D 初始化失败" monitor/static/index.html       # 应为 1
```

## Firefox 车端 WebGL 修复(08-28 实车确认案例)

车端 Firefox 报"3D 初始化失败"时,新版会**自动切换 2D 俯视渲染**
(地图三线/路径/障碍/车辆/停车点/托盘/点云全有,滚轮缩放)——即使
WebGL 修不好也可用。2D 按数据变化重画并限制到最高 10 FPS,避免四路点云
在 Jetson 上按浏览器刷新率重复绘制造成高 CPU。要恢复 3D,按序尝试:

1. 地址栏进 `about:config`:
   - `webgl.disabled` = false
   - `webgl.force-enabled` = true(驱动黑名单时强制启用)
2. **Jetson/X11 关键一步**:Firefox 83+ 在 X11 上需 EGL 后端跑 WebGL,
   旧的 GLX 路径在 Tegra 上常创建失败:
   ```bash
   MOZ_X11_EGL=1 firefox
   # 或写进启动环境: export MOZ_X11_EGL=1
   ```
3. snap 版 Firefox 沙箱可能挡 GPU 访问 → 换 deb 版 firefox-esr 或
   chromium 之一
4. 验证:地址栏 `about:support` → 图形 → WebGL1/2 Renderer 应显示
   适配器名而非 Unavailable

## 坐标系(前端 MonitorMath,已单测锁定)

ROS 显示系(x东,y北,z上) → THREE(x右,y上,z南):
- 位置:`three = (x_ros, z_ros, -y_ros)`
- 朝向:`rotation.y = yaw_ros`(**不取负**;推导:R_y(φ)·e_x=(cosφ,0,-sinφ)
  与映射后的方向向量一致,φ=θ 直接相等——取负是最常见翻车点)
- 车辆/障碍 `yaw = normalize(90° - heading)·π/180`,与 simviewer C++ 一致

## 二进制协议(scan.bin / cloud.bin)

小端。头 20B:`QWMC`(4) + version u16 + 通道数 u16 + 总点数 u32 + 时刻ms u64;
每通道子头 8B:通道id u8 + flags u8 + 保留 u16 + 点数 u32;
每点 12B:x_mm i32 + y_mm i32 + z_mm i16 + intensity u8 + pad u8。
车体坐标(前端挂车辆组随位姿动)。Python/JS 两侧各有解码实现且被
同一 fixture 字节锁测试锁死(tests/test_full.py t05 与
tests/frontend_test.js F4)。

## 本机(无 ROS)调试与测试

```bash
cd monitor
python3 tests/test_full.py        # 伪 ROS 全栈 60 项
python3 tests/edge_test.py       # 边界值 23 项
node tests/frontend_test.js       # 前端无头 45 项(DOM+THREE/2D Canvas 桩)
python3 tests/chaos_test.py      # 混沌/浸泡 11 项(约 2 分钟)
python3 tests/fuzz_proto.py      # 协议双解码对账 fuzz(50 轮随机数据)
bash monitor.sh                   # 起服务,页面显示"无 ROS 环境"横幅
```

`tests/` 仅开发机用,**车载不部署**。

## 文件结构

| 文件 | 职责 |
|---|---|
| `monitor.sh` | 启动包装(source devel → python3) |
| `monitor_config.py` | 话题/外参/降采样/图层默认态配置 |
| `monitor_server.py` | HTTP 入口(静态+4 个 API) |
| `ros_visualizer.py` | rospy 桥:快照组装/地图解析/scan/cloud 二进制 |
| `map/view.csv` | 地图(拷自 simview,消除对 simview 包的依赖) |
| `static/index.html` | 前端单页(MonitorMath 纯函数+场景+循环) |
| `static/three.min.js`、`OrbitControls.js` | Three.js r147 vendored(MIT) |
| `tests/` | 伪 ROS 全栈 / 前端无头测试 |
