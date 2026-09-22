## 一，适用版本

## 本工程使用方法

核对：2026-09-19。整车使用父目录 `rb_camera.sh ros1_jpg`，完整环境与运行目录见
[相机入口](../README.md)。独立demo调试需先确认设备接线、cfg和动态库，在本目录执行
`./camera_demo`；不要与正在运行的ROS相机实例同时取图。

## 注意事项与容易疏忽的点

- 下方供应商更换cfg示例含清空目录，执行前备份并确认设备型号；不是整车启动必做步骤。
- demo内可执行是现役启动依赖，不因未列入catkin而删除。
- 编码库和BSP/JetPack要配套，取图成功不等于ROS压缩图像话题已发布。
- 本轮仅添加工程说明，下方保留供应商历史原文。

## 随工程保留的上游说明

1.1 硬件版本: GEAC所有产品
	1.2 GEAC底软版本: T19K510SCE-R1.0-V4.5.0.2.beta 或之后版本(若版本不确定，可联系BSP确认)

## 二，文件夹结构
	
├── 7v_4v.sh  测试使用文件(7v和4v同时取图测试脚本)
├── camera_demo  相机取图demo
├── jpegenc_demo  相机取图,并进行jpeg编码，保存为jpg文件
├── rtsps_demo  相机取图，并进行rtsps推流
├── out/lib  相机驱动库以及依赖库
│   ├── libovx3cnew.so  x3c相机驱动库(包含后视,周视相机)
│   ├── libovx8bnew.so  x8b相机驱动库(包含前广,前窄相机)
│   └── libgeaccam.so  相机基础库
├── cfg  *_demo 读取配置文件路径
├── README.md  说明文件
├── Makefile  编译文件, 若需要推流，则可修改Makefile中的参数，重新编译
		
## 三，使用说明
	1 camera_demo 取图程序使用说明:
		1)相机接入域控,接法请根据提供的配置命名规则
		2)域控上电
		3)替换配置文件,如我要使用cfg_gac_4v_host2_ox01f_desaysv_4这份配置文件
			rm cfg/*
			cp cfg_2v_bus30_imx390_bus31_imx390/* cfg/
		4)执行./camera_demo
