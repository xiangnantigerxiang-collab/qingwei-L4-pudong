CPP = aarch64-linux-gnu-g++
CC =  aarch64-linux-gnu-gcc

CFLAGS =  -g -I ./inc -I ../../../libcam/src/inc/develop/
LDFLAGS = -L../lib -L../../../libcam/out -L../../../libcam/out/lib   -ldl -lrt   -lpthread -ldl -lm -lgeaccam -Wl,-rpath=../lib -Wl,-rpath=../out -Wl,-rpath=../out/lib
SOURCES_DIR=./source
DIR_BIN = .

# 获取系统版本信息
version := $(shell cat /proc/version)

# 默认为空
codec_dir =

# 判断设备是jp5还是jp6
ifeq ($(findstring jp6,$(version)),jp6)
    LDFLAGS += -L../../lib/codec_so/jp6
else ifeq ($(findstring jp5.0.2,$(version)),jp5.0.2)
    LDFLAGS += -L../../lib/codec_so/jp5.0.2
else ifeq ($(findstring jp5.1.3,$(version)),jp5.1.3)
    LDFLAGS += -L../../lib/codec_so/jp5.1.3
else ifeq ($(findstring jp7,$(version)),jp7)
    LDFLAGS += -L../../lib/codec_so/jp7
else
    LDFLAGS += -L../../lib/codec_so/jp5
endif

SOURCES = $(foreach DIR,$(SOURCES_DIR),$(wildcard $(DIR)/*))
SOURCES_C = $(filter %.c,$(SOURCES))
SOURCES_CPP = $(filter %.cpp,$(SOURCES))
OBJS += $(SOURCES_C:%.c=%.o)
OBJS += $(SOURCES_CPP:%.cpp=%.o)
TARGET = camera_demo

## support list
SUPPORT_RTSPS=n
SUPPORT_JPEGENC=n
SUPPORT_ROS1_JPG=n
SUPPORT_ROS2_JPG=n
SUPPORT_SHOWIMG=y
RB_V4L2_DMA=y
SUPPORT_INIT=n
SUPPORT_ROS1_YUV=n
SUPPORT_ROS2_YUV=n
SUPPORT_D415_ALIGN=n

ifeq ($(SUPPORT_INIT),y)
CFLAGS += -DSUPPORT_INIT
TARGET = init_demo
endif

ifeq ($(RB_V4L2_DMA),y)
CFLAGS += -DRB_V4L2_DMA
LDFLAGS += -L/usr/lib/aarch64-linux-gnu/tegra/ -lnvbufsurface -lnvbufsurftransform
endif

ifeq ($(SUPPORT_SHOWIMG),y)
CFLAGS += -DSUPPORT_SHOWIMG
LDFLAGS += -lX11 -lyuv -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_imgcodecs -Wl,-rpath=../out -Wl,-rpath=../out/lib	
CFLAGS +=  -I ./inc/libyuv -I/usr/include/opencv4
CFLAGS += -I/usr/include/X11
LDFLAGS += -L/usr/lib/X11	
TARGET = showimg_demo	
endif

ifeq ($(SUPPORT_RTSPS),y)
CFLAGS += -DSUPPORT_RTSPS
LDFLAGS += -lvideoenc -lrtppay -lrtsps -lyuv -Wl,-rpath=../out -Wl,-rpath=../out/lib
CFLAGS +=  -I ./inc/libyuv
TARGET = rtsps_demo
endif

ifeq ($(SUPPORT_JPEGENC),y)
CFLAGS += -DSUPPORT_JPEGENC
LDFLAGS += -lyuv -ljpegenc -Wl,-rpath=../out -Wl,-rpath=../out/lib
CFLAGS +=  -I ./inc/libyuv
TARGET = jpegenc_demo
endif

ifeq ($(SUPPORT_ROS1_JPG),y)
CFLAGS += -DSUPPORT_ROS1_JPG
LDFLAGS += -lroscpp -limage_transport -lcv_bridge -lmessage_filters -lclass_loader -lroslib -lcpp_common -lrosconsole_log4cxx
LDFLAGS += -lrosconsole_backend_interface -lrospack -lrosconsole -lroscpp_serialization -lrostime -lxmlrpcpp
LDFLAGS +=  -L/opt/ros/noetic/lib/
CFLAGS +=  -I /opt/ros/noetic/include -I /opt/ros/noetic/include/rclcpp/
TARGET = ros1_jpg_demo
endif

ifeq ($(SUPPORT_ROS2_JPG),y)
CFLAGS += -DSUPPORT_ROS2_JPG
LDFLAGS += -lrclcpp -llibstatistics_collector -lstd_msgs__rosidl_typesupport_cpp -lrcl  -lstatistics_msgs__rosidl_typesupport_cpp  -lrcutils 
LDFLAGS += -ltracetools -lrcl_yaml_param_parser  -lrcl_interfaces__rosidl_typesupport_cpp  -lrmw_implementation
LDFLAGS += -lrmw -lrcpputils -lrosidl_typesupport_cpp -lrcl_interfaces__rosidl_typesupport_c 
LDFLAGS += -lrcl_logging_spdlog -lrcl_interfaces__rosidl_generator_c -lrosidl_runtime_c  -lrosidl_typesupport_c -lbuiltin_interfaces__rosidl_generator_c
LDFLAGS += -lrosgraph_msgs__rosidl_typesupport_cpp -lsensor_msgs__rosidl_typesupport_cpp
# jp5和jp6的ros2版本不一样，需要区分
ifeq ($(findstring jp5,$(version)),jp5)
    LDFLAGS +=  -L/opt/ros/foxy/lib/
	CFLAGS +=  -I /opt/ros/foxy/include -I /opt/ros/foxy/include/rclcpp/ 
else ifeq ($(findstring jp6,$(version)),jp6)
    LDFLAGS +=  -L/opt/ros/humble/lib/
	CFLAGS += -I /opt/ros/humble/include -I /opt/ros/humble/include/rclcpp/ -I /opt/ros/humble/include/rcl/ -I /opt/ros/humble/include/rcutils
	CFLAGS += -I /opt/ros/humble/include/rmw -I /opt/ros/humble/include/rcl_yaml_param_parser -I /opt/ros/humble/include/rcl_yaml_param_parser
	CFLAGS += -I /opt/ros/humble/include/rosidl_runtime_c -I /opt/ros/humble/include/rosidl_typesupport_interface -I /opt/ros/humble/include/rcpputils
	CFLAGS += -I /opt/ros/humble/include/builtin_interfaces -I /opt/ros/humble/include/rosidl_runtime_cpp -I /opt/ros/humble/include/tracetools
	CFLAGS += -I /opt/ros/humble/include/rcl_interfaces -I /opt/ros/humble/include/libstatistics_collector -I /opt/ros/humble/include/statistics_msgs
	CFLAGS += -I /opt/ros/humble/include/sensor_msgs -I /opt/ros/humble/include/std_msgs
else ifeq ($(findstring jp7,$(version)),jp7)
    LDFLAGS +=  -L/opt/ros/jazzy/lib/
	CFLAGS += -I /opt/ros/jazzy/include -I /opt/ros/jazzy/include/rclcpp/ -I /opt/ros/jazzy/include/rcl_interfaces -I /opt/ros/jazzy/include/rosidl_runtime_cpp
	CFLAGS += -I /opt/ros/jazzy/include/rosidl_runtime_c -I /opt/ros/jazzy/include/service_msgs -I /opt/ros/jazzy/include/builtin_interfaces
	CFLAGS += -I /opt/ros/jazzy/include/rosidl_typesupport_interface -I /opt/ros/jazzy/include/rcutils -I /opt/ros/jazzy/include/rcl/ -I /opt/ros/jazzy/include/rmw
	CFLAGS += -I /opt/ros/jazzy/include/rcpputils -I /opt/ros/jazzy/include/rcl_yaml_param_parser -I /opt/ros/jazzy/include/type_description_interfaces
	CFLAGS += -I /opt/ros/jazzy/include/rosidl_dynamic_typesupport -I /opt/ros/jazzy/include/tracetools -I /opt/ros/jazzy/include/libstatistics_collector
	CFLAGS += -I /opt/ros/jazzy/include/statistics_msgs -I /opt/ros/jazzy/include/rosidl_typesupport_introspection_cpp -I /opt/ros/jazzy/include/sensor_msgs
	CFLAGS += -I /opt/ros/jazzy/include/std_msgs
endif
TARGET = ros2_jpg_demo
endif


ifeq ($(SUPPORT_ROS1_YUV),y)
CFLAGS += -DSUPPORT_ROS1_YUV
LDFLAGS += -lroscpp -limage_transport -lcv_bridge -lmessage_filters -lclass_loader -lroslib -lcpp_common -lrosconsole_log4cxx
LDFLAGS += -lrosconsole_backend_interface -lrospack -lrosconsole -lroscpp_serialization -lrostime -lxmlrpcpp
LDFLAGS +=  -L/opt/ros/noetic/lib/
CFLAGS +=  -I /opt/ros/noetic/include -I /opt/ros/noetic/include/rclcpp/
TARGET = ros1_yuv_demo
endif

ifeq ($(SUPPORT_ROS2_YUV),y)
CFLAGS += -DSUPPORT_ROS2_YUV
LDFLAGS += -lrclcpp -llibstatistics_collector -lstd_msgs__rosidl_typesupport_cpp -lrcl  -lstatistics_msgs__rosidl_typesupport_cpp  -lrcutils 
LDFLAGS += -ltracetools -lrcl_yaml_param_parser  -lrcl_interfaces__rosidl_typesupport_cpp  -lrmw_implementation
LDFLAGS += -lrmw -lrcpputils -lrosidl_typesupport_cpp -lrcl_interfaces__rosidl_typesupport_c 
LDFLAGS += -lrcl_logging_spdlog -lrcl_interfaces__rosidl_generator_c -lrosidl_runtime_c  -lrosidl_typesupport_c -lbuiltin_interfaces__rosidl_generator_c
LDFLAGS += -lrosgraph_msgs__rosidl_typesupport_cpp -lsensor_msgs__rosidl_typesupport_cpp
# jp5和jp6的ros2版本不一样，需要区分
ifeq ($(findstring jp5,$(version)),jp5)
    LDFLAGS +=  -L/opt/ros/foxy/lib/
	CFLAGS +=  -I /opt/ros/foxy/include -I /opt/ros/foxy/include/rclcpp/ 
else ifeq ($(findstring jp6,$(version)),jp6)
    LDFLAGS +=  -L/opt/ros/humble/lib/
	CFLAGS += -I /opt/ros/humble/include -I /opt/ros/humble/include/rclcpp/ -I /opt/ros/humble/include/rcl/ -I /opt/ros/humble/include/rcutils
	CFLAGS += -I /opt/ros/humble/include/rmw -I /opt/ros/humble/include/rcl_yaml_param_parser -I /opt/ros/humble/include/rcl_yaml_param_parser
	CFLAGS += -I /opt/ros/humble/include/rosidl_runtime_c -I /opt/ros/humble/include/rosidl_typesupport_interface -I /opt/ros/humble/include/rcpputils
	CFLAGS += -I /opt/ros/humble/include/builtin_interfaces -I /opt/ros/humble/include/rosidl_runtime_cpp -I /opt/ros/humble/include/tracetools
	CFLAGS += -I /opt/ros/humble/include/rcl_interfaces -I /opt/ros/humble/include/libstatistics_collector -I /opt/ros/humble/include/statistics_msgs
	CFLAGS += -I /opt/ros/humble/include/sensor_msgs -I /opt/ros/humble/include/std_msgs
else ifeq ($(findstring jp7,$(version)),jp7)
    LDFLAGS +=  -L/opt/ros/jazzy/lib/
	CFLAGS += -I /opt/ros/jazzy/include -I /opt/ros/jazzy/include/rclcpp/ -I /opt/ros/jazzy/include/rcl_interfaces -I /opt/ros/jazzy/include/rosidl_runtime_cpp
	CFLAGS += -I /opt/ros/jazzy/include/rosidl_runtime_c -I /opt/ros/jazzy/include/service_msgs -I /opt/ros/jazzy/include/builtin_interfaces
	CFLAGS += -I /opt/ros/jazzy/include/rosidl_typesupport_interface -I /opt/ros/jazzy/include/rcutils -I /opt/ros/jazzy/include/rcl/ -I /opt/ros/jazzy/include/rmw
	CFLAGS += -I /opt/ros/jazzy/include/rcpputils -I /opt/ros/jazzy/include/rcl_yaml_param_parser -I /opt/ros/jazzy/include/type_description_interfaces
	CFLAGS += -I /opt/ros/jazzy/include/rosidl_dynamic_typesupport -I /opt/ros/jazzy/include/tracetools -I /opt/ros/jazzy/include/libstatistics_collector
	CFLAGS += -I /opt/ros/jazzy/include/statistics_msgs -I /opt/ros/jazzy/include/rosidl_typesupport_introspection_cpp -I /opt/ros/jazzy/include/sensor_msgs
	CFLAGS += -I /opt/ros/jazzy/include/std_msgs
endif
TARGET = ros2_yuv_demo
endif

ifeq ($(SUPPORT_D415_ALIGN),y)
SOURCES_CU = $(filter %.cu,$(SOURCES))
OBJS += $(SOURCES_CU:%.cu=%.o)
ifeq ($(findstring jp6,$(version)),jp6)
	# 指定NVCC编译器路径（适配aarch64架构的CUDA编译器）
	NVCC = /usr/local/cuda-12.2/bin/nvcc
	# NVCC编译参数（适配aarch64架构、链接CUDA库、指定CUDA头文件）
	NVCCFLAGS = -g -ccbin $(CPP) -m64 -arch=sm_72 \
				-I ./inc -I ../../../libcam/src/inc/develop/ -I ./inc/librealsense/ \
				-I /usr/local/cuda-12.2/targets/aarch64-linux/include/ \
				-L /usr/local/cuda-12.2/targets/aarch64-linux/lib64/ -lcudart
	CFLAGS += -DSUPPORT_D415_ALIGN 
	LDFLAGS += -Wl,-rpath=../out -Wl,-rpath=../out/lib  
	LDFLAGS += -L /usr/local/cuda-12.2/targets/aarch64-linux/lib/ -lcudart
	CFLAGS +=  -I ./inc/libyuv -I ./inc/librealsense/ -I /usr/local/cuda-12.2/targets/aarch64-linux/include/
else ifeq ($(findstring jp7,$(version)),jp7)
	NVCC = /usr/local/cuda-13.0/bin/nvcc
	NVCCFLAGS = -g -ccbin $(CPP) -m64 -arch=sm_90 \
				-I ./inc -I ../../../libcam/src/inc/develop/ -I ./inc/librealsense/ \
				-I /usr/local/cuda-13.0/targets/sbsa-linux/include/ \
				-L /usr/local/cuda-13.0/targets/sbsa-linux/lib64/ -lcudart
	CFLAGS += -DSUPPORT_D415_ALIGN 
	LDFLAGS += -Wl,-rpath=../out -Wl,-rpath=../out/lib  
	LDFLAGS += -L /usr/local/cuda-13.0/targets/sbsa-linux/lib/ -lcudart
	CFLAGS +=  -I ./inc/libyuv -I ./inc/librealsense/ -I /usr/local/cuda-13.0/targets/sbsa-linux/include/
endif
TARGET = align_demo
# CUDA文件编译规则
%.o: %.cu
	$(NVCC) $(NVCCFLAGS) -c -o $@ $<
endif

# all:$(TARGET)
BIN_TARGET = ${DIR_BIN}/${TARGET}


$(BIN_TARGET) : $(OBJS)
	$(CPP) -o $@ $^ $(LDFLAGS)

%.o: %.c
	$(CC) -c $(CFLAGS) $< -o $@

%.o: %.cpp
	$(CPP) -c $(CFLAGS) $< -o $@ 

.PHONY:clean RM
RM:
	@rm -f $(TARGET) ;\
	rm -f $(OBJS)
clean:
	@rm -f $(TARGET)
	 rm -f $(OBJS)



