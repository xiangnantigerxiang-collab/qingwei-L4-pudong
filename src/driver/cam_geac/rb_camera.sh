#!/bin/bash

# 获取 /proc/version 的内容
version_info=$(cat /proc/version)

# 根据jp版本使用编解码库
if echo "$version_info" | grep -q "jp6"; then
    cp ./lib/codec_so/jp6/* ./lib
elif echo "$version_info" | grep -q "jp5.0.2"; then
    cp ./lib/codec_so/jp5.0.2/* ./lib
elif echo "$version_info" | grep -q "jp5.1.3"; then
    cp ./lib/codec_so/jp5.1.3/* ./lib
elif echo "$version_info" | grep -q "jp7"; then
    cp ./lib/codec_so/jp7/* ./lib
else
    cp ./lib/codec_so/jp5/* ./lib
fi

sudo cp ./lib/lib* ./tools/.jq/lib* /usr/lib
cd demo
./init.sh
cd ..
#定义一个数存放掩码
sum_of_squares=0
# 指定pipeline_id存储文件的路径  
pipeline_ids_file="pipeline_ids.txt"  
# 清空或创建pipeline_id存储文件  
> "$pipeline_ids_file"

#遍历读取相机的pipeline_id
function read_pipeline_id
{
    # 获取当前目录
    parent_dir="$(pwd)"
    while IFS= read -r line; do
        pipeline_id=$(echo "$line" | awk -F' ' '{split($1, id, "port_"); print id[2]}') # 分割出ID
        echo "$pipeline_id" >> "$pipeline_ids_file"
    done < <(./tools/.jq/jq -r '
        to_entries[] | 
        select(.key | startswith("port_")) | "\(.key) \(.value.deserial_index) \(.value.deserial_port)"' "$parent_dir/cfg/hb_j5dev.json"
        )
}

#计算掩码
function get_mask
{
while IFS= read -r pipeline_id; do   
    ((sum_of_squares += 2 ** pipeline_id))
done < "$pipeline_ids_file"  
rm "$pipeline_ids_file"
}

# 打印帮助信息的函数  
show_help(){
    echo  -e "使用方法:
    格式: ./rb_camera.sh \033[31m功能名称   相机掩码\033[0m
    例如: ./rb_camera.sh demo     15      
    含义: 获取相机数据，15表示 0000 1111 ，即 获取 0、1、2、3号相机数据\n"
    

    echo -e "\033[36m功能列表:
    功能1: demo 获取相机数据
    功能2: jpg  保存jpg数据
    功能3: rtsp 推流
    功能4: show OPENCV显示
    功能5: init 相机初始化（不包含取图）
    功能6: ros1_jpg ros1发布 jpg 格式相机数据
    功能7: ros1_yuv ros1发布 yuv 格式相机数据
    功能8: ros2_jpg ros2发布 jpg 格式相机数据
    功能9: ros2_yuv ros2发布 yuv 格式相机数据
    功能10: align 对齐相机数据（仅支持 D405/D415/D457 相机）\033[0m"
}

function camera_func
{
    sudo ./demo/camera_demo $param2
}

function init_func
{
    sudo ./demo/init_demo $param2
}

function rtsps_func
{
    echo " rtsp推流 "
    sudo ./demo/rtsps_demo $param2
}

function jpegenc_func
{
    echo "保存jpeg数据"
    sudo ./demo/jpegenc_demo $param2
}

function getparm_func
{
    echo "获取相机内参"
    sudo ./demo/getparm_demo $param2
}

function cicd_func
{
    echo "开始cd测试"
    sudo ./demo/cicd_demo $param2 $param3
}

function cicd_jpg_func
{
    echo "开始存储一张jpg图像"
    sudo ./demo/cicd_jpeg_demo $param2
}

function ros2_jpg_func
{
    echo "ros2 发布,需安装 ros2 环境,目前发布的是 jpeg 数据"
    ./demo/ros2_jpg_demo $param2
}

function ros1_jpg_func
{
    echo "ros1 发布,需安装 ros1 环境,并且提前运行 roscore ,目前发布的是 jpeg 数据"
    ./demo/ros1_jpg_demo $param2
}

function ros2_yuv_func
{
    echo "ros2 发布,需安装 ros2 环境,目前发布的是 yuv 数据"
    ./demo/ros2_yuv_demo $param2
}

function ros1_yuv_func
{
    echo "ros1 发布,需安装 ros1 环境,并且提前运行 roscore ,目前发布的是 yuv 数据"
    ./demo/ros1_yuv_demo $param2
}

function diag_test_func
{
    echo "诊断故障注入测试"
    sudo ./demo/diag_inject_demo $param2
}

function show_func
{
    echo "OPENCV显示"
    sudo ./demo/showimg_demo $param2
}

function align_func
{
    echo "D415 图像对齐"
    sudo ./demo/align_demo $param2
}

function stop_func
{
    echo "停止服务"
    killall camera_demo
    killall init_demo
    killall rtsps_demo
    killall jpegenc_demo    
}

read_pipeline_id
get_mask

# 初始化变量  
param1=""
param2=""
param3=""
  
# 定义一个函数来存储前三个参数  
store_params() {  
    if [ $# -ge 1 ]; then  
        param1=$1  
    fi 
    #如果$2未设置或者设置为空，则使用 'sum_of_squares'的值
    if [ -z "$2" ]; then  
        param2=$sum_of_squares
    else  
        param2=$2 
    fi
    if [ $# -ge 3 ]; then  
        param3=$3  
    fi  
}  

store_params "$@"

modprobe i2c-mux-pca954x

# 根据传入的参数调用不同的功能  
case $param1 in  
    "-h")  
        show_help  
        ;;  
    "demo")  
        camera_func  
        ;;  
    "init")  
        init_func  
        ;;  
    "rtsp")  
        rtsps_func  
        ;;  
    "jpg")  
        jpegenc_func  
        ;;  
    "parm")  
        getparm_func  
        ;;  
    "ros2_jpg")  
        ros2_jpg_func  
        ;;  
    "ros1_jpg")
        ros1_jpg_func
        ;;
    "ros2_yuv")  
        ros2_yuv_func  
        ;;  
    "ros1_yuv")
        ros1_yuv_func
        ;;
    "cicd")  
        cicd_func  
        cicd_jpg_func
        ;;
    "show")  
        show_func
        ;;
    "align")
        align_func
        ;;
    "diag")  
        diag_test_func
        ;;
    "stop")  
        stop_func
        ;;
    *)  
        echo "无效的功能名称 $param1 ，默认执行Init"  
        init_func  
        ;;  
esac


