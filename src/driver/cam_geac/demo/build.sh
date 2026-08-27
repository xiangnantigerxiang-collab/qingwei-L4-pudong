#!/bin/bash

# 备份原始 Makefile
cp Makefile Makefile.bak

# 编译函数
compile_project() {
    echo "编译 $1..."
    make clean && make -j
    echo "********$1 编译完成********"
}

# 更新 Makefile 中的变量
update_makefile_var() {
    sed -i "s/^$1.*/$1=$2/" Makefile
}

# 显示帮助信息
show_help() {
    echo "用法: $0 [选项]"
    echo "  -h          显示此帮助信息"
    echo "  1           编译 camera_demo"
    echo "  2           编译 rtsps_demo"
    echo "  3           编译 jpegenc_demo"
    echo "  4           编译 showing_demo"
    echo "  5           编译 init_demo"
    echo "  6           编译 ros1_jpg_demo"
    echo "  7           编译 ros2_jpg_demo"
    echo "  8           编译 ros1_yuv_demo"
    echo "  9           编译 ros2_yuv_demo"
    echo "  10          编译 align_demo(仅Inetl D405G/D415G/D457G使用)"
    echo "  无参数      编译所有 demo"
}

# 检查是否提供了参数
if [ "$#" -eq 0 ]; then
    # 没有参数，编译所有 demo
    echo "编译所有 demo..."
    update_makefile_var "SUPPORT_RTSPS" "n"
    update_makefile_var "SUPPORT_JPEGENC" "n"
    update_makefile_var "SUPPORT_ROS2_JPG" "n"
    update_makefile_var "SUPPORT_ROS2_YUV" "n"
    update_makefile_var "SUPPORT_SHOWIMG" "n"
    update_makefile_var "SUPPORT_ROS1_JPG" "n"
    update_makefile_var "SUPPORT_ROS1_YUV" "n"
    update_makefile_var "SUPPORT_D415_ALIGN" "n"
    compile_project "camera_demo"

    update_makefile_var "SUPPORT_RTSPS" "y"
    compile_project "rtsps_demo"

    update_makefile_var "SUPPORT_RTSPS" "n"
    update_makefile_var "SUPPORT_D415_ALIGN" "y"
    compile_project "align_demo"

    update_makefile_var "SUPPORT_D415_ALIGN" "n"
    update_makefile_var "SUPPORT_JPEGENC" "y"
    compile_project "jpegenc_demo"

    update_makefile_var "SUPPORT_ROS2_JPG" "y"
    compile_project "ros2_jpg_demo"

    update_makefile_var "SUPPORT_ROS2_JPG" "n"
    update_makefile_var "SUPPORT_ROS1_JPG" "y"
    compile_project "ros1_jpg_demo"

    update_makefile_var "SUPPORT_JPEGENC" "n"
    update_makefile_var "SUPPORT_ROS1_JPG" "n"
    update_makefile_var "SUPPORT_ROS2_YUV" "y"
    compile_project "ros2_yuv_demo"

    update_makefile_var "SUPPORT_ROS2_YUV" "n"
    update_makefile_var "SUPPORT_ROS1_YUV" "y"
    compile_project "ros1_yuv_demo"

    update_makefile_var "SUPPORT_ROS1_YUV" "n"
    update_makefile_var "SUPPORT_SHOWIMG" "y"
    compile_project "showing_demo"

    update_makefile_var "SUPPORT_SHOWIMG" "n"
    update_makefile_var "SUPPORT_INIT" "y"
    compile_project "init_demo"


else
    # 根据参数编译特定的 demo
    update_makefile_var "SUPPORT_RTSPS" "n"
    update_makefile_var "SUPPORT_JPEGENC" "n"
    update_makefile_var "SUPPORT_ROS2_JPG" "n"
    update_makefile_var "SUPPORT_ROS2_YUV" "n"
    update_makefile_var "SUPPORT_SHOWIMG" "n"
    update_makefile_var "SUPPORT_ROS1_JPG" "n"
    update_makefile_var "SUPPORT_ROS1_YUV" "n"
    update_makefile_var "SUPPORT_D415_ALIGN" "n"
    case $1 in
        1)
            compile_project "camera_demo"
            ;;
        2)
            update_makefile_var "SUPPORT_RTSPS" "y"
            compile_project "rtsps_demo"
            ;;
        3)
            update_makefile_var "SUPPORT_JPEGENC" "y"
            compile_project "jpegenc_demo"
            ;;
        4)
            update_makefile_var "SUPPORT_SHOWIMG" "y"
            compile_project "showing_demo"
            ;;
        5)
            update_makefile_var "SUPPORT_INIT" "y"
            compile_project "init_demo"
            ;;
        6)
            update_makefile_var "SUPPORT_JPEGENC" "y"
            update_makefile_var "SUPPORT_ROS1_JPG" "y"
            compile_project "ros1_demo"
            ;;
        7)
            update_makefile_var "SUPPORT_JPEGENC" "y"
            update_makefile_var "SUPPORT_ROS2_JPG" "y"
            compile_project "ros2_demo"
            ;;
        8)
            update_makefile_var "SUPPORT_ROS1_YUV" "y"
            compile_project "ros1_demo"
            ;;
        9)
            update_makefile_var "SUPPORT_ROS2_YUV" "y"
            compile_project "ros2_demo"
            ;;
        10)
            update_makefile_var "SUPPORT_D415_ALIGN" "y"
            compile_project "align_demo"
            ;;
        h|--help)
            show_help
            ;;
        *)
            echo "未知选项：$1"
            show_help
            ;;
    esac
fi

# 恢复 Makefile 到初始状态
mv Makefile.bak Makefile
echo "Makefile 已恢复到初始状态。"