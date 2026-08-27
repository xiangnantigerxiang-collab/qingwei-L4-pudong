#include <bits/stdc++.h>
#include <unistd.h> 
#include <sys/wait.h>
#include <sys/stat.h>
#include "cJSON.h"
#include "hb_vin_interface.h"
#include "mgr_camera.h"
#include "vin_log.h"
#include "camera_sys.h"
#include <atomic>
#include <signal.h>


// 数据类型映射表：将数字编码转换为对应的字符串描述
std::map<u32, const char*> DateTypeMap = {
    {0, "RAW"},        // 原始图像格式
    {1, "YUYV"},       // YUV422格式（YUV交错排列，常见于视频采集）
    {2, "UYVY"},       // YUV422格式（另一种字节排列顺序）
    {3, "RAW10"},
};

#if (defined(SUPPORT_ROS1_JPG) || defined(SUPPORT_ROS1_YUV))
std::atomic<bool> is_running(true);

void signal_handler(int sig)
{
    if (sig == SIGINT)
    {
        is_running = false;  // 设置标志，通知退出
    }
}
#endif

// 前置函数声明
static std::vector<int> get_hb_j5_json(std::vector<int>& channum, int& portmask); // 从JSON配置文件获取通道信息
int sensor_system(const char *pCmd); // 执行系统命令（通过子进程）
int reset_power(); // 重置相机电源（根据不同板卡型号）
const char* get_gpio_base_path(); // 获取GPIO基础路径

int config_index; // 全局配置索引，用于指定启用的通道位掩码（命令行参数传入）

/**
 * @brief 获取GPIO基础路径
 * @return 返回系统中存在的GPIO路径，优先返回rb_gpio，如果不存在则返回tz_gpio
 */
const char* get_gpio_base_path()
{
    static const char* gpio_path = nullptr;
    
    // 如果已经检测过，直接返回结果
    if (gpio_path != nullptr) {
        return gpio_path;
    }
    
    // 检查路径是否存在
    struct stat path_stat;
    
    // 优先检查rb_gpio路径
    if (::stat("/sys/class/rb_gpio/", &path_stat) == 0 && S_ISDIR(path_stat.st_mode)) {
        gpio_path = "/sys/class/rb_gpio/";
        return gpio_path;
    }
    
    // 检查tz_gpio路径
    if (::stat("/sys/class/tz_gpio/", &path_stat) == 0 && S_ISDIR(path_stat.st_mode)) {
        gpio_path = "/sys/class/tz_gpio/";
        return gpio_path;
    }
    
    // 如果两个路径都不存在，默认使用rb_gpio
    gpio_path = "/sys/class/rb_gpio/";
    return gpio_path;
}

/**
 * @brief 主函数：相机系统启动入口
 * @param argc 命令行参数数量
 * @param argv 命令行参数字符串数组
 * @return 程序退出状态（0表示正常运行）
 */
int main(int argc, char *argv[])
{
    reset_power(); // 初始化时重置相机电源，确保硬件就绪
    // 获取线程的底层句柄
    pthread_t nativeHandle = pthread_self();
    // 创建线程属性对象
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    // 设置线程调度策略为RR类型
    pthread_attr_setschedpolicy(&attr, SCHED_RR);
    // 创建线程参数对象
    struct sched_param param;
    param.sched_priority = 90;
    // 设置线程调度参数
    pthread_attr_setschedparam(&attr, &param);
    // 设置线程属性
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    // 修改线程的调度属性
    pthread_setschedparam(nativeHandle, SCHED_RR, &param);
    // 销毁线程属性对象
    pthread_attr_destroy(&attr);

    int ret = 0;
    if (argc > 1)
    {
        // 检查是否为帮助命令（未实现具体帮助信息，直接退出）
        if (strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0)
        {
            return 0;
        }
        // 解析命令行参数：获取通道位掩码（二进制位表示启用的通道）
        if (argc >= 2)
        {
            config_index = atoi(argv[1]); // 将参数转换为整数（位掩码）
        }
    }

#if (defined(SUPPORT_ROS1_JPG) || defined(SUPPORT_ROS1_YUV))
    // ROS1初始化
    ros::init(argc, argv, "image_publisher_node");
    ros::NodeHandle nh;
#endif

#if (defined(SUPPORT_ROS2_JPG) || defined(SUPPORT_ROS2_YUV))
    // ROS2支持分支：初始化ROS2运行时环境（用于图像数据发布）
    rclcpp::init(argc, argv);
#endif

    std::vector<int> channum; // 存储启用的相机通道号列表

    // 处理通道配置：无参数时使用默认通道（0-3号）
    if (config_index == 0)
    {
        for (int i = 0; i < 4; i++)
        {
            channum.push_back(i); // 默认启用前4个通道
        }
        debug_info("传参为空，使用默认通道号\n");
    }
    else
    {
        // 根据位掩码解析启用的通道号（每一位对应一个通道是否启用）
        int bit_position = 0;
        while (config_index > 0)
        {
            if (config_index & 1)
            {
                channum.push_back(bit_position); // 记录当前位对应的通道号（从0开始）
            }
            config_index >>= 1; // 右移一位，检查下一个通道
            bit_position++;
        }
    }
    
    // 初始化相机接口（参数：保留字段，配置文件路径）
    ret = hb_vin_init(0, "./cfg/hb_j5dev.json"); 
    if (ret < 0)
    {
        printf(" Failed to hb_cam_init.\n");
        return -1; // 初始化失败，程序退出
    }
#ifdef SUPPORT_INIT  
    return 0;
#endif
    int cam_num = channum.size(); // 获取启用的相机数量
    CCameraMgr *cam[cam_num]; // 相机管理器数组，每个元素对应一个通道

#ifdef SUPPORT_D415_ALIGN
    // 创建相机对齐管理器
    CMgrCameraAlign *CameraAlign;
    CameraAlign = new CMgrCameraAlign();
    if (CameraAlign == nullptr)
    {
        debug_err("creatHandle failed !!!\n");
        return false;
    }
#endif

    // 初始化每个相机管理器
    for (int i = 0; i < cam_num; i++)
    {
        int pipeid = channum[i]; // 当前通道ID
        uint32_t video, width, height, fps, format;
        
        // 通过硬件抽象层获取相机参数（通道号映射到具体设备参数）
        hb_port_mapping(pipeid, &video, &width, &height, &fps, &format);
        
        // 日志输出相机参数（通道号、分辨率、帧率、格式等）
        debug_crit("i :%d,pipeid:%d video:%d width:%d height:%d fps:%d,format:%s\n",
                   i, pipeid, video, width, height, fps, DateTypeMap[format]);

#ifdef SUPPORT_D415_ALIGN
        if (pipeid == 0) {
            CameraAlign->setDepthResolution(width, height);
        } else if (pipeid == 1) {
            CameraAlign->setRgbResolution(width, height);
        }
#endif
        // 创建相机管理器实例并初始化
        cam[i] = new CCameraMgr(pipeid, video, width, height, fps, format);
        cam[i]->Init(); // 初始化相机（创建句柄、设置回调等）
        
#ifdef SUPPORT_SHOWIMG 
        cam[i]->SetNum(cam_num); // 设置相机数量（用于多窗口布局显示）
#endif
    }
#ifdef SUPPORT_D415_ALIGN
        CameraAlign->getparameter();
#endif
    // 启动所有相机的数据采集线程
    for (int i = 0; i < cam_num; i++)
    {
#ifdef SUPPORT_D415_ALIGN
        cam[i]->m_pCameraAlign = CameraAlign;
#endif
        cam[i]->Start(); // 调用相机驱动开始采集图像
    }

#ifdef SUPPORT_SHOWIMG
    while (1)
    {
        for (int i = 0; i < cam_num; i++)
        {
            if (cam[i]->m_hasNewImage) { // 检查标志位）
                cam[i]->doShowImage();
            }
            else
            {
                continue;
            }
        }
        usleep(1000);
    }
#endif

#if (defined(SUPPORT_ROS1_JPG) || defined(SUPPORT_ROS1_YUV))
    signal(SIGINT, signal_handler);  // 捕获 Ctrl+C 信号
    // 主循环：保持程序运行（防止退出）
    while (is_running)  // 检查标志以确定是否退出
    {
        // 定期检查每个相机是否超时（每秒检查一次）
        for (int i = 0; i < cam_num; i++) {
            cam[i]->CheckTimeout(1000);
        }
        sleep(1); // 每秒检查一次
        ros::spinOnce();  // ROS 1 事件处理
    }

    ros::shutdown();  // ROS 1 清理资源
    return 0;
#else
    // 主循环：保持程序运行（防止退出）
    while (1)
    {
        sleep(10); // 休眠10秒（降低CPU占用）
    }
#endif
}

/**
 * @brief 执行系统命令（通过子进程）
 * @param pCmd 要执行的命令字符串（如shell命令）
 * @return 命令执行状态：0表示成功，非零表示失败
 */
int sensor_system(const char *pCmd)
{
    pid_t pid; // 进程ID
    struct sigaction sa, intr, quit; // 信号处理结构体
    int status = 0; // 命令执行状态

    if (pCmd == NULL)
    {
        return 1; // 无效命令，返回错误状态
    }

    // 临时忽略SIGINT和SIGQUIT信号（防止子进程被终端中断）
    sa.sa_handler = SIG_IGN; // 信号处理函数：忽略信号
    sa.sa_flags = 0;
    sigemptyset(&sa.sa_mask); // 清空信号掩码

    // 设置信号处理：忽略中断和退出信号
    if ((sigaction(SIGINT, &sa, &intr) < 0) || (sigaction(SIGQUIT, &sa, &quit) < 0))
    {
        return -1; // 设置失败，返回错误
    }

    // 创建子进程（vfork保证子进程与父进程共享地址空间）
    pid = vfork();
    if (pid == 0)
    {
        // 子进程执行路径
        char achCmd[2048] = {0};
        strncpy(achCmd, pCmd, sizeof(achCmd) - 1); // 复制命令字符串到缓冲区

        // 构造shell命令参数（/bin/sh -c "命令"）
        char *achArgv[4] = {(char *)"/bin/sh", (char *)"-c", achCmd, NULL};
        
        // 关闭所有非必要文件描述符（3及以上，防止资源泄漏）
        for (int nfd = 3; nfd < 4096; nfd++)
        {
            if (close(nfd) != 0)
            {
                break; // 遇到错误停止（正常应全部关闭）
            }
        }
        
        // 恢复原始信号处理（子进程需要响应信号）
        (void)sigaction(SIGINT, &intr, (struct sigaction *)NULL);
        (void)sigaction(SIGQUIT, &quit, (struct sigaction *)NULL);

        // 执行命令（替换子进程为shell进程）
        int childRet = execv("/bin/sh", achArgv);
        if (childRet < 0)
        {
            // 执行失败时输出错误信息并退出子进程
            printf("%s failed %d errno =%d  %s!\n", pCmd, childRet, errno, strerror(errno));
            _exit(childRet); // 子进程退出，状态码传递给父进程
        }
    }
    else if (pid < 0)
    {
        return -1; // vfork失败，返回错误
    }
    else
    {
        // 父进程等待子进程完成
        int n;
        do
        {
            // 阻塞等待子进程退出，获取状态码
            n = waitpid(pid, &status, 0);
        } while (n == -1 && errno == EINTR); // 处理被中断的系统调用

        if (n != pid)
        {
            status = -1; // 等待失败，设置错误状态
        }
    }

    // 恢复父进程的原始信号处理
    if (sigaction(SIGINT, &intr, (struct sigaction *)NULL) ||
        sigaction(SIGQUIT, &quit, (struct sigaction *)NULL) != 0)
    {
        return -1; // 恢复失败，返回错误
    }
    return status; // 返回命令执行结果（0表示成功）
}

/**
 * @brief 重置相机电源（根据不同板卡型号执行差异化操作）
 * @return 0表示成功，非零表示失败
 */
int reset_power()
{
    int board_id = camera_sys_get_board_id(); // 获取当前板卡型号
    const char* gpio_path = get_gpio_base_path(); // 获取GPIO基础路径
    char cmd[256];
    debug_info("start reset power\n");
    debug_info("power off\n");

    // 根据板卡型号执行不同的电源管理逻辑
    if (board_id == GEAC91 ||
        board_id == GEAC90 ||
        board_id == GEACJX ||
        board_id == GEAC91VP )
    {
        // 解串器下电（通过GPIO控制，适用于特定板卡）
        snprintf(cmd, sizeof(cmd), "echo 0 > %scamera_0_1-power/value", gpio_path);
        sensor_system(cmd); // 主机0电源控制
        snprintf(cmd, sizeof(cmd), "echo 0 > %scamera_2_3-power/value", gpio_path);
        sensor_system(cmd); // 主机1电源控制
        snprintf(cmd, sizeof(cmd), "echo 0 > %scamera_4_5-power/value", gpio_path);
        sensor_system(cmd); // 主机2电源控制
        snprintf(cmd, sizeof(cmd), "echo 0 > %scamera_6_7-power/value", gpio_path);
        sensor_system(cmd); // 主机3电源控制
        usleep(3000 * 1000); // 等待3秒确保下电完成

        // 解串器上电（重新激活相机硬件）
        snprintf(cmd, sizeof(cmd), "echo 1 > %scamera_0_1-power/value", gpio_path);
        sensor_system(cmd);
        snprintf(cmd, sizeof(cmd), "echo 1 > %scamera_2_3-power/value", gpio_path);
        sensor_system(cmd);
        snprintf(cmd, sizeof(cmd), "echo 1 > %scamera_4_5-power/value", gpio_path);
        sensor_system(cmd);
        snprintf(cmd, sizeof(cmd), "echo 1 > %scamera_6_7-power/value", gpio_path);
        sensor_system(cmd);
    }
    else if (board_id == T24DG26TYA_ORIN ||
             board_id == T24DG26TYB_ORIN )
    {
        // 解串器下电（主机0和1）
        snprintf(cmd, sizeof(cmd), "echo 0 > %scamera_0_1-power/value", gpio_path);
        sensor_system(cmd);
        snprintf(cmd, sizeof(cmd), "echo 0 > %scamera_2_3-power/value", gpio_path);
        sensor_system(cmd);
        usleep(3000 * 1000);

        // 通过I2C控制相机电源下电（特定芯片地址，如0x28和0x29）
        sensor_system("i2ctransfer -f -y 7 w2@0x28 0x01 0x10");
        sensor_system("i2ctransfer -f -y 7 w2@0x29 0x01 0x10");
        usleep(100 * 1000); // 等待100ms

        // I2C控制相机上电（恢复电源）
        sensor_system("i2ctransfer -f -y 7 w2@0x28 0x01 0x1f");
        sensor_system("i2ctransfer -f -y 7 w2@0x29 0x01 0x1f");
        usleep(100 * 1000);

        // 解串器上电
        snprintf(cmd, sizeof(cmd), "echo 1 > %scamera_0_1-power/value", gpio_path);
        sensor_system(cmd);
        snprintf(cmd, sizeof(cmd), "echo 1 > %scamera_2_3-power/value", gpio_path);
        sensor_system(cmd);
    }
    // 其他板卡型号的电源管理逻辑（逻辑类似，省略详细注释）
    else if (board_id == C24HH01 || board_id == T24DG26TYA_NX ||
             board_id == T24DG26TYB_NX || board_id == C24HH01ARFAYF ||
            board_id == T25GJ65CF_FLY || board_id == T25LK26)
    {
        // 解串器下电 + I2C控制相机电源（主机0和1）
        snprintf(cmd, sizeof(cmd), "echo 0 > %scamera_0_1-power/value", gpio_path);
        sensor_system(cmd);
        snprintf(cmd, sizeof(cmd), "echo 0 > %scamera_2_3-power/value", gpio_path);
        sensor_system(cmd);
        sensor_system("i2ctransfer -f -y 1 w2@0x28 0x01 0x10");
        sensor_system("i2ctransfer -f -y 1 w2@0x29 0x01 0x10");
        usleep(100 * 1000);
        sensor_system("i2ctransfer -f -y 1 w2@0x28 0x01 0x1f");
        sensor_system("i2ctransfer -f -y 1 w2@0x29 0x01 0x1f");
        usleep(100 * 1000);
        // 解串器上电
        snprintf(cmd, sizeof(cmd), "echo 1 > %scamera_0_1-power/value", gpio_path);
        sensor_system(cmd);
        snprintf(cmd, sizeof(cmd), "echo 1 > %scamera_2_3-power/value", gpio_path);
        sensor_system(cmd);
    }
    else if (board_id == T25BH79)
    {
        // 解串器下电（主机0和1，控制不同通道范围）
        snprintf(cmd, sizeof(cmd), "echo 0 > %scamera_0_3-power/value", gpio_path);
        sensor_system(cmd);
        snprintf(cmd, sizeof(cmd), "echo 0 > %scamera_4_7-power/value", gpio_path);
        sensor_system(cmd);
        sensor_system("i2ctransfer -f -y 11 w2@0x28 0x01 0x10");
        sensor_system("i2ctransfer -f -y 11 w2@0x29 0x01 0x10");
        usleep(100 * 1000);
        sensor_system("i2ctransfer -f -y 11 w2@0x29 0x01 0x1f");
        sensor_system("i2ctransfer -f -y 11 w2@0x28 0x01 0x1f");
        usleep(100 * 1000);
        snprintf(cmd, sizeof(cmd), "echo 1 > %scamera_0_3-power/value", gpio_path);
        sensor_system(cmd);
        snprintf(cmd, sizeof(cmd), "echo 1 > %scamera_4_7-power/value", gpio_path);
        sensor_system(cmd); 
    }
    else if (board_id == B25EJ21_007)
    {
        // 解串器下电（主机0和1，控制不同通道范围）
        snprintf(cmd, sizeof(cmd), "echo 0 > %sdser_1-power/value", gpio_path);
        sensor_system(cmd);
        snprintf(cmd, sizeof(cmd), "echo 0 > %sdser_2-power/value", gpio_path);
        sensor_system(cmd);
        snprintf(cmd, sizeof(cmd), "echo 0 > %sdser_3-power/value", gpio_path);
        sensor_system(cmd);
        sensor_system("i2ctransfer -f -y 2 w2@0x77 0x01 0x00");
        sensor_system("i2ctransfer -f -y 2 w2@0x70 0x01 0x00");
        sensor_system("i2ctransfer -f -y 2 w2@0x72 0x01 0x00");
        usleep(100 * 1000);
        sensor_system("i2ctransfer -f -y 2 w2@0x77 0x01 0x0f");
        sensor_system("i2ctransfer -f -y 2 w2@0x70 0x01 0x0f");
        sensor_system("i2ctransfer -f -y 2 w2@0x72 0x01 0x0f");
        usleep(100 * 1000);
        snprintf(cmd, sizeof(cmd), "echo 1 > %sdser_1-power/value", gpio_path);
        sensor_system(cmd);
        snprintf(cmd, sizeof(cmd), "echo 1 > %sdser_2-power/value", gpio_path);
        sensor_system(cmd);
        snprintf(cmd, sizeof(cmd), "echo 1 > %sdser_3-power/value", gpio_path);
        sensor_system(cmd);
    }
    else if (board_id == B22LD37X0XSJ)
    {
        sensor_system("i2ctransfer -f -y 11 w2@0x77 0x01 0x10");
        usleep(100 * 1000);
        snprintf(cmd, sizeof(cmd), "echo 0 > %scamera_0_1-power/value", gpio_path);
        sensor_system(cmd);
        usleep(100 * 1000);
        snprintf(cmd, sizeof(cmd), "echo 1 > %scamera_0_1-power/value", gpio_path);
        sensor_system(cmd);
        usleep(100 * 1000);
        sensor_system("i2ctransfer -f -y 11 w2@0x77 0x01 0x1f");
    }
    else if (board_id == B25EJ21_T1)
    {
        snprintf(cmd, sizeof(cmd), "echo 0 > %sdser_1-power/value", gpio_path);
        sensor_system(cmd);
        snprintf(cmd, sizeof(cmd), "echo 0 > %sdser_2-power/value", gpio_path);
        sensor_system(cmd);
        snprintf(cmd, sizeof(cmd), "echo 0 > %sdser_3-power/value", gpio_path);
        sensor_system(cmd);
        snprintf(cmd, sizeof(cmd), "echo 0 > %sdser_4-power/value", gpio_path);
        sensor_system(cmd);
        sensor_system("i2ctransfer -f -y 2 w2@0x77 0x01 0x00");
        sensor_system("i2ctransfer -f -y 2 w2@0x70 0x01 0x00");
        sensor_system("i2ctransfer -f -y 2 w2@0x72 0x01 0x00");
        sensor_system("i2ctransfer -f -y 2 w2@0x76 0x01 0x00");
        usleep(100 * 1000);
        sensor_system("i2ctransfer -f -y 2 w2@0x77 0x01 0x0f");
        sensor_system("i2ctransfer -f -y 2 w2@0x70 0x01 0x0f");
        sensor_system("i2ctransfer -f -y 2 w2@0x72 0x01 0x0f");
        sensor_system("i2ctransfer -f -y 2 w2@0x76 0x01 0x0f");
        usleep(100 * 1000);
        snprintf(cmd, sizeof(cmd), "echo 1 > %sdser_1-power/value", gpio_path);
        sensor_system(cmd);
        snprintf(cmd, sizeof(cmd), "echo 1 > %sdser_2-power/value", gpio_path);
        sensor_system(cmd);
        snprintf(cmd, sizeof(cmd), "echo 1 > %sdser_3-power/value", gpio_path);
        sensor_system(cmd);
        snprintf(cmd, sizeof(cmd), "echo 1 > %sdser_4-power/value", gpio_path);
        sensor_system(cmd);
    }
    else if (board_id == T25KK15)
    {
        snprintf(cmd, sizeof(cmd), "echo 0 > %scamera_0_1-power/value", gpio_path);
        sensor_system(cmd);
        sensor_system("i2ctransfer -f -y 1 w2@0x77 0x01 0x00");
        usleep(100 * 1000);
        sensor_system("i2ctransfer -f -y 1 w2@0x77 0x01 0x0f");
        usleep(100 * 1000);
        snprintf(cmd, sizeof(cmd), "echo 1 > %scamera_0_1-power/value", gpio_path);
        sensor_system(cmd);
    }
    else if (board_id == T25KK15_R2)
    {
        snprintf(cmd, sizeof(cmd), "echo 0 > %scamera_0_1-power/value", gpio_path);
        sensor_system(cmd);
        snprintf(cmd, sizeof(cmd), "echo 0 > %scamera_2_3-power/value", gpio_path);
        sensor_system(cmd);
        sensor_system("i2ctransfer -f -y 1 w2@0x76 0x01 0x00");
        sensor_system("i2ctransfer -f -y 1 w2@0x77 0x01 0x00");
        usleep(100 * 1000);
        sensor_system("i2ctransfer -f -y 1 w2@0x76 0x01 0x0f");
        sensor_system("i2ctransfer -f -y 1 w2@0x77 0x01 0x0f");
        usleep(100 * 1000);
        snprintf(cmd, sizeof(cmd), "echo 1 > %scamera_0_1-power/value", gpio_path);
        sensor_system(cmd);
        snprintf(cmd, sizeof(cmd), "echo 1 > %scamera_2_3-power/value", gpio_path);
        sensor_system(cmd);
    }
    else
    {
        // 通用处理分支（适用于未识别的板卡，执行默认电源管理）
        snprintf(cmd, sizeof(cmd), "echo 0 > %scamera_0_1-power/value", gpio_path);
        sensor_system(cmd);
        snprintf(cmd, sizeof(cmd), "echo 0 > %scamera_2_3-power/value", gpio_path);
        sensor_system(cmd);
        snprintf(cmd, sizeof(cmd), "echo 0 > %scamera_4_5-power/value", gpio_path);
        sensor_system(cmd);
        snprintf(cmd, sizeof(cmd), "echo 0 > %scamera_6_7-power/value", gpio_path);
        sensor_system(cmd);
        usleep(3000 * 1000);
        sensor_system("i2ctransfer -f -y 7 w2@0x28 0x01 0x10");
        sensor_system("i2ctransfer -f -y 7 w2@0x29 0x01 0x10");
        sensor_system("i2ctransfer -f -y 8 w2@0x28 0x01 0x10");
        sensor_system("i2ctransfer -f -y 8 w2@0x29 0x01 0x10");
        usleep(100 * 1000);
        sensor_system("i2ctransfer -f -y 7 w2@0x28 0x01 0x1f");
        sensor_system("i2ctransfer -f -y 7 w2@0x29 0x01 0x1f");
        sensor_system("i2ctransfer -f -y 8 w2@0x28 0x01 0x1f");
        sensor_system("i2ctransfer -f -y 8 w2@0x29 0x01 0x1f");
        usleep(100 * 1000);
        snprintf(cmd, sizeof(cmd), "echo 1 > %scamera_0_1-power/value", gpio_path);
        sensor_system(cmd);
        snprintf(cmd, sizeof(cmd), "echo 1 > %scamera_2_3-power/value", gpio_path);
        sensor_system(cmd);
        snprintf(cmd, sizeof(cmd), "echo 1 > %scamera_4_5-power/value", gpio_path);
        sensor_system(cmd);
        snprintf(cmd, sizeof(cmd), "echo 1 > %scamera_6_7-power/value", gpio_path);
        sensor_system(cmd);
    }
    debug_info("power on\n");
    return 0;
}

/**
 * @brief 从JSON配置文件中解析通道配置信息
 * @param channum 输出参数：存储解析出的通道号列表
 * @param portmask 输出参数：存储端口掩码（若配置文件中使用掩码模式）
 * @return 解析后的通道号列表
 */
static std::vector<int> get_hb_j5_json(std::vector<int> &channum, int &portmask)
{
    int chan = 0;
    // 打开JSON配置文件（路径固定为./cfg/hb_j5dev.json）
    FILE *file = fopen("./cfg/hb_j5dev.json", "r");
    if (file == NULL)
    {
        printf("Failed to open JSON file.\n");
        return channum; // 打开失败，返回空列表
    }

    // 获取文件长度以分配内存
    fseek(file, 0, SEEK_END);
    long file_size = ftell(file);
    rewind(file); // 重置文件指针到开头

    // 分配内存缓冲区存储JSON数据
    char *json_data = (char *)malloc(file_size + 1);
    if (json_data == NULL)
    {
        printf("Failed to allocate memory for JSON data.\n");
        fclose(file);
        return channum; // 内存分配失败
    }

    // 读取文件内容到内存缓冲区
    if (fread(json_data, 1, file_size, file) != file_size)
    {
        printf("Failed to read JSON data from file.\n");
        free(json_data);
        fclose(file);
        return channum; // 读取失败
    }

    fclose(file); // 关闭文件

    // 解析JSON数据（使用cJSON库）
    cJSON *root = cJSON_Parse(json_data);
    if (root == NULL)
    {
        printf("Failed to parse JSON data.\n");
        free(json_data); // 释放内存
        return channum; // 解析失败
    }

    // 获取配置节点（假设主配置节点为config_0）
    cJSON *config = cJSON_GetObjectItem(root, "config_0");
    if (config == NULL)
    {
        printf("Failed to get 'config_0' object.\n");
        cJSON_Delete(root); // 释放JSON解析树
        free(json_data);
        return channum; // 配置节点不存在
    }

    // 提取端口号或端口掩码字段
    cJSON *portNumber = cJSON_GetObjectItem(config, "port_number");
    cJSON *portMask = cJSON_GetObjectItem(config, "port_mask");
    
    if (portNumber == NULL && portMask == NULL)
    {
        // 缺少必要字段，解析失败
        printf("Failed to get 'port_number' or 'port_mask'.\n");
        cJSON_Delete(root);
        free(json_data);
        return channum;
    }
    else if (portMask == NULL)
    {
        // 使用端口号模式：启用连续的通道号（0到portNumber-1）
        chan = portNumber->valueint;
        for (int i = 0; i < chan; i++)
        {
            channum.push_back(i); // 添加连续通道号
        }
    }
    else
    {
        // 使用端口掩码模式：按位解析启用的通道
        chan = portMask->valueint;
        portmask = chan; // 保存掩码供外部使用
        int bit_position = 0;
        while (chan > 0)
        {
            if (chan & 1)
            {
                channum.push_back(bit_position); // 记录当前位对应的通道号
            }
            chan >>= 1; // 右移一位继续解析
            bit_position++;
        }
    }

    // 释放资源
    cJSON_Delete(root); // 销毁JSON解析树
    free(json_data); // 释放内存缓冲区
    return channum;
}
