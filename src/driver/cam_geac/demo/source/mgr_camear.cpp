#include "../inc/mgr_camera.h"
#define SKIP_NUM 4  // 跳帧显示参数，每SKIP_NUM帧显示一帧

/**
 * @brief 相机管理器构造函数
 * @param dwPipeId 管道ID（对应相机设备）
 * @param dwVideoIndex 视频设备索引
 * @param dwWidth 图像宽度
 * @param dwHeight 图像高度
 * @param dwFps 帧率
 * @param format 图像格式（如RAW、YUYV等）
 */
CCameraMgr::CCameraMgr(int dwPipeId, int dwVideoIndex, int dwWidth, int dwHeight, int dwFps, int format)
{
    m_nChan = dwPipeId;                // 通道ID
    m_dwVideoIndex = dwVideoIndex;     // 视频设备索引
    m_dwWidth = dwWidth;               // 图像宽度
    m_dwHeight = dwHeight;             // 图像高度
    m_nFps = dwFps;                    // 帧率
    m_dwFormat = format;               // 图像格式
    m_pCameraBase = NULL;              // 基础相机操作句柄
    m_llTimestamp = 0;                 // 时间戳记录
    m_llFrame = 0;                     // 帧计数器

#ifdef SUPPORT_JPEGENC
    m_pCamerajpeg = NULL;              // JPEG编码句柄
#endif

#ifdef SUPPORT_RTSPS
    m_pVideoPush = NULL;               // RTSP推流句柄
    memset(&m_stuImage, 0, sizeof(m_stuImage));  // 图像数据结构初始化
#endif

#ifdef SUPPORT_D415_ALIGN
    m_pCameraAlign = NULL;
    output_aligned.assign(3840 * 2160 * 2, 0);  // 对齐输出缓冲区初始化
#endif

}

/**
 * @brief 相机管理器析构函数
 */
CCameraMgr::~CCameraMgr()
{
}

/**
 * @brief 初始化相机管理器
 * @return 初始化成功返回true
 */
bool CCameraMgr::Init()
{
    creatHandle();     // 创建各类句柄
    initHandle();      // 初始化句柄
    return true;
}

/**
 * @brief 开始相机数据采集
 */
void CCameraMgr::Start()
{
    m_pCameraBase->StartAcquire();  // 调用基础相机接口开始采集
}

/**
 * @brief 检查相机是否超时，并更新发布状态
 * @param timeoutMs 超时时间（毫秒），默认1000ms
 * @return 总是返回true
 */
bool CCameraMgr::CheckTimeout(int timeoutMs)
{
#ifdef SUPPORT_JPEGENC
    if (m_pCamerajpeg) {
        m_pCamerajpeg->CheckAndPublishTimeout(timeoutMs);
    }
#endif
    return true;
}

/**
 * @brief 创建各类功能句柄
 * @return 创建成功返回true
 */
bool CCameraMgr::creatHandle()
{
    /* 创建取图句柄 最基础的功能：取图 */
    m_pCameraBase = new CV4l2Cam(m_nChan, m_dwVideoIndex, m_dwWidth, m_dwHeight, m_nFps, m_dwFormat);
    if (m_pCameraBase == nullptr)
    {
        debug_err("creatHandle failed, /dev/video%d\n", m_nChan);
        return false;
    }

#ifdef SUPPORT_JPEGENC
    /* 创建jpeg句柄 */
    m_pCamerajpeg = new CMgrCameraJpegEnc(m_nChan, m_dwWidth, m_dwHeight, 75);  // 压缩质量75%
    if (m_pCamerajpeg == nullptr)
    {
        debug_err("creatHandle failed, /dev/video%d\n", m_nChan);
        return false;
    }
#endif

#ifdef SUPPORT_RTSPS
    /*  创建推流句柄： 封包，编码，推流*/
    m_pVideoPush = new CVideoPush(m_nChan, m_dwWidth, m_dwHeight, m_nFps);
    m_pVideoPush->Init();  // 初始化推流模块
#endif

#ifdef SUPPORT_ROS1_YUV
    // 创建ROS1话题名称（与ROS2相同）
    char m_szTopic[128];
    snprintf(m_szTopic, sizeof(m_szTopic), "cam%d/compressed", m_nChan);
    m_pRosPublisher = std::make_shared<CCameraPublisher>(m_szTopic);
#endif

#ifdef SUPPORT_ROS2_YUV
    // 创建ROS1话题名称（与ROS2相同）
    char m_szTopic[128];
    snprintf(m_szTopic, sizeof(m_szTopic), "cam%d/compressed", m_nChan);
    m_pRosPublisher = std::make_shared<CCameraPublisher>(m_szTopic);
#endif


    return true;
}

/**
 * @brief 初始化各类句柄
 * @return 初始化成功返回true
 */
bool CCameraMgr::initHandle()
{
    m_pCameraBase->Init();  // 初始化基础相机句柄
    setCamPublish();        // 设置相机数据回调

#ifdef SUPPORT_JPEGENC
    m_pCamerajpeg->Init();  // 初始化JPEG编码器
#endif

#ifdef SUPPORT_SHOWIMG
    //获取显示窗口分辨率
    Display* display = XOpenDisplay(nullptr);
    Screen* screen = DefaultScreenOfDisplay(display);
    m_dwWinWidth = WidthOfScreen(screen);   // 屏幕宽度
    m_dwWinHeight = HeightOfScreen(screen); // 屏幕高度
    XCloseDisplay(display);
    
    // 初始化 XCB 线程支持,程序是多线程的，所以调用 XInitThreads 函数来初始化 XCB 线程支持
    XInitThreads();
#endif

    return true;
}

/**
 * @brief 设置相机数据回调函数
 */
void CCameraMgr::setCamPublish()
{
    m_pCameraBase->SetDataCallback(CCameraMgr::callbackImage, this);  // 设置图像回调函数
}

#ifdef SUPPORT_SHOWIMG
/**
 * @brief 设置相机数量（用于多窗口布局）
 * @param dwNUm 相机总数
 * @return 设置成功返回true
 */
bool CCameraMgr::SetNum(int dwNUm)
{
    m_dwCamNum = dwNUm;  // 记录相机总数
    return true;
}

/**
 * @brief 显示图像（支持多窗口排列）
 * @param nChan 通道号
 * @param llTimestamp 时间戳
 * @param dwDiff 帧间隔时间
 * @param nWidth 图像宽度
 * @param nHeight 图像高度
 * @param pData 图像数据
 * @param nDatalen 数据长度
 * @return 显示成功返回true
 */
bool CCameraMgr::doShowImage()
{
    struct timeval tv1, tv2, tv3, tv4, tv5;
    gettimeofday(&tv1, NULL);  // 记录开始时间

    int NUM_OF_BOX_GRID = 3;   // 默认为3×3网格显示
    if(m_dwCamNum > 9)
    {
        NUM_OF_BOX_GRID = 4;   // 超过9个相机时使用4×4网格
    }

    std::lock_guard<std::mutex> lock(m_mutex);  // 加锁，避免数据竞争
    
    std::string windowname = std::string("chan:") + std::to_string(m_ImageData.nChan);  // 窗口名称
    cv::namedWindow(windowname.c_str(), cv::WINDOW_NORMAL);  // 创建可调整大小的窗口
    
    // 窗口大小设置为 m_dwWinWidth / NUM_OF_BOX_GRID -10 * m_dwWinHeight / NUM_OF_BOX_GRID -10 ,增加了窗口间的间隔
    cv::resizeWindow(windowname.c_str(), m_dwWinWidth / NUM_OF_BOX_GRID - 20, m_dwWinHeight / NUM_OF_BOX_GRID - 10);

    // 移动窗口到指定位置（dwWindowX, dwWindowY）
    int dwRow = m_ImageData.nChan % NUM_OF_BOX_GRID;    // 窗口属于第几行
    int dwColumn = m_ImageData.nChan / NUM_OF_BOX_GRID; // 窗口属于第几列
    int dwWindowX = dwRow * m_dwWinWidth / NUM_OF_BOX_GRID;
    int dwWindowY = dwColumn * m_dwWinHeight / NUM_OF_BOX_GRID;
    
    // 偏移是为了不重叠
    cv::moveWindow(windowname.c_str(), dwWindowX + m_dwWinWidth / 40, dwWindowY + m_dwWinHeight / 40);
    
    // cv::Mat imgRGB;  // RGB格式图像
    gettimeofday(&tv2, NULL);

    // 将 pData YUV422数据转化为CV能处理的格式
    // cv::Mat imgYUV(nHeight, nWidth, CV_8UC2, pData);
    gettimeofday(&tv3, NULL);
    
    // 将 YUV422 数据转换为 RGB 格式
    // cv::cvtColor(imgYUV, imgRGB, cv::COLOR_YUV2BGR_YUYV);
    gettimeofday(&tv4, NULL);

    // 设置文本格式与位置，并绘制时间戳文本
    std::string text = "tm:" + std::to_string(m_ImageData.llTimestamp) + ",diff:" + std::to_string(m_ImageData.diff_ms)+"ms";
    cv::Scalar textColor(255, 255, 255); // 文本颜色 RGB
    int fontSize = 4;
    int fontThickness = 3;
    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    cv::Size textSize = cv::getTextSize(text, fontFace, fontSize, fontThickness, nullptr);
    cv::Point textPosition((m_ImageData.imgRGB.cols - textSize.width) / 2, textSize.height);
    cv::putText(m_ImageData.imgRGB, text, textPosition, fontFace, fontSize, textColor, fontThickness);
    cv::imshow(windowname.c_str(), m_ImageData.imgRGB);  // 显示 RGB 图像
    cv::waitKey(1);  // 1ms 等待更新UI，如果没有此等待无法显示图片
    gettimeofday(&tv5, NULL);

    // 性能统计（当前注释掉）
    long diffms1 = (tv2.tv_sec * 1000 + tv2.tv_usec / 1000) - (tv1.tv_sec * 1000 + tv1.tv_usec / 1000);
    long diffms2 = (tv3.tv_sec * 1000 + tv3.tv_usec / 1000) - (tv2.tv_sec * 1000 + tv2.tv_usec / 1000);
    long diffms3 = (tv4.tv_sec * 1000 + tv4.tv_usec / 1000) - (tv3.tv_sec * 1000 + tv3.tv_usec / 1000);
    long diffms4 = (tv5.tv_sec * 1000 + tv5.tv_usec / 1000) - (tv4.tv_sec * 1000 + tv4.tv_usec / 1000);
    long diffms = (tv5.tv_sec * 1000 + tv5.tv_usec / 1000) - (tv1.tv_sec * 1000 + tv1.tv_usec / 1000);
    // debug_dbg("diff time1:%ldms,diff time2:%ldms,diff time3:%ldms,diff time4:%ldms，alltime:%ldms\n", diffms1, diffms2, diffms3, diffms4, diffms);
    m_hasNewImage = false; // 重置标志位

    return true;
}
#endif

/**
 * @brief 图像数据回调函数（静态成员函数）
 * @param nChan 通道号
 * @param stTime 时间戳
 * @param nWidth 图像宽度
 * @param nHeight 图像高度
 * @param pData 图像数据
 * @param nDatalen 数据长度
 * @param pUserData 用户数据指针（指向CCameraMgr实例）
 */
void CCameraMgr::callbackImage(int nChan, struct timespec stTime, int nWidth, int nHeight, unsigned char *pData, int nDatalen, void *pUserData)
{
    CCameraMgr *pCamMgr = (CCameraMgr *)pUserData;  // 获取相机管理器实例
    if (pCamMgr == NULL)
    {
        return;
    }
    
    /* 打印基础信息 */
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    struct timespec stv;
    stv.tv_sec = tv.tv_sec;
    stv.tv_nsec = tv.tv_usec * 1000;
    
    // rtcpu时间戳（毫秒）
    long llTimestamp = stTime.tv_sec * 1000 + stTime.tv_nsec / 1000000; 

    // sys上下两帧帧间隔（毫秒）
    long llsysDiff = stv.tv_sec * 1000 + stv.tv_nsec / 1000000 - pCamMgr->m_oldllTimestamp;
    pCamMgr->m_oldllTimestamp = stv.tv_sec * 1000 + stv.tv_nsec / 1000000;

    // rtcpu上下两帧帧间隔（毫秒）
    long dwDiff = llTimestamp - pCamMgr->m_llTimestamp;
    pCamMgr->m_llTimestamp = stTime.tv_sec * 1000 + stTime.tv_nsec / 1000000;
    
    // 当前系统时间-rtcpu时间（延迟）
    long diff_ms = stv.tv_sec * 1000 + stv.tv_nsec / 1000000 - llTimestamp;
    
    debug_info("[chan:%d]: frametime:%ld.%09ld,systime:%ld.%09ld,delay:%ldms interval:%ldms,llsysDiff:%ldms\n",
               nChan, stTime.tv_sec, stTime.tv_nsec, stv.tv_sec, stv.tv_nsec, diff_ms, dwDiff, llsysDiff);
    pCamMgr->m_llFrame++;  // 帧计数器递增

#ifdef SUPPORT_SHOWIMG
    // 抽 SKIP_NUM 帧显示
    if ((pCamMgr->m_llFrame % SKIP_NUM) == 0)
    {
        cv::Mat imgYUV(nHeight, nWidth, CV_8UC2, pData);  // 输入 YUY2 数据
        cv::Mat imgBGR_temp;
        cv::cvtColor(imgYUV, imgBGR_temp, cv::COLOR_YUV2BGR_YUYV);  // 转成 OpenCV 支持的 BGR 格式
        std::lock_guard<std::mutex> lock(pCamMgr->m_mutex);  // 加锁，避免数据竞争
        pCamMgr->m_ImageData.nChan = nChan;
        pCamMgr->m_ImageData.llTimestamp = llTimestamp;
        pCamMgr->m_ImageData.diff_ms = diff_ms;
        pCamMgr->m_ImageData.imgRGB = imgBGR_temp.clone();
        pCamMgr->m_hasNewImage = true;
    }
#endif

#ifdef SUPPORT_RTSPS
    // 准备推流数据
    pCamMgr->m_stuImage.imgHead.wWidth = nWidth;
    pCamMgr->m_stuImage.imgHead.wHeight = nHeight;
    pCamMgr->m_stuImage.imgHead.dwDataLength = 3 * nWidth * nHeight / 2;  // YUV420数据长度
    pCamMgr->m_stuImage.imgHead.dwTimestampSec = (unsigned int)stTime.tv_sec;
    pCamMgr->m_stuImage.imgHead.dwTimestampNsec = (unsigned int)stTime.tv_nsec;
    
    // YUY2转I420（YUV420格式）
    libyuv::YUY2ToI420((uint8_t *)pData, 2 * nWidth,
                       pCamMgr->m_stuImage.image, nWidth,
                       pCamMgr->m_stuImage.image + nWidth * nHeight, nWidth / 2,
                       pCamMgr->m_stuImage.image + 5 * nWidth * nHeight / 4, nWidth / 2, nWidth, nHeight);
    
    // 送给推流服务
    pCamMgr->m_pVideoPush->InputData(&pCamMgr->m_stuImage.imgHead);
#endif

#ifdef SUPPORT_JPEGENC
    /* 送入 jpeg 编码 */
    pCamMgr->m_pCamerajpeg->Save(nChan, stTime, nWidth, nHeight, pData, nDatalen);
#endif


#ifdef SUPPORT_ROS1_YUV
    pCamMgr->m_pRosPublisher->Publisher(stTime, pData, nDatalen);
    ros::spinOnce();
#endif
#ifdef SUPPORT_ROS2_YUV
    pCamMgr->m_pRosPublisher->Publisher(stTime, pData, nDatalen);
    rclcpp::spin_some(pCamMgr->m_pRosPublisher);
#endif

#ifdef SUPPORT_D415_ALIGN
         int out_nDatalen = 0;
        pCamMgr->m_pCameraAlign->inputFrame(nChan, pData, nDatalen, pCamMgr->output_aligned.data(), &out_nDatalen);
        // 以 out_nDatalen 为准使用对齐后的数据
        // 抽 SKIP_NUM 帧显示
        // if (nChan == 0 )
        // {
        //     if ((pCamMgr->m_llFrame % SKIP_NUM) == 0)
        //     {
        //          // 直接使用 CV_16UC1 构造 Mat（注意宽度可能需调整，因为 16 位每个像素占 2 字节）
        //         cv::Mat depth16U(nHeight, nWidth, CV_16UC1, pCamMgr->output_aligned.data());
 
        //         // 将 16 位深度值缩放到 8 位（例如线性映射到 0-255）
        //         double minVal, maxVal;
        //         cv::minMaxLoc(depth16U, &minVal, &maxVal);
        //         cv::Mat depth8U;
        //         depth16U.convertTo(depth8U, CV_8U, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));
        //         cv::Mat imgBGR_temp;
        //         // 应用颜色映射
        //         cv::applyColorMap(depth8U, imgBGR_temp, cv::COLORMAP_JET);
 
        //         std::lock_guard<std::mutex> lock(pCamMgr->m_mutex);  // 加锁，避免数据竞争
        //         pCamMgr->m_ImageData.nChan = nChan;
        //         pCamMgr->m_ImageData.llTimestamp = llTimestamp;
        //         pCamMgr->m_ImageData.diff_ms = diff_ms;
        //         pCamMgr->m_ImageData.imgRGB = imgBGR_temp.clone();
        //         pCamMgr->m_hasNewImage = true;
        //     }
        // } else
        // {
        //     if ((pCamMgr->m_llFrame % SKIP_NUM) == 0)
        //     {
        //     // 使用实际数据长度创建图像，避免使用未初始化的缓冲区数据
        //         cv::Mat imgYUV(nHeight, nWidth, CV_8UC2, pData);  // 输入 YUY2 数据
        //         cv::Mat imgBGR_temp;
        //         cv::cvtColor(imgYUV, imgBGR_temp, cv::COLOR_YUV2BGR_YUYV);  // 转成 OpenCV 支持的 BGR 格式
        //         std::lock_guard<std::mutex> lock(pCamMgr->m_mutex);  // 加锁，避免数据竞争
        //         pCamMgr->m_ImageData.nChan = nChan;
        //         pCamMgr->m_ImageData.llTimestamp = llTimestamp;
        //         pCamMgr->m_ImageData.diff_ms = diff_ms;
        //         pCamMgr->m_ImageData.imgRGB = imgBGR_temp.clone();
        //         pCamMgr->m_hasNewImage = true;
        //     }
        // }

#endif

    return;
}