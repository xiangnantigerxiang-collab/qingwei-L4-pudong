#ifdef SUPPORT_RTSPS
#include "../inc/rtsps_push.h" // RTSPS协议相关头文件

// 视频编码回调函数，用于处理编码后的H265数据并封装为RTP包
void CALLBACK videoEncCallBack(unsigned char type, unsigned char *data, int datalen, void *userdata)
{
    CVideoPush *pHandle = (CVideoPush *)userdata;
    if (NULL == pHandle->m_pRtpPayHandle)
    { // 检查RTP封装句柄有效性
        debug_info( "RtppayHandle is NULL.\n");
        return;
    }

    // RTP封装处理参数初始化
    RTPPAY_PROCESS_PARAM stuRtpProcessPara;
    memset(&stuRtpProcessPara, 0, sizeof(stuRtpProcessPara));
    stuRtpProcessPara.pInData = data;              // 输入原始H265数据指针
    stuRtpProcessPara.dwInDataLen = datalen;       // 数据长度
    stuRtpProcessPara.byStreamType = STREAM_TYPE_VIDEO; // 指定视频流类型
    RTPPAY_Process(pHandle->m_pRtpPayHandle, &stuRtpProcessPara); // 执行RTP封装

    unsigned char *pOut_data = stuRtpProcessPara.pOutData; // 输出RTP包起始地址
    unsigned int current_rtppacket_size = 0;
    int sumsize = 0;

    // 遍历所有生成的RTP分片包
    for (int i = 1; i <= stuRtpProcessPara.dwPacketNum; i++)
    {
        current_rtppacket_size = ntohl(*(int *)pOut_data); // 解析RTP包头中的有效载荷长度（网络字节序转主机字节序）

        // 补充RTSPS协议的前导字节（0x24标记）
        pOut_data[0] = 0x24; // 协议标识符
        pOut_data[1] = 0;    // 保留字段
        pOut_data[2] = current_rtppacket_size >> 8; // 长度高8位
        pOut_data[3] = current_rtppacket_size & 0xff; // 长度低8位

        sumsize += (current_rtppacket_size + 4); // 累计总数据量（包含前导字段）
        pOut_data += current_rtppacket_size + 4; // 移动指针到下一个RTP包
    }

    // 发送处理后的RTP数据到RTSPS服务器
    RTSPServer_SendRealSteam(pHandle->m_dwChan, 0, stuRtpProcessPara.pOutData, stuRtpProcessPara.dwOutDataLen);
    debug_info( "------------chan:%d,RTSPServer_SendRealSteam-----------\n",pHandle->m_dwChan);
}

// 构造函数初始化基础参数
CVideoPush::CVideoPush(int dwChan, int dwWidth, int dwHeight, int dwFps)
{
    m_dwChan = dwChan;      // 通道号
    m_dwWidth = dwWidth;    // 视频宽度
    m_dwHeight = dwHeight;  // 视频高度
    m_dwFps = dwFps;        // 帧率
    m_pEncHandle = NULL;    // 编码器句柄初始化
    m_pRtpPayHandle = NULL; // RTP封装句柄初始化
}

// 析构函数（当前无资源释放逻辑）
CVideoPush::~CVideoPush()
{
}

// 核心初始化函数
bool CVideoPush::Init()
{
    // RTSPS服务器初始化
    TRtspServerPara struPara;
    memset(&struPara, 0, sizeof(TRtspServerPara));
    struPara.byDbg = 0; // 调试模式关闭
    strncpy(struPara.szUserName, "admin", sizeof(struPara.szUserName)); // 用户名
    strncpy(struPara.szPasswd, "rb123456", sizeof(struPara.szPasswd)); // 密码
    struPara.nServerPort = RTSPS_LISTEN_PORT; // 服务器监听端口
    struPara.nMinPort = 22000; // RTP最小端口
    struPara.nMaxPort = 23000; // RTP最大端口
    RTSPServer_Init(&struPara); // 初始化RTSPS服务

    // 设置流媒体参数
    TRtspServerStreamInfo stuStreamInfo;
    stuStreamInfo.byVideoPayload = 96; // H265的RTP负载类型（RFC7798）
    stuStreamInfo.byAudioPayload = 0;  // 无音频流
    stuStreamInfo.dwAudioSSrc = 100;   // 音频SSRC（占位值）
    stuStreamInfo.dwVideoSSrc = 200;   // 视频SSRC（需确保全局唯一）
    stuStreamInfo.dwVideoFormat = RTSPS_VIDEO_STREAM_TYPE_H265; // 指定视频编码格式
    RTSPServer_SetStreamInfo(m_dwChan, 0, &stuStreamInfo); // 绑定通道参数
    RTSPServer_Start(); // 启动RTSPS服务

    // RTP封装模块初始化
    RTPPAY_Init();
    RTPPAY_PARAM param;
    memset(&param, 0, sizeof(RTPPAY_PARAM));
    param.dwMtu = 1400; // MTU限制（典型以太网1500，保留头部空间）
    param.byVideoFps = m_dwFps; // 帧率同步编码器参数
    param.byStreamType = STREAM_TYPE_VIDEO; // 视频流类型
    param.byVideoPayloadType = 96; // 与RTSPS参数一致
    param.byVideoStreamType = VIDEO_STREAM_TYPE_H265; // 视频编码格式
    param.dwAudioSsrc = 100; // 音频SSRC（占位）
    param.dwVideoSsrc = 200; // 视频SSRC（需确保全局唯一）
    param.dwPrivateSsrc = 300; // 专用SSRC（备用）
    m_pRtpPayHandle = RTPPAY_CreateHandle(&param); // 创建RTP封装句柄
    if (NULL == m_pRtpPayHandle)
    {
       debug_err( "------------rtppay_handle is failed.-------\n");
    }

    // 视频编码器初始化
    VIDEOENC_Init();
    VIDEOENC_PARA para;
    memset(&para, 0, sizeof(VIDEOENC_PARA));
    para.encfmt = 1; // 编码格式（1=H265）
    para.fps = param.byVideoFps; // 帧率参数
    para.pixfmt = 0; // 像素格式（默认）
    para.dma = 1;    // 开启DMA加速
    para.insert_sps_pps_at_idr = 1; // 在IDR帧插入SPS/PPS（RTSPS协议要求）
    para.alliframe = 0; // 关闭全I帧模式
    para.bitrate = 4 * 1024 * 1024; // 码率设置（4Mbps）
    para.ifi = param.byVideoFps; // I帧间隔（与帧率一致）
    para.width = m_dwWidth; // 分辨率宽度
    para.height = m_dwHeight; // 分辨率高度
    para.bufs = 1; // 编码缓冲区数量
    m_pEncHandle = VIDEOENC_CreateHandle(&para); // 创建编码器句柄
    if (m_pEncHandle == NULL)
    {
        debug_err( "VIDEOENC_CreateHandle failed,dwChan[%u]", m_dwChan);
        return false;
    }

    // 注册编码回调
    VIDEOENC_SetDataCallBack(m_pEncHandle, videoEncCallBack, this); // 绑定回调函数
    return true;
}

// 输入原始图像数据到编码器
bool CVideoPush::InputData(TImageInfo* pImgInfo)
{
    VIDEOENC_FRAME_INFO stuFi;
    memset(&stuFi,0,sizeof(stuFi));
    // 时间戳处理（兼容两种时间戳类型）
    stuFi.sec = pImgInfo->byTimestampType == 0? pImgInfo->dwTimestampSec : pImgInfo->dwTimestampSec;
    stuFi.nan = pImgInfo->dwTimestampNsec; // 纳秒级时间戳
    stuFi.exposure = pImgInfo->dwExposureUsec; // 曝光时间（摄像头相关）

    // 将图像数据输入编码器（缓冲区超时10ms）
    if (VIDEOENC_InputData2(m_pEncHandle, &stuFi, (unsigned char*)(pImgInfo + 1), pImgInfo->dwDataLength, 10) < 0)
    {
        debug_err( "VIDEOENC_InputData failed, dwChan[%u]\n", m_dwChan);
        return false;
    }
    return true;
}
#endif
