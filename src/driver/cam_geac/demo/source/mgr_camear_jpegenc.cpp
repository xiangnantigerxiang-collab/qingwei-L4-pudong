#ifdef SUPPORT_JPEGENC
#include "../inc/mgr_camera_jpegenc.h"
#include "../inc/libyuv.h"

/* 
JPEG编码回调函数，接收编码后的数据并发布/保存
data：指向CMgrCameraJpegEnc实例的指针
*/
void CALLBACK JpegEncCallBack(unsigned char *data, int datalen, void *userdata)
{
	CMgrCameraJpegEnc *pOper = (CMgrCameraJpegEnc *)userdata;
	if (pOper == NULL)
	{
		return;
	}

	string strName;
	{
		std::lock_guard<std::mutex> lock(pOper->m_mutex); // 线程安全访问文件名队列
		if (pOper->m_qFileName.empty())
		{
			return;
		}

		strName = pOper->m_qFileName.front();
		pOper->m_qFileName.pop();
	}
#ifdef SUPPORT_ROS1_JPG
	struct timespec stTime;
	int nChan = 0;
	// 从文件路径字符串中解析通道号和时间戳
	sscanf(strName.c_str(), "demo/pic/%d_%010ld%09ld.jpg", &nChan, &stTime.tv_sec, &stTime.tv_nsec);
	// 发布ROS压缩图像消息
	pOper->m_pRosPublisher->Publisher(stTime, data, datalen); // 发布ROS压缩图像消息
	// 更新相机状态（表示有数据）
	if (pOper->m_pStatusPublisher) {
		pOper->m_pStatusPublisher->UpdateDataReceived();
	}
	// 处理ROS回调
	ros::spinOnce();
#elif SUPPORT_ROS2_JPG
	struct timespec stTime;
	int nChan = 0;
	// 从文件路径字符串中解析通道号和时间戳
	sscanf(strName.c_str(), "demo/pic/%d_%010ld%09ld.jpg", &nChan, &stTime.tv_sec, &stTime.tv_nsec);
	pOper->m_pRosPublisher->Publisher(stTime, data, datalen); // 发布ROS压缩图像消息
	rclcpp::spin_some(pOper->m_pRosPublisher); // 必要调用：触发ROS2事件循环处理消息发布
#else
	FILE *pf = fopen(strName.c_str(), "w"); // 以写模式打开文件
	if (pf)
	{
		fwrite(data, 1, datalen, pf); // 写入JPEG数据到文件
		fclose(pf);
	}
	else
	{
		debug_err("open :%s failed\n", strName.c_str()); // 记录文件打开失败
	}
#endif 
}

/* 
构造函数初始化摄像头参数
dwChan：通道号
dwWidth/dwHeight：分辨率
dwQuality：JPEG编码质量
*/
CMgrCameraJpegEnc::CMgrCameraJpegEnc(int dwChan, int dwWidth, int dwHeight, int dwQuality)
{
	m_dwChan = dwChan;
	m_dwWidth = dwWidth;
	m_dwHeight = dwHeight;
	m_dwQuality = dwQuality;
	m_pEncHandle = NULL; // 初始化编码句柄为空
}

CMgrCameraJpegEnc::~CMgrCameraJpegEnc()
{
}

/* 
创建JPEG编码器句柄
返回：成功true/失败false
*/
bool CMgrCameraJpegEnc::createHandle()
{
	JPEGENC_PARA para;
	memset(&para, 0, sizeof(para)); // 清零参数结构体
	para.pixfmt = 0;               // 默认像素格式
	para.userbuf = 1;              // 使用用户提供的缓冲区
	para.bufsize = 0;              // 自动计算缓冲区大小
	para.width = m_dwWidth;        // 设置分辨率
	para.height = m_dwHeight;
	para.quality = m_dwQuality;    // 设置编码质量
	para.mode = 2;                 // 模式2表示实时编码
	para.gpuid = 0;                // 使用GPU 0进行加速

	m_pEncHandle = JPEGENC_CreateHandle(&para);
	if (m_pEncHandle == NULL)
	{
		debug_err("JPEGENC_CreateHandle failed,chan[%u]\n", m_dwChan);
		return false;
	}

	JPEGENC_SetDataCallBack(m_pEncHandle, JpegEncCallBack, this); // 注册编码完成回调

#ifdef SUPPORT_ROS1_JPG
	// 创建ROS1话题名称（与ROS2相同）
	char m_szTopic[128];
	snprintf(m_szTopic, sizeof(m_szTopic), "cam%d/compressed", m_dwChan);
	m_pRosPublisher = std::make_shared<CCameraPublisher>(m_szTopic);
	// 创建相机状态话题名称：cam<channel>/status
	char m_szStatusTopic[128];
	snprintf(m_szStatusTopic, sizeof(m_szStatusTopic), "cam%d/status", m_dwChan);
	m_pStatusPublisher = std::make_shared<CCameraStatusPublisher>(m_szStatusTopic);
#elif SUPPORT_ROS2_JPG
	// 创建ROS2话题名称格式：cam<channel>/compressed
	char m_szTopic[128];
	snprintf(m_szTopic, sizeof(m_szTopic), "cam%d/compressed", m_dwChan);
	m_pRosPublisher = std::make_shared<CCameraPublisher>(m_szTopic); // 初始化ROS发布者
#endif

	return true;
}

/* 
初始化编码器资源
1. 分配YUV缓冲区
2. 创建编码器句柄
返回：成功true/失败false
*/
bool CMgrCameraJpegEnc::Init()
{
	m_pYuv = new unsigned char[m_dwWidth * m_dwHeight * 3 / 2]; // 分配YUV420p缓冲区空间
	if (!m_pYuv)
	{
		debug_err("calloc yuv buffer failed.\n");
		return false;
	}

	createHandle(); // 创建编码器句柄
	return true;
}

/* 
释放编码器资源
返回：成功true
*/
bool CMgrCameraJpegEnc::Release()
{
	destroyHandle(); // 销毁编码器句柄
	return true;
}

/* 
保存YUV数据为JPEG图像
nChan：通道号
stTime：时间戳
nWidth/nHeight：图像尺寸
pData：YUV422 YUY2格式数据指针
nDatalen：数据长度
返回：成功true/失败false
*/
bool CMgrCameraJpegEnc::Save(int nChan, struct timespec stTime, int nWidth, int nHeight, unsigned char *pData, int nDatalen)
{
	if (pData == NULL)
	{
		return false;
	}

	char szPath[512];
	memset(szPath, 0, sizeof(szPath));
	// 生成文件路径格式：demo/pic/<通道号>_<时间戳>.jpg
	snprintf(szPath, sizeof(szPath), "demo/pic/%d_%010ld%09ld.jpg", nChan, stTime.tv_sec, stTime.tv_nsec);

	std::lock_guard<std::mutex> lock(m_mutex); // 保护YUV转换和编码操作

	// 将YUY2格式转换为I420格式（JPEG编码器要求）
	libyuv::YUY2ToI420(
		pData, 2 * nWidth,              // YUY2数据及步长
		m_pYuv, nWidth,                 // Y分量缓冲区及步长
		m_pYuv + nWidth * nHeight, nWidth / 2,  // U分量缓冲区及步长
		m_pYuv + 5 * nWidth * nHeight / 4, nWidth / 2, // V分量缓冲区及步长
		nWidth, nHeight);

	if (JPEGENC_InputData(m_pEncHandle, (unsigned char *)m_pYuv, nWidth*nHeight*3/2) < 0)
	{
		debug_err("JPEGENC_InputData failed\n");
		return false;
	}
	m_qFileName.push(szPath); // 保存文件路径到队列供回调处理
	return true;
}

/* 
销毁编码器句柄
返回：成功true
*/
bool CMgrCameraJpegEnc::destroyHandle()
{
	if (m_pEncHandle)
	{
		JPEGENC_DestroyHandle(m_pEncHandle);
		m_pEncHandle = NULL;
	}
	return true;
}
#endif
