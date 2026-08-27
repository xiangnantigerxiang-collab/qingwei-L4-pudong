#pragma once
#include <queue>
#include <bits/stdc++.h>
#include "jpeg_encode_api.h"
#include "camera_type.h"
#include "vin_log.h"
#ifdef SUPPORT_ROS1_JPG
#include "camera_publish.h"
#endif
#ifdef SUPPORT_ROS2_JPG
#include "camera_publish.h"
#endif
using namespace std;
	

class CMgrCameraJpegEnc
{
public:
	CMgrCameraJpegEnc(int dwChan, int dwWidth, int dwHeight, int dwQuality);
	~CMgrCameraJpegEnc();
	virtual bool Init();
	virtual bool Release();
	bool Save(int nChan,struct timespec stTime,int nWidth,int nHeight,unsigned char *pData,int nDatalen);
	friend void CALLBACK JpegEncCallBack(unsigned char *data,int datalen,void *userdata);
#ifdef SUPPORT_ROS1_JPG
	/**
	 * @brief 获取相机状态发布者指针
	 * @return 状态发布者指针
	 */
	std::shared_ptr<CCameraStatusPublisher> GetStatusPublisher() { return m_pStatusPublisher; }

	/**
	 * @brief 检查并发布超时状态（封装方法，避免宏守卫问题）
	 * @param timeoutMs 超时时间（毫秒），默认1000ms
	 */
	void CheckAndPublishTimeout(int timeoutMs = 1000) {
		if (m_pStatusPublisher) {
			m_pStatusPublisher->IsTimeout(timeoutMs);
		}
	}
#endif
protected:
	bool createHandle();
	bool destroyHandle();
private:
	int					m_dwChan;
	int					m_dwWidth;
	int					m_dwHeight;
	int 				m_dwQuality;
	void*				m_pEncHandle;
	queue<string> 		m_qFileName;
	mutex               m_mutex;
	unsigned char*      m_pYuv;
#ifdef SUPPORT_ROS1_JPG
    std::shared_ptr<CCameraPublisher> m_pRosPublisher;
    std::shared_ptr<CCameraStatusPublisher> m_pStatusPublisher;  ///< 相机状态发布者
#endif
#ifdef SUPPORT_ROS2_JPG
    std::shared_ptr<CCameraPublisher> m_pRosPublisher;
#endif
};
