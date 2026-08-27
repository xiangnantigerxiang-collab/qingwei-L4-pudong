#include <chrono>
#include <memory>
#include <string>
#include <unistd.h>
#include <thread>
#include <mutex>
#include <sys/time.h>

#include "libyuv.h"
#include "jpeg_encode_api.h"
#include "mgr_camera_jpegenc.h"
#include "camera_type.h"
#include "rtsps_push.h"
#include "vin_log.h"
#ifdef SUPPORT_ROS1_YUV
#include "camera_publish.h"
#endif
#ifdef SUPPORT_ROS2_YUV
#include "camera_publish.h"
#endif
#ifdef SUPPORT_D415_ALIGN
#include "librealsense/realsense_camera_align.h"
#endif
#ifdef SUPPORT_SHOWIMG
#include <opencv2/opencv.hpp>
#include <X11/Xlib.h>
struct ImageData {
    int nChan;                  // 通道号（对应窗口）
    long llTimestamp;            // 时间戳（用于显示）
    long diff_ms;                 // 延迟（用于显示）
    cv::Mat imgRGB;             // 图像数据
};
#endif
#include "v4l2cam.hpp"
using namespace std;

class CCameraMgr
{
public:
    CCameraMgr(int dwPipeId, int dwVideoIndex, int dwWidth, int dwHeight, int dwFps, int format);
    ~CCameraMgr();
    bool Init();
    void Start();

    /**
     * @brief 检查相机是否超时，并更新发布状态
     * @param timeoutMs 超时时间（毫秒），默认1000ms
     * @return 总是返回true
     */
    bool CheckTimeout(int timeoutMs = 1000);

    /* 相机接收图像回调并发布消息,这里可以拿到数据，进而去处理，比如编码，或者直接推流 */
    static void callbackImage(int nChan, struct timespec stTime, int nWidth, int nHeight, unsigned char *pData, int nDatalen, void *pUserData);

#ifdef SUPPORT_SHOWIMG
    bool doShowImage();
    bool SetNum(int dwNUm);//设置相机总数
#endif

#ifdef SUPPORT_JPEGENC
    friend void CALLBACK JpegEncCallBack(unsigned char *data, int datalen, void *userdata);
#endif

#ifdef SUPPORT_D415_ALIGN
    std::vector<uint8_t> output_aligned;
    CMgrCameraAlign *m_pCameraAlign;
#endif

    TImageWrap m_stuImage; // 图像数据
    long m_llTimestamp;

private:
    bool creatHandle();   // 创建相机实例
    bool initHandle();    // 初始化相机
    void setCamPublish(); // 设置相机图片发布回调函数

private:
    std::mutex mtx;          // 互斥锁对象
    CV4l2Cam *m_pCameraBase; // 相机实例
    unsigned char *m_pRgbBuf; // 相机图片数据转化为rgb的buf
    unsigned char *m_pYuvBuf; // 相机图片原始yuv数据buf
    uint32_t m_dwHeight;
    uint32_t m_dwWidth;
    int m_nChan;
    int m_nFps;
    int m_dwFormat;
    int m_dwVideoIndex;
    long m_llFrame;
    long m_oldllTimestamp{0};

#ifdef SUPPORT_SHOWIMG
    unsigned char* m_pYuv420;//原始的YUV420数据
    int m_dwWinWidth;
    int m_dwWinHeight;
    int m_dwCamNum;//相机总数
public:
    ImageData m_ImageData;
    bool m_hasNewImage = false;      // 标记有新图
    std::mutex m_mutex;     // 互斥锁，保证线程安全
#endif

#ifdef SUPPORT_JPEGENC
    CMgrCameraJpegEnc *m_pCamerajpeg; // jpeg编码器
#endif

#ifdef SUPPORT_RTSPS
    CVideoPush *m_pVideoPush;
#endif

#ifdef SUPPORT_ROS1_YUV
    std::shared_ptr<CCameraPublisher> m_pRosPublisher;
#endif

#ifdef SUPPORT_ROS2_YUV
    std::shared_ptr<CCameraPublisher> m_pRosPublisher;
#endif

};
