#ifndef _H_V4L2CAM_H
#define _H_V4L2CAM_H
#include <iostream>
#include <string>
#include <string.h>
#include <queue>
#include <linux/videodev2.h>
#include <thread>
#include <mutex>
#include <semaphore.h>
#include "camera_type.h"
using namespace std;
#define IO_METHOD_MMAP 0
#define IO_METHOD_USERPTR 1
#define IO_METHOD_DMA 2
#ifdef SUPPORT_RTSPS
#include "rtsps_push.h"
#endif
#ifdef RB_V4L2_DMA
#include "NvBufSurface.h"
#endif
typedef struct
{
        void *start;
        size_t length;
} V4l2Buffer;

#ifdef RB_V4L2_DMA
typedef struct
{
        unsigned char *start;
        unsigned int size;
        int dmabuff_fd;
} nv_buffer;

typedef struct
{
        unsigned int v4l2_pixfmt;
        NvBufSurfaceColorFormat nvbuff_color;
} nv_color_fmt;
#endif

#define TIMESTAMP_MODE_HARD 0
#define TIMESTAMP_MODE_HARD_ONCE 1
#define TIMESTAMP_MODE_SOFT 2

typedef void (*PV4L2_DATACALLBACK)(int nChan, struct timespec stTime, int nWidth, int nHeight, unsigned char *pData, int nDatalen, void *pUserData);

class CV4l2Cam
{
public:
        CV4l2Cam(int dwPipeId, int dwVideoIndex, int dwWidth, int dwHeight, int dwFps ,int format);
        ~CV4l2Cam();
        int Init();
        int Release();
        int StartAcquire();
        int StopAcquire();
        void GrabRoutine();
        void SetDataCallback(PV4L2_DATACALLBACK cb, void *pUserData);
        int GetFps() { return m_nFps; }
        int ClearBuffer();

protected:
        int grabImg();
        int startCapture();
        int stopCapture();
        void rtcpuToRealtime(timeval rtcpu_time, timespec *real_time);
        bool initDevice();
        bool releaseDevice();
        bool clearBuffer();
        bool initMmap();
        bool initUserp();
        bool initDma();
        bool setVideoFmt();
        bool checkCapabilities();
        int xioctl(int fh, int request, void *arg);
        int GetOffset();

private:
        bool m_bRunning;
        int m_dwIoType; // 访问IO的方式
        int m_dwFault;
        int m_nChan;
        int m_nFps;
        int m_nErr;
        long m_llSkippedFrameNum; // 由于时间戳到达延迟，导致跳过的帧数
        uint32_t m_dwFrameCnt;
        int32_t m_videoFd;
        uint32_t m_dwHeight;
        uint32_t m_dwWidth;
        string m_strDevName;
        V4l2Buffer *m_pBuffers;
        std::deque<timespec> m_queueTs;
        sem_t semTrig;
        std::mutex m_mutex;
        std::thread *m_pGrabThread;
        PV4L2_DATACALLBACK m_Cb;
        void *m_pUserData;
        bool m_nFirstCapture;
        uint64_t m_lstFrameTimestamp;
        long m_lNsecOffset;
       
        struct timeval  m_lstV4l2Timestamp;         //上一帧图像的v4l2时间戳
        int m_dwFormat;
#ifdef RB_V4L2_DMA
        nv_buffer *m_pNvbuff;
        NvBufSurface *m_pSurf = NULL;
#endif
};
#endif
