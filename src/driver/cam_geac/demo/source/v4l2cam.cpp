#include <getopt.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <time.h>
#include <assert.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <dlfcn.h>
#include <poll.h>
#include "../inc/vin_log.h"
#include "../inc/v4l2cam.hpp"
#include "../inc/rtsps_server_api.h"
#include "../inc/libyuv.h"

#ifdef RB_V4L2_DMA
static nv_color_fmt nvcolor_fmt[] =
{
    /* TODO: add more pixel format mapping */
    {V4L2_PIX_FMT_UYVY, NVBUF_COLOR_FORMAT_UYVY},
    {V4L2_PIX_FMT_VYUY, NVBUF_COLOR_FORMAT_VYUY},
    {V4L2_PIX_FMT_YUYV, NVBUF_COLOR_FORMAT_YUYV},
    {V4L2_PIX_FMT_YVYU, NVBUF_COLOR_FORMAT_YVYU},
    {V4L2_PIX_FMT_GREY, NVBUF_COLOR_FORMAT_GRAY8},
    {V4L2_PIX_FMT_YUV420M, NVBUF_COLOR_FORMAT_YUV420},
};

static NvBufSurfaceColorFormat get_nvbuff_color_fmt(unsigned int v4l2_pixfmt)
{
    unsigned i;

    for (i = 0; i < sizeof(nvcolor_fmt) / sizeof(nvcolor_fmt[0]); i++)
    {
        if (v4l2_pixfmt == nvcolor_fmt[i].v4l2_pixfmt)
            return nvcolor_fmt[i].nvbuff_color;
    }

    return NVBUF_COLOR_FORMAT_INVALID;
}
#endif

// buf个数
#define V4L2_BUFFER_LENGHT 2
// 图像格式
#define V4L2_VIDEO_FORMAT V4L2_PIX_FMT_YUYV
// V4L2_PIX_FMT_SRGGB12
//  计算timeval 时间差
#define difftimeval(end, beginning) ((end.tv_sec - beginning.tv_sec) * 1000000 + end.tv_usec - beginning.tv_usec)


// 静态变量用于存储设备前缀和初始化控制
static pthread_once_t prefix_once = PTHREAD_ONCE_INIT;
static const char *device_prefix = NULL;

// 设备前缀初始化函数
static void init_device_prefix() {
    // 检查是否存在/dev/gmslcam0设备节点
    if (access("/dev/gmslcam0", F_OK) == 0) {
        device_prefix = "/dev/gmslcam";
    } else {
        device_prefix = "/dev/video";
    }
//     printf("Using device prefix: %s\n", device_prefix);
}

CV4l2Cam::CV4l2Cam(int dwPipeId, int dwVideoIndex, int dwWidth, int dwHeight, int dwFps, int format)
{
        m_videoFd = -1;
        m_nChan = dwPipeId;
        m_dwWidth = dwWidth;
        m_dwHeight = dwHeight;
        m_nFps = dwFps;
        m_dwFormat = format;
        char tmp[64] = {0};
        pthread_once(&prefix_once, init_device_prefix);
        (void)snprintf(tmp, sizeof(tmp), "%s%d", device_prefix, dwVideoIndex);
        m_strDevName = tmp;
        m_pBuffers = NULL;
#ifdef RB_V4L2_DMA
        m_dwIoType = IO_METHOD_DMA;
        m_pNvbuff = NULL;
#else
        m_dwIoType = IO_METHOD_MMAP;
#endif
        m_pGrabThread = nullptr;
        m_bRunning = false;
        m_pUserData = nullptr;
        m_Cb = nullptr;
        m_nFirstCapture = true;
        m_dwFrameCnt = 0;
        m_llSkippedFrameNum = 0;
        m_lNsecOffset = 0;
}

CV4l2Cam::~CV4l2Cam()
{
}

int CV4l2Cam::Init()
{
        do
        {
                GetOffset();

                // 阻塞形式打开设备
                m_videoFd = open(m_strDevName.c_str(), O_RDWR /* required */ /*| O_NONBLOCK*/, 0);
                if (-1 == m_videoFd)
                {
                        debug_err("cannot open %s:,errno:%d, %s\n", m_strDevName.c_str(), errno, strerror(errno));
                        break;
                }
                // 校验能力级
                if (checkCapabilities() == false)
                {
                        debug_err("checkCapabilities failed,devname=%s\n", m_strDevName.c_str());
                        break;
                }
                // 设备输出格式
                if (setVideoFmt() == false)
                {
                        debug_err("setVideoFmt failed,devname=%s\n", m_strDevName.c_str());
                        break;
                }
                usleep(10 * 1000);

                // 初始化内存
                if (initDevice() == false)
                {
                        debug_err("initDevice failed,devname=%s\n", m_strDevName.c_str());
                        break;
                }

                return 0;

        } while (0);

        if (m_videoFd != -1)
        {
                close(m_videoFd);
                m_videoFd = -1;
        }

        return 0;
}

int CV4l2Cam::Release()
{
        releaseDevice();
        return 0;
}

int CV4l2Cam::StartAcquire()
{
        m_bRunning = true;
        m_pGrabThread = new std::thread(&CV4l2Cam::GrabRoutine, this);
        return 0;
}

int CV4l2Cam::StopAcquire()
{
        stopCapture();
        m_bRunning = false;
        if (m_pGrabThread != nullptr)
        {
                m_pGrabThread->join();
                delete m_pGrabThread;
                m_pGrabThread = nullptr;
        }
        return 0;
}

void CV4l2Cam::SetDataCallback(PV4L2_DATACALLBACK cb, void *pUserData)
{
        m_pUserData = pUserData;
        m_Cb = cb;
}

int CV4l2Cam::ClearBuffer()
{
        return clearBuffer();
}

int CV4l2Cam::grabImg()
{
        int bRet = false;
        unsigned char *srcData = NULL;
        int srcDatalen = 0;
        int retsel = 0;

        struct timeval t1, t2;
        gettimeofday(&t1, nullptr);

        /* 将已经捕获好视频的内存拉出已捕获视频的队列 */
        fd_set rset;
        fd_set eset;
        for (int i = 0; i < 10; i++)
        {
                FD_ZERO(&rset);
                FD_ZERO(&eset);
                FD_SET(m_videoFd, &rset);
                FD_SET(m_videoFd, &eset);
                struct timeval tv;
                tv.tv_sec = 0;
                tv.tv_usec = 10 * 1000;
                retsel = select(m_videoFd + 1, &rset, NULL, &eset, &tv);
                if (retsel > 0)
                {
                        break;
                }
        }

        // 异常,或者超时
        if (FD_ISSET(m_videoFd, &rset) == false)
        {
                if (FD_ISSET(m_videoFd, &eset))
                {

                        debug_err("select failed,dev=%s,errno=%d,errstr=%s\n", m_strDevName.c_str(), errno, strerror(errno));
                }
                else
                {

                        debug_err("select timeout,dev=%s\n", m_strDevName.c_str());
                }

                struct timespec ts = {0, 0};
                if (m_Cb)
                {
                        //m_Cb(m_nChan, ts, m_dwWidth, m_dwHeight, nullptr, 0, m_pUserData);
                }
                return -1;
        }

        gettimeofday(&t2, nullptr);

        int nSelDiff = (t2.tv_sec - t1.tv_sec) * 1000 + (t2.tv_usec - t1.tv_usec) / 1000;
        if (nSelDiff >= 1.5 * 1000 / m_nFps)
        {
                debug_warn("chan=%d,select cost=%dms\n", m_nChan, nSelDiff);
        }

        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(struct v4l2_buffer));

        switch (m_dwIoType)
        {
        case IO_METHOD_MMAP:
                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory = V4L2_MEMORY_MMAP;
                if (-1 == xioctl(m_videoFd, VIDIOC_DQBUF, &buf)) // 从缓存区取出一个缓存帧
                {
                        bRet = false;
                        debug_err("xioctl VIDIOC_DQBUF failed,dev=%s,error:%d:%s\n", m_strDevName.c_str(), errno, strerror(errno));
                        return -1;
                }
                srcData = (unsigned char *)m_pBuffers[buf.index].start;
                srcDatalen = buf.bytesused;
                break;
        case IO_METHOD_USERPTR:
                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory = V4L2_MEMORY_USERPTR;
                if (-1 == xioctl(m_videoFd, VIDIOC_DQBUF, &buf))
                {
                        bRet = false;
                        debug_err("xioctl VIDIOC_DQBUF failed,dev=%s,error:%d:%s\n", m_strDevName.c_str(), errno, strerror(errno));
                        return -1;
                }
                srcData = (unsigned char *)buf.m.userptr;
                srcDatalen = buf.bytesused;
                break;
#ifdef RB_V4L2_DMA
        case IO_METHOD_DMA:
                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory = V4L2_MEMORY_DMABUF;
                if (-1 == xioctl(m_videoFd, VIDIOC_DQBUF, &buf))
                {
                        bRet = false;
                        debug_err("xioctl VIDIOC_DQBUF failed,dev=%s,error:%d:%s\n", m_strDevName.c_str(), errno, strerror(errno));
                        return -1;
                }
                if (-1 == NvBufSurfaceFromFd(m_pNvbuff[buf.index].dmabuff_fd,
                        (void**)(&m_pSurf)))
                    debug_err("Cannot get NvBufSurface from fd");
                /* Cache sync for VIC operation since the data is from CPU */
                if (-1 == NvBufSurfaceSyncForDevice(m_pSurf, 0, 0))
                        debug_err("Cannot sync output buffer");
                srcData = (unsigned char *)m_pSurf->surfaceList[0].mappedAddr.addr[0];
                srcDatalen = m_pSurf->surfaceList[0].dataSize;
                break;
#endif
        default:
                break;
        }

        struct timeval tv;
        gettimeofday(&tv, nullptr);

        m_lstV4l2Timestamp.tv_sec = buf.timestamp.tv_sec;
        m_lstV4l2Timestamp.tv_usec = buf.timestamp.tv_usec;

        if ((buf.timestamp.tv_sec == 0 && buf.timestamp.tv_usec == 0) || m_llSkippedFrameNum < 0)
        {
                debug_err("[chan:%d]systime:%ld.%06ld, buftime:%ld.%06ld\n",
                        m_nChan, tv.tv_sec, tv.tv_usec, buf.timestamp.tv_sec, buf.timestamp.tv_usec);
        }
        else
        {
                struct timespec ft;
                rtcpuToRealtime(buf.timestamp, &ft);
                if (m_Cb)
                {
                        m_Cb(m_nChan, ft, m_dwWidth, m_dwHeight, srcData, srcDatalen, m_pUserData);
                }
        }
        // 将buf重新入队
        if (-1 == xioctl(m_videoFd, VIDIOC_QBUF, &buf))
        {
                debug_err("xioctl VIDIOC_QBUF failed,dev=%s,error:%d:%s\n", m_strDevName.c_str(), errno, strerror(errno));
        }

        m_dwFrameCnt++;
        return bRet;
}

int CV4l2Cam::startCapture()
{
        if (m_videoFd < 0)
        {
                debug_err("video fd is invlaid,devname=%s\n", m_strDevName.c_str());
                return -1;
        }

        enum v4l2_buf_type type;

        switch (m_dwIoType)
        {
        case IO_METHOD_MMAP:
                for (int i = 0; i < V4L2_BUFFER_LENGHT; ++i)
                {
                        struct v4l2_buffer buf;
                        memset(&buf, 0, sizeof(struct v4l2_buffer));

                        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                        buf.memory = V4L2_MEMORY_MMAP;
                        buf.index = i;

                        if (-1 == xioctl(m_videoFd, VIDIOC_QBUF, &buf))
                        {
                                debug_err("xioctl VIDIOC_QBUF,devname=%s\n", m_strDevName.c_str());
                                return -1;
                        }
                }
                break;
        case IO_METHOD_USERPTR:
                for (int i = 0; i < V4L2_BUFFER_LENGHT; ++i)
                {
                        struct v4l2_buffer buf;
                        memset(&buf, 0, sizeof(struct v4l2_buffer));

                        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                        buf.memory = V4L2_MEMORY_USERPTR;
                        buf.index = i;
                        buf.m.userptr = m_pBuffers[i].length;

                        if (-1 == xioctl(m_videoFd, VIDIOC_QBUF, &buf))
                        {
                                debug_err("xioctl VIDIOC_QBUF,devname=%s\n", m_strDevName.c_str());
                                return -1;
                        }
                }
                break;
#ifdef RB_V4L2_DMA
        case IO_METHOD_DMA:
                for (unsigned int index = 0; index < V4L2_BUFFER_LENGHT; index++)
                {
                        struct v4l2_buffer buf;

                        memset(&buf, 0, sizeof buf);
                        buf.index = index;
                        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                        buf.memory = V4L2_MEMORY_DMABUF;
                        buf.m.fd = (unsigned long)m_pNvbuff[index].dmabuff_fd;
                        if (ioctl(m_videoFd, VIDIOC_QBUF, &buf) < 0)
                        {
                                debug_err("xioctl VIDIOC_QBUF,devname=%s\n", m_strDevName.c_str());
                                return -1;
                        }
                }
                break;
#endif
        default:
                break;
        }

        // set streaming on   将缓存帧放入队列
        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (-1 == xioctl(m_videoFd, VIDIOC_STREAMON, &type))
        {
                debug_err("xioctl VIDIOC_STREAMON,devname=%s\n", m_strDevName.c_str());
                return -1;
        }
        // 实际测试，底层有数据缓存
        clearBuffer();
        return 0;
}

int CV4l2Cam::stopCapture()
{
        enum v4l2_buf_type type;
        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (-1 == xioctl(m_videoFd, VIDIOC_STREAMOFF, &type))
        {
                debug_err("xioctl VIDIOC_STREAMOFF,devname=%s\n", m_strDevName.c_str());
                return -1;
        }
        return 0;
}

bool CV4l2Cam::clearBuffer()
{
        struct v4l2_buffer buf;

        /* 将已经捕获好视频的内存拉出已捕获视频的队列 */
        for (int i = 0; i < V4L2_BUFFER_LENGHT; i++)
        {
                memset(&buf, 0, sizeof(struct v4l2_buffer));
                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory = V4L2_MEMORY_MMAP;

                fd_set rset;
                FD_ZERO(&rset);
                FD_SET(m_videoFd, &rset);
                struct timeval tv;
                tv.tv_sec = 0;
                tv.tv_usec = 100 * 1000;
                int retsel = select(m_videoFd + 1, &rset, NULL, NULL, &tv);
                if (retsel <= 0)
                {
                        break;
                }
                if (-1 == xioctl(m_videoFd, VIDIOC_DQBUF, &buf))
                {
                        debug_err("xioctl VIDIOC_DQBUF failed,dev=%s,error:%d:%s\n", m_strDevName.c_str(), errno, strerror(errno));
                        break;
                }

                if (-1 == xioctl(m_videoFd, VIDIOC_QBUF, &buf))
                {
                        debug_err("xioctl VIDIOC_DQBUF failed,dev=%s,error:%d:%s\n", m_strDevName.c_str(), errno, strerror(errno));
                        break;
                }
        }
        return true;
}

bool CV4l2Cam::initUserp()
{
        struct v4l2_requestbuffers req;
        memset(&req, 0, sizeof(struct v4l2_requestbuffers));

        req.count = V4L2_BUFFER_LENGHT;         // 缓存数量,根据图像占用空间大小申请的缓存区个数
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; // 数据流类型,视频捕获模式.
        req.memory = V4L2_MEMORY_USERPTR;

        // 请求buf
        if (-1 == xioctl(m_videoFd, VIDIOC_REQBUFS, &req))
        {
                debug_err("xioctl VIDIOC_REQBUFS failed,dev=%s,error:%d:%s\n", m_strDevName.c_str(), errno, strerror(errno));
                return false;
        }

        // 分别struct buffer结构
        m_pBuffers = new V4l2Buffer[V4L2_BUFFER_LENGHT];
        if (!m_pBuffers)
        {
                debug_err("calloc  buffer failed,devname=%s,errno: %d, %s\n", m_strDevName.c_str(), errno, strerror(errno));
                return false;
        }
        memset(m_pBuffers, 0, sizeof(V4l2Buffer) * V4L2_BUFFER_LENGHT);

        // 获取请求的buf
        bool bRet = true;
        for (uint32_t i = 0; i < req.count; ++i)
        {
                m_pBuffers[i].length = 2 * m_dwWidth * m_dwHeight;
                m_pBuffers[i].start = new char[m_pBuffers[i].length];
                if (NULL == m_pBuffers[i].start)
                {
                        bRet = false;

                        debug_err("new  failed,devname=%s,errno: %d, %s\n", m_strDevName.c_str(), errno, strerror(errno));
                        break;
                }
        }

        // 失败，释放已映射的内存
        if (bRet == false)
        {
                if (m_pBuffers)
                {
                        for (uint32_t i = 0; i < req.count; i++)
                        {
                                if (m_pBuffers[i].start != NULL)
                                {
                                        free(m_pBuffers[i].start);
                                        m_pBuffers[i].start = NULL;
                                }
                        }
                        delete[] m_pBuffers;
                        m_pBuffers = NULL;
                }
        }
        return bRet;
}

bool CV4l2Cam::initDevice()
{
        int bRet = false;
        switch (m_dwIoType)
        {
        case IO_METHOD_MMAP:
                bRet = initMmap();
                break;
        case IO_METHOD_USERPTR:
                bRet = initUserp();
                break;
#ifdef RB_V4L2_DMA
        case IO_METHOD_DMA:
                bRet = initDma();
                break;
#endif
        default:
                break;
        }
        return bRet;
}

void CV4l2Cam::GrabRoutine()
{
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

        while (m_bRunning)
        {
                if (m_nFirstCapture)
                {
                        m_nFirstCapture = false;
                        usleep(5 * 1000);
                        startCapture();
                        while (m_queueTs.size() > 0)
                        {
                                m_queueTs.pop_front();
                        }
                }
                if (m_bRunning == false)
                {
                        break;
                }
                if (grabImg() == -1)
                {
                        usleep(5 * 1000);
                        continue;
                }
        }
}

bool CV4l2Cam::releaseDevice()
{
        if (m_videoFd != -1)
        {
                close(m_videoFd);
                m_videoFd = -1;
        }

        switch (m_dwIoType)
        {
        case IO_METHOD_MMAP:
                for (int i = 0; i < V4L2_BUFFER_LENGHT; ++i)
                        munmap(m_pBuffers[i].start, m_pBuffers[i].length);
                break;
        case IO_METHOD_USERPTR:
                for (int i = 0; i < V4L2_BUFFER_LENGHT; ++i)
                        free(m_pBuffers[i].start);
                break;
#ifdef RB_V4L2_DMA
        case IO_METHOD_DMA:
                for (unsigned i = 0; i < V4L2_BUFFER_LENGHT; i++)
                {
                        if (m_pNvbuff[i].dmabuff_fd)
                                NvBufSurf::NvDestroy(m_pNvbuff[i].dmabuff_fd);
                }
                free(m_pNvbuff);
                m_pNvbuff = NULL;
                break;
#endif
        default:
                break;
        }

        if (m_pBuffers)
        {
                delete[] m_pBuffers;
                m_pBuffers = NULL;
        }
        return true;
}

#ifdef RB_V4L2_DMA
bool CV4l2Cam::initDma()
{
    bool bRet = true;
    NvBufSurf::NvCommonAllocateParams camparams = {0};
    int fd[V4L2_BUFFER_LENGHT] = {0};

    /* Allocate global buffer context */
    m_pNvbuff = (nv_buffer *)malloc(V4L2_BUFFER_LENGHT * sizeof(nv_buffer));
    if (m_pNvbuff == NULL)
        debug_err("Failed to allocate global buffer context");

    camparams.memType = NVBUF_MEM_SURFACE_ARRAY;
    camparams.width = m_dwWidth;
    camparams.height = m_dwHeight;
    camparams.layout = NVBUF_LAYOUT_PITCH;
    camparams.colorFormat = get_nvbuff_color_fmt(V4L2_PIX_FMT_YUYV);
    camparams.memtag = NvBufSurfaceTag_CAMERA;
    if (NvBufSurf::NvAllocate(&camparams, V4L2_BUFFER_LENGHT, fd))
        debug_err("Failed to create NvBuffer");
    /* Create buffer and provide it with camera */
    for (unsigned int index = 0; index < V4L2_BUFFER_LENGHT; index++)
    {
        NvBufSurface *pSurf = NULL;

        m_pNvbuff[index].dmabuff_fd = fd[index];

        if (-1 == NvBufSurfaceFromFd(fd[index], (void**)(&pSurf)))
            debug_err("Failed to get NvBuffer parameters");

        /* TODO: add multi-planar support
           Currently only supports YUV422 interlaced single-planar */

        if (-1 == NvBufSurfaceMap (pSurf, 0, 0, NVBUF_MAP_READ_WRITE))
        debug_err("Failed to map buffer");
        m_pNvbuff[index].start = (unsigned char *)pSurf->surfaceList[0].mappedAddr.addr[0];
        m_pNvbuff[index].size = pSurf->surfaceList[0].dataSize;
        
    }

    /* Request camera v4l2 buffer */
    struct v4l2_requestbuffers rb;
    memset(&rb, 0, sizeof(rb));
    rb.count = V4L2_BUFFER_LENGHT; //缓存数量,根据图像占用空间大小申请的缓存区个数
    rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; //数据流类型,视频捕获模式.
    rb.memory = V4L2_MEMORY_DMABUF; //DMA使用方式
    //请求buf
    if (ioctl(m_videoFd, VIDIOC_REQBUFS, &rb) < 0)
        debug_err("Failed to request v4l2 buffers: %s (%d)",
                strerror(errno), errno);
    if (rb.count != V4L2_BUFFER_LENGHT)
        debug_err("V4l2 buffer number is not as desired");

    for (unsigned int index = 0; index < V4L2_BUFFER_LENGHT; index++)
    {
        struct v4l2_buffer buf;

        /* Query camera v4l2 buf length */
        memset(&buf, 0, sizeof buf);
        buf.index = index;
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_DMABUF;

        if (ioctl(m_videoFd, VIDIOC_QUERYBUF, &buf) < 0)
            debug_err("Failed to query buff: %s (%d)",
                    strerror(errno), errno);
        

        /* TODO: add support for multi-planer
           Enqueue empty v4l2 buff into camera capture plane */
        buf.m.fd = (unsigned long)m_pNvbuff[index].dmabuff_fd;
        if (buf.length != m_pNvbuff[index].size)
        {
            debug_warn("Camera v4l2 buf length is not expected.\n");
            m_pNvbuff[index].size = buf.length;
        }
    }
    return bRet;
}
#endif

bool CV4l2Cam::initMmap()
{
        struct v4l2_requestbuffers req;

        memset(&req, 0, sizeof(struct v4l2_requestbuffers));
        req.count = V4L2_BUFFER_LENGHT;         // 缓存数量,根据图像占用空间大小申请的缓存区个数
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; // 数据流类型,视频捕获模式.
        req.memory = V4L2_MEMORY_MMAP;          // 内存区的使用方式

        // 请求buf
        if (-1 == xioctl(m_videoFd, VIDIOC_REQBUFS, &req))
        {
                debug_err("xioctl  VIDIOC_REQBUFS failed,devname=%s,errno: %d, %s\n", m_strDevName.c_str(), errno, strerror(errno));
                return false;
        }

        // 分别struct buffer结构
        m_pBuffers = new V4l2Buffer[V4L2_BUFFER_LENGHT];
        if (!m_pBuffers)
        {
                debug_err("calloc  buffer failed,devname=%s,errno: %d, %s\n", m_strDevName.c_str(), errno, strerror(errno));
                return false;
        }
        memset(m_pBuffers, 0, sizeof(V4l2Buffer) * V4L2_BUFFER_LENGHT);

        // 获取请求的buf
        bool bRet = true;
        for (uint32_t i = 0; i < req.count; ++i)
        {
                struct v4l2_buffer buf;
                memset(&buf, 0, sizeof(struct v4l2_buffer));

                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory = V4L2_MEMORY_MMAP;
                buf.index = i;

                if (-1 == xioctl(m_videoFd, VIDIOC_QUERYBUF, &buf))
                {
                        bRet = false;
                        debug_err("xioctl  VIDIOC_QUERYBUF failed,devname=%s,errno: %d, %s\n", m_strDevName.c_str(), errno, strerror(errno));
                        break;
                }

                m_pBuffers[i].length = buf.length;
                m_pBuffers[i].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, m_videoFd, buf.m.offset);
                if (MAP_FAILED == m_pBuffers[i].start)
                {
                        bRet = false;
                        debug_err("mmap  failed,devname=%s,errno: %d, %s\n", m_strDevName.c_str(), errno, strerror(errno));
                        break;
                }
        }

        // 失败，释放已映射的内存
        if (bRet == false)
        {
                if (m_pBuffers)
                {
                        for (uint32_t i = 0; i < req.count; i++)
                        {
                                if (m_pBuffers[i].start != MAP_FAILED && m_pBuffers[i].start != NULL)
                                {
                                        munmap(m_pBuffers[i].start, m_pBuffers[i].length);
                                        m_pBuffers[i].start = NULL;
                                }
                        }
                        delete[] m_pBuffers;
                        m_pBuffers = NULL;
                }
        }

        return bRet;
}

bool CV4l2Cam::setVideoFmt()
{
        struct v4l2_format fmt;
        memset(&fmt, 0, sizeof(fmt));

        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = m_dwWidth;
        fmt.fmt.pix.height = m_dwHeight;
        
        switch (m_dwFormat)
        {
        case 0:
                fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_SRGGB12;
                break;
        case 1:
                fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
                break;
        case 2:
                fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
                break;
        case 3:
                fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_SRGGB10;
                break;
        default:
                fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
                debug_err("not support format:%d, default use YUYV\n", m_dwFormat);
                break;
        }

        fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;

        // 设置当前驱动的频捕获格式
        if (-1 == xioctl(m_videoFd, VIDIOC_S_FMT, &fmt))
        {
                debug_err("xioctl  VIDIOC_S_FMT failed,devname=%s,errno: %d, %s\n", m_strDevName.c_str(), errno, strerror(errno));
                return false;
        }
        return true;
}

bool CV4l2Cam::checkCapabilities()
{
        // 得到视频设备信息
        struct v4l2_capability cap;
        if (-1 == xioctl(m_videoFd, VIDIOC_QUERYCAP, &cap))
        {
                debug_err("xioctl  VIDIOC_QUERYCAP failed,devname=%s,errno: %d, %s\n", m_strDevName.c_str(), errno, strerror(errno));
                return false;
        }

        if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
        {
                debug_err("capabilities not support V4L2_CAP_VIDEO_CAPTURE \n");
                return false;
        }

        if (!(cap.capabilities & V4L2_CAP_STREAMING))
        {
                debug_err("capabilities not support V4L2_CAP_STREAMING \n");
                return false;
        }
        return true;
}

int CV4l2Cam::xioctl(int fh, int request, void *arg)
{
        int r;
        do
        {
                r = ioctl(fh, request, arg);
        } while (-1 == r && EINTR == errno);

        return r;
}

long readOffsetNs()
{
        unsigned long raw_nsec, tsc_ns;
        unsigned long cycles, frq;
        struct timespec tp;

        asm volatile("mrs %0, cntfrq_el0" : "=r"(frq));
        asm volatile("mrs %0, cntvct_el0" : "=r"(cycles));

        clock_gettime(CLOCK_MONOTONIC_RAW, &tp);

        tsc_ns = (cycles * 100 / (frq / 10000)) * 1000;
        raw_nsec = tp.tv_sec * 1000000000 + tp.tv_nsec;
        long offset_ns = llabs(tsc_ns-raw_nsec);

        return offset_ns;
}

int CV4l2Cam::GetOffset()
{
        char tmpStr[128] = {0};
        FILE *pf = fopen("/sys/devices/system/clocksource/clocksource0/offset_ns", "r");
        if (pf)
        {
                fgets(tmpStr, sizeof(tmpStr), pf);
                m_lNsecOffset = atol(tmpStr);
                fclose(pf);
        }
        else
        {
                m_lNsecOffset = readOffsetNs();
                if(m_lNsecOffset < 0)
                {
                        debug_err("read offset failed, m_lNsecOffset = %ld\n", m_lNsecOffset);
                        return -1;
                }
        }
        return 0;
}

void CV4l2Cam::rtcpuToRealtime(timeval rtcpu_time, timespec *real_time)
{
        struct timespec real_sample, monotonic_sample, monotonic_time, time_diff;
        const int64_t NSEC_PER_SEC = 1000000000;

        // printf("[chan:%d]rtcpu_time: %ld.%09ld\n", m_nChan, rtcpu_time.tv_sec, rtcpu_time.tv_usec*1000);
        long long ns = rtcpu_time.tv_sec * NSEC_PER_SEC + rtcpu_time.tv_usec * 1000 - m_lNsecOffset;
        monotonic_time.tv_sec = ns / NSEC_PER_SEC;
        monotonic_time.tv_nsec = ns % NSEC_PER_SEC;
        // printf("[chan:%d] monotonic_time: %ld.%09ld\n", m_nChan,monotonic_time.tv_sec, monotonic_time.tv_nsec);

        clock_gettime(CLOCK_MONOTONIC_RAW, &monotonic_sample);
        clock_gettime(CLOCK_REALTIME, &real_sample);

        time_diff.tv_sec = real_sample.tv_sec - monotonic_sample.tv_sec;
        time_diff.tv_nsec = real_sample.tv_nsec - monotonic_sample.tv_nsec;

        real_time->tv_sec = monotonic_time.tv_sec + time_diff.tv_sec;
        real_time->tv_nsec = monotonic_time.tv_nsec + time_diff.tv_nsec;
        if (real_time->tv_nsec >= NSEC_PER_SEC)
        {
                ++real_time->tv_sec;
                real_time->tv_nsec -= NSEC_PER_SEC;
        }
        else if (real_time->tv_nsec < 0)
        {
                --real_time->tv_sec;
                real_time->tv_nsec += NSEC_PER_SEC;
        }
        return;
}
