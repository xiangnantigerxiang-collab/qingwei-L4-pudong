#pragma once
#include "mux_common_type.h"


#if defined(_WIN32) || defined(_WIN64)
#ifndef CALLBACK
#define CALLBACK __stdcall
#endif
#ifdef  RTPPAY_EXPORTS
#define RTPPAY_API extern "C" __declspec(dllexport)
#else
#define RTPPAY_API extern "C" __declspec(dllimport)
#endif
#elif defined (__linux__)
#define RTPPAY_API extern "C"
#ifndef CALLBACK
#define CALLBACK 
#endif
#else 
#define RTPPAY_API
#ifndef CALLBACK
#define CALLBACK 
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef  struct tagRTPPAY_PARAM
{
        unsigned char    byStreamType;                          //流类型
        unsigned char    byVideoPayloadType;              //视频负载类型
        unsigned char    byAudioPayloadType;             //音频负载类型
        unsigned char    byVideoStreamType;              //视频类型,VIDEO_STREAM_TYPE
        unsigned char    byAudioStreamType;              //音频类型
        unsigned int     dwAudioSsrc;
        unsigned int     dwVideoSsrc;
        unsigned int     dwPrivateSsrc;
        unsigned int     dwMtu;                          //mtu大小
        unsigned char    byVideoFps;                     //视频帧率
        unsigned char    byAudioFps;                     //音频帧率
        unsigned char    byPrint;                       
}RTPPAY_PARAM;

typedef  struct tagRTPPAY_PROCESS_PARAM
{
        unsigned char *pInData;                                   //输入数据
        unsigned int       dwInDataLen;                       //输入数据长度 
        unsigned char    byStreamType;                   //流类型，STREAM_TYPE
        unsigned int       dwTimeStampSec;              //秒部分
        unsigned int       dwTimeStampUsec;           //毫秒部分

        /*  输出缓冲区pOutData内数据结构如下图，size为网络序
        0 1 2 3 4 5 6 7 ... 0 1 2 3 4 5 6 7 ... 0 1 2 3 4 5 6 7 ...
        +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
        |           PACK  0       |  ...............  |           PACK n       | 
        +-------------------+-------------------+-------------------+
        |  size |  data........|  ...............  |  size |  data....... | 
        +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ */    
        unsigned char *pOutData;                                //输出数据
        unsigned int       dwOutDataLen;                    //输出数据长度
        unsigned int       dwPacketNum;                    //输出的rtp包个数

}RTPPAY_PROCESS_PARAM;

RTPPAY_API int CALLBACK RTPPAY_Init();
RTPPAY_API int CALLBACK RTPPAY_Release();
RTPPAY_API void * CALLBACK RTPPAY_CreateHandle(RTPPAY_PARAM *param);
RTPPAY_API int CALLBACK RTPPAY_Process(void *handle,RTPPAY_PROCESS_PARAM *param);
RTPPAY_API int CALLBACK RTPPAY_Reset(void *handle);
RTPPAY_API int CALLBACK RTPPAY_DestroyHandle(void *handle);

#ifdef __cplusplus
}
#endif
