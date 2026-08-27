#ifndef __RTSPS_SERVER_API_H__
#define __RTSPS_SERVER_API_H__

#if defined(_WIN32)
#ifndef CALLBACK
#define CALLBACK __stdcall
#endif
#ifdef  RTSPSERVER_EXPORTS
#define RTSPSERVER_API extern "C" __declspec(dllexport)
#else
#define RTSPSERVER_API extern "C" __declspec(dllimport)
#endif
#elif defined (__linux__)
#define RTSPSERVER_API extern "C"
#ifndef CALLBACK
#define CALLBACK 
#endif
#else
#define RTSPSERVER_API
#ifndef CALLBACK
#define CALLBACK 
#endif
#endif

enum RTSP_SERVER_STREAM_TYPE
{
        //视频
         RTSPS_VIDEO_STREAM_TYPE_UNDEF = 0,
         RTSPS_VIDEO_STREAM_TYPE_H264 = 1,
         RTSPS_VIDEO_STREAM_TYPE_H265 =2,

        //音频
         RTSPS_AUDIO_STREAM_TYPE_G711A = 32,
         RTSPS_AUDIO_STREAM_TYPE_AAC = 33,
         RTSPS_AUDIO_STREAM_TYPE_UNDEF = 100,
};

typedef struct tagRtspServerPara
{
    unsigned char   byDbg;                                  //调试标志
    unsigned char  byRes[15];
    char                    szMultiAddr[16];             //组播地址，组播时有效
    char                    szUserName[32];           //用户名
    char                    szPasswd[32];                //密码
    unsigned int     nServerPort;                 //服务端口
     int nMinPort;                                           //通信的最小端口，内部使用比如22000
    int nMaxPort;                                            //通信的最大端口，内部使用比如23000
}TRtspServerPara;

typedef struct tagRtspServerStreamInfo
{
    unsigned char  byVideoPayload;                      //视频负载值
    unsigned char  byAudioPayload;                      //音频负载值
    unsigned int     dwVideoSSrc;                           //视频ssrc
    unsigned int     dwAudioSSrc;                           //音频ssrc
    unsigned int     dwAudioFormat;                       //音频格式 ，视频格式，RTSP_SERVER_STREAM_TYPE    
    unsigned int     dwVideoFormat;                       //视频格式，RTSP_SERVER_STREAM_TYPE    
    unsigned int     res[6];                                            //预留
}TRtspServerStreamInfo;

//nSessionId:会话id
//nEvent:事件类型
//pEventData:事件数据
//nLen:事件数据长度
typedef void (*pRtspServerEventCallBack)(int nSessionId,int nEvent,void *pEventData,int nLen,void *pUserData);

/*************************************************
函数名称 : RTSPServer_Init
函数功能 : 初始化
输入参数说明 : nDbgLevel:打印信息开关;nMiniPort:TCP/UDP会话的最小端口;nMaxPort:TCP/UDP会话的最大端口
函数返回值的说明 : 成功返回0，失败返回-1
*************************************************/
RTSPSERVER_API int RTSPServer_Init(TRtspServerPara *pPara);

/*************************************************
函数名称 : RTSPServer_Release
函数功能 : 结束化
输入参数说明 : 无
函数返回值的说明 : 成功返回0，失败返回-1
*************************************************/
RTSPSERVER_API int RTSPServer_Release();

/*************************************************
函数名称 : RTSPServer_SetStreamInfo
函数功能 :  设置流信息
输入参数说明 : nDbgLevel:打印信息开关;nMiniPort:TCP/UDP会话的最小端口;nMaxPort:TCP/UDP会话的最大端口
函数返回值的说明 : 成功返回0，失败返回-1
*************************************************/
RTSPSERVER_API int RTSPServer_SetStreamInfo(int nChan,int nStreamType, TRtspServerStreamInfo *pPara);

/*************************************************
函数名称 : RTSPServer_Start
函数功能 : 开启服务器
函数返回值的说明 : 成功返回0，失败返回-1
*************************************************/
RTSPSERVER_API int RTSPServer_Start();

/*************************************************
函数名称 : RTSPServer_Stop
函数功能 : 停止服务器
输入参数说明 : 
函数返回值的说明 : 成功返回0，失败返回-1
*************************************************/
RTSPSERVER_API int RTSPServer_Stop();

/*************************************************
函数名称 : RTSPServer_SendRealSteam
函数功能 : 发送实时流，数据内容为4字节标准interleaved+rtp数据包
输入参数说明 : 
nChan：通道号
nStreamType：流类型，0：主码流，1：子码流
pData:数据
nLen：数据长度
函数返回值的说明 :  成功返回0，失败返回-1
说明：该函数不会阻塞
*************************************************/
RTSPSERVER_API int RTSPServer_SendRealSteam(int nChan,int nStreamType,void *pData,int nLen);


/*************************************************
函数名称 : RTSPServer_SendFileStream
函数功能 : 发送文件流，数据内容为4字节标准interleaved+rtp数据包
输入参数说明 : 
nSessionId：请求的会话id
pData:数据
nLen：数据长度
函数返回值的说明 : 成功返回0，失败返回-1
说明：该函数不会阻塞
*************************************************/
RTSPSERVER_API int RTSPServer_SendFileStream(int nSessionId,void *pData,int nLen);
/*************************************************
函数名称 : RTSPServer_SetEventCallBack
函数功能 : 设置事件回调函数
输入参数说明 : 
函数返回值的说明 : 成功返回0，失败返回-1
*************************************************/
RTSPSERVER_API int RTSPServer_SetEventCallBack(pRtspServerEventCallBack pCallBack,void *pUserData);
#endif //__RTSPS_SERVER_API_H__

