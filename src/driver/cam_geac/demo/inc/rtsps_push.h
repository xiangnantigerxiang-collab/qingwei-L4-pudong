#pragma once
#include "mux_common_type.h"
#include "rtppay.h"
#include "rtsps_server_api.h"
#include "video_encode_api.h"
#include "camera_type.h"
#include "vin_log.h"
#include <arpa/inet.h>
#include <thread>
#include <bits/stdc++.h>

#define STREAM_TYPE_VIDEO 1
#define RTSPS_LISTEN_PORT 554

typedef struct tagProgressContext
{
    int dwChan;
    void *rtppay_handle;
} ProgressContext;



class CVideoPush
{
public:
    CVideoPush(int dwChan, int dwWidth, int dwHeight, int dwFps);
    ~CVideoPush();
    bool Init();
    bool InputData(TImageInfo *pImgInfo);
    friend void CALLBACK videoEncCallBack(unsigned char type, unsigned char *data, int datalen, void *userdata);

public:
    int m_dwHeight;
    int m_dwWidth;
    int m_dwChan;
    int m_dwFps;
    void *m_pEncHandle;
    void *m_pRtpPayHandle;
};
