#pragma once
//定长类型
typedef char s8;
typedef unsigned char u8;
typedef short s16;
typedef unsigned short u16;
typedef int s32;
typedef unsigned int u32;
typedef signed long long s64;
typedef unsigned long    u64;
// 图片定义
typedef struct tagImageInfo
{
        unsigned int dwIndex;          // 图片编号
        unsigned int dwChan;           // 相机通道号
        unsigned int dwTimestampSec;   // utc时间戳 s
        unsigned int dwTimestampNsec;  // utc时间戳 ns
        unsigned int dwDataLength;     // 数据长度
        char szSerialNum[32];          // serial num
        unsigned short dwExposureUsec; // 曝光时间 us
        unsigned short wWidth;         // 图片宽度
        unsigned short wHeight;        // 图片高度
        unsigned short wStatus;        // 状态,参见ImageStatus
        unsigned char byPixelFormat;   // 像素格式
        unsigned char byTimestampType; // 0:GPS,1:CPU
        unsigned char res[30];         // 预留
                                       // unsigned char data[0];
} TImageInfo;

typedef struct tagImageWrap
{
        TImageInfo imgHead;
        unsigned char image[2 * 4096 * 2160];
} TImageWrap;
