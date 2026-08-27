#pragma once

/*流模式*/
enum MUX_STREAM_TYPE
{
        STREAM_TYPE_VIDEO   = (1<<0),                   //视频流
        STREAM_TYPE_AUDIO   = (1<<1),                   //音频流
        STREAM_TYPE_PRIVT   = (1<<2),                   //私有数据
};

/*帧类型*/
enum MUX_FRAME_TYPE
{
        FRAME_TYPE_UNDEF = -1,                          /*定义的帧类型 */
        FRAME_TYPE_PRIVT_FRAME = 0,             /* 私有数据帧 */
        FRAME_TYPE_AUDIO_FRAME =  1,            /* 音频数据帧 */
        FRAME_TYPE_VIDEO_FRAME =  2,             /* 通用视频数据帧 */
        FRAME_TYPE_VIDEO_SPS =  3,               /*视频SPS数据帧 */
        FRAME_TYPE_VIDEO_PPS =  4,                       /*视频PPS数据帧 */
        FRAME_TYPE_VIDEO_IFRAME = 5,            /* 视频数据 I 帧 */
        FRAME_TYPE_VIDEO_EFRAME = 6,             /* 视频数据 E 帧 */
        FRAME_TYPE_VIDEO_PFRAME = 7,             /* 视频数据 P 帧 */
        FRAME_TYPE_VIDEO_BFRAME = 8,            /* 视频数据 B 帧 */
         FRAME_TYPE_VIDEO_VPS = 9,                      /* 视频VPS 帧 */
} ;

/* H264 帧类型 正确 */
enum {
    NALU_TYPE_UNUSE    = 0 ,   /* 未使用 */
    NALU_TYPE_SLICE    = 1 ,   /* 非IDR的片 */
    NALU_TYPE_DPA      = 2 ,   /* 片数据A分区 */
    NALU_TYPE_DPB      = 3 ,   /* 片数据B分区 */
    NALU_TYPE_DPC      = 4 ,   /* 片数据C分区 */
    NALU_TYPE_IDR      = 5 ,   /* IDR图像的片 */
    NALU_TYPE_SEI      = 6 ,   /* 补充增强信息单元（SEI） */
    NALU_TYPE_SPS      = 7 ,   /* 序列参数集(SPS) */
    NALU_TYPE_PPS      = 8 ,   /* 图像参数集(PPS) */
    NALU_TYPE_AUD      = 9 ,   /* 分界符 */
    NALU_TYPE_EOSEQ    = 10,   /* 序列结束 */
    NALU_TYPE_EOSTREAM = 11,   /* 码流结束 */
    NALU_TYPE_FILL     = 12,   /* 填充 */
};

/* h265帧类型 正确 */
enum FRAME_TYPE_H265 {
    HEVC_NAL_TRAIL_N    = 0,
    HEVC_NAL_TRAIL_R    = 1,
    HEVC_NAL_TSA_N      = 2,
    HEVC_NAL_TSA_R      = 3,
    HEVC_NAL_STSA_N     = 4,
    HEVC_NAL_STSA_R     = 5,
    HEVC_NAL_RADL_N     = 6,
    HEVC_NAL_RADL_R     = 7,
    HEVC_NAL_RASL_N     = 8,
    HEVC_NAL_RASL_R     = 9,
    HEVC_NAL_VCL_N10    = 10,
    HEVC_NAL_VCL_R11    = 11,
    HEVC_NAL_VCL_N12    = 12,
    HEVC_NAL_VCL_R13    = 13,
    HEVC_NAL_VCL_N14    = 14,
    HEVC_NAL_VCL_R15    = 15,
    HEVC_NAL_BLA_W_LP   = 16,
    HEVC_NAL_BLA_W_RADL = 17,
    HEVC_NAL_BLA_N_LP   = 18,
    HEVC_NAL_IDR_W_RADL = 19,
    HEVC_NAL_IDR_N_LP   = 20,
    HEVC_NAL_CRA_NUT    = 21,
    HEVC_NAL_IRAP_VCL22 = 22,
    HEVC_NAL_IRAP_VCL23 = 23,
    HEVC_NAL_RSV_VCL24  = 24,
    HEVC_NAL_RSV_VCL25  = 25,
    HEVC_NAL_RSV_VCL26  = 26,
    HEVC_NAL_RSV_VCL27  = 27,
    HEVC_NAL_RSV_VCL28  = 28,
    HEVC_NAL_RSV_VCL29  = 29,
    HEVC_NAL_RSV_VCL30  = 30,
    HEVC_NAL_RSV_VCL31  = 31,
    HEVC_NAL_VPS        = 32,
    HEVC_NAL_SPS        = 33,
    HEVC_NAL_PPS        = 34,
    HEVC_NAL_AUD        = 35,
    HEVC_NAL_EOS_NUT    = 36,
    HEVC_NAL_EOB_NUT    = 37,
    HEVC_NAL_FD_NUT     = 38,
    HEVC_NAL_SEI_PREFIX = 39,
    HEVC_NAL_SEI_SUFFIX = 40,
};


enum MUX_VIDEO_STREAM_TYPE
{
        VIDEO_STREAM_TYPE_UNDEF = 0,
        VIDEO_STREAM_TYPE_H264 = 1,
        VIDEO_STREAM_TYPE_H265 =2,
};

/*h264NAL类型*/
enum MUX_H264_NALU_TYPE
{
        H264_NALU_TYPE_UNKOWN = 0,
        H264_NALU_TYPE_NAL_SLICE=1,
        H264_NALU_TYPE_SLICE_DPA=2,
        H264_NALU_TYPE_SLICE_DPB=3,
        H264_NALU_TYPE_SLICE_DPC =4,
        H264_NALU_TYPE_SLICE_IDR=5,
        H264_NALU_TYPE_SEI=6,
        H264_NALU_TYPE_SPS=7,
        H264_NALU_TYPE_PPS=8,
        H264_NALU_TYPE_AUD=9,
        H264_NALU_TYPE_FILLER=12,
};

enum MUX_H265_NALU_TYPE
{
	H265_NALU_TYPE_VPS=32,
	H265_NALU_TYPE_SPS=33,
	H265_NALU_TYPE_PPS=34,
	H265_NALU_TYPE_SEI=39,
};

/* descriptor 设置宏 */
#define INCLUDE_BASIC_DESCRIPTOR		        (1 << 0)
#define INCLUDE_DEVICE_DESCRIPTOR		      (1 << 1)
#define INCLUDE_VIDEO_DESCRIPTOR		        (1 << 2)
#define INCLUDE_AUDIO_DESCRIPTOR		       (1 << 3)
#define INCLUDE_VIDEO_CLIP_DESCRIPTOR		 (1 << 4)
#define INCLUDE_TIMING_HRD_DESCRIPTOR	       (1 << 5)

typedef struct  tagSTREAM_ABS_TIME
{
    unsigned int    year;                  /* 全局时间年，2008 年输入2008*/
    unsigned int    month;           /* 全局时间月，1 - 12*/
    unsigned int    date;              /* 全局时间日，1 - 31*/
    unsigned int    hour;              /* 全局时间时，0 - 23*/
    unsigned int    minute;          /* 全局时间分，0 - 59*/
    unsigned int    second;          /* 全局时间秒，0 - 59*/
    unsigned int    msecond;     /* 全局时间毫秒，0 - 999*/
} STREAM_ABS_TIME;
