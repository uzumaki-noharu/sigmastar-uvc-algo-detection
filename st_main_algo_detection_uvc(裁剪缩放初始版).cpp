/* Copyright (c) 2018-2019 Sigmastar Technology Corp.
 All rights reserved.

  Unless otherwise stipulated in writing, any and all information contained
 herein regardless in any format shall remain the sole proprietary of
 Sigmastar Technology Corp. and be kept in strict confidence
 (��Sigmastar Confidential Information��) by the recipient.
 Any unauthorized act including without limitation unauthorized disclosure,
 copying, use, reproduction, sale, distribution, modification, disassembling,
 reverse engineering and compiling of the contents of Sigmastar Confidential
 Information is unlawful and strictly prohibited. Sigmastar hereby reserves the
 rights to any and all damages, losses, costs and expenses resulting therefrom.
*/
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/syscall.h>
#include <string>
#include <poll.h>
#include <vector>
#include <math.h>

#include <iostream>

#include "st_common.h"
#include "st_vif.h"
#include "st_vpe.h"
#include "st_venc.h"
#include "st_uvc.h"
#include "st_cus3a.h"
#include "st_rgn.h"

#include "BasicUsageEnvironment.hh"
#include "liveMedia.hh"
#include "Live555RTSPServer.hh"
#include "dictionary.h"
#include "iniparser.h"

#include "mi_rgn.h"
#include "mi_divp.h"
#include "mi_ipu.h"
#include "mi_divp_datatype.h"

#include "mi_od.h"
#include "mi_md.h"

#include "mi_vdf.h"
#include "mi_ao.h"
#include "mi_aio_datatype.h"
#include "mi_isp.h"
#include "mi_iqserver.h"
#include "mi_sensor.h"
#include "mi_sensor_datatype.h"

#include "sstar_detection_api.h"

#include "linux_list.h"
#include "isp_cus3a_if.h"
#include "mi_eptz.h"
#include "mi_ldc.h"


#ifdef CONFIG_SIGMASTAR_CHIP_I6E

#include "mi_eptz.h"
#include "mi_ldc.h"

#endif

#include "algor.h"
#include "agc_core.h"
#include "ns_core.h"
#include "signal_processing_library.h"
#define XF_AGC_SAT2(a, b, c) (a > b ? (short)((float)a*0.422188+18934) : (a < c ? (short)((float)a*0.422188-18934 ) : a*2))
#define XF_AGC_SAT1_5(a, b, c) (a > b ? (short)((float)a*0.5+16000) : (a < c ? (short)((float)a*0.5-16000 ) : a*1.5))
#define AI_VOLUME_AMIC_PreSet 15
#define FS 48000
#define LEN1024_PACKET (1024)

#define MODE 1

extern const float para_a[5];
extern const short para_b[5];
extern const short para_c[5];
void *Ns_Inst = NULL;   //NS
// void *Ns_Inst2 = NULL; 
void *Agc_Inst = NULL;  //AGC	
// void *Agc_Inst2 = NULL;  //AGC	
short agcpara[9] = { 16,3,3,110,0,1024,0,2500,6 };
float nspara[2] = { 1.25f,0.09f};
int  Analysis_state11[6];
int  Analysis_state12[6];
int  Synthesis_state11[6];
int  Synthesis_state12[6];
int  Analysis_state21[6];
int  Analysis_state22[6];
int  Synthesis_state21[6];
int  Synthesis_state22[6];

int micLevelOut = 0;
int inMicLevel = 0;
int outMicLevel = 0;
unsigned char saturationWarning;


#define DIVP_INPUT_WIDTH        1920
#define DIVP_INPUT_HEIGHT        1080

#define MODEL_WIDTH    800   //640   
#define MODEL_HEIGHT   480   //352 
#define DISP_WIDTH    1920
#define DISP_HEIGHT   1080



#define RTSP_LISTEN_PORT        554
#define USB_CAMERA0_INDEX          0
#define USB_CAMERA1_INDEX          1
#define USB_CAMERA2_INDEX          2
#define USB_CAMERA3_INDEX          3
#define UVC_STREAM0                "uvc_stream0"
#define UVC_STREAM1                "uvc_stream1"
#define UVC_STREAM2                "uvc_stream2"
#define UVC_STREAM3                "uvc_stream3"
#define MAX_UVC_DEV_NUM             4
#define PATH_PREFIX                "/mnt"
#define DEBUG_ES_FILE            0

#define RGN_OSD_TIME_START        8
#define RGN_OSD_MAX_NUM         4
#define RGN_OSD_TIME_WIDTH        180
#define RGN_OSD_TIME_HEIGHT        32

#define DOT_FONT_FILE            "/customer/mi_demo/gb2312.hzk"

#define MAX_CAPTURE_NUM            4
#define CAPTURE_PATH            "/mnt/capture"

#define MI_AUDIO_SAMPLE_PER_FRAME 768
#define AO_INPUT_FILE            "8K_16bit_STERO_30s.wav"
#define AO_OUTPUT_FILE            "./tmp.pcm"

#define DIVP_CHN_FOR_OSD        0
#define DIVP_CHN_FOR_DLA        1
#define DIVP_CHN_FOR_VDF        2
#define DIVP_CHN_FOR_ROTATION    3
#define DIVP_CHN_FOR_SCALE        3
#define VENC_CHN_FOR_CAPTURE    12

#define USE_STREAM_FILE 0
#define DIVP_CHN_FOR_RESOLUTION 0
#define DIVP_SCALE_IPNUT_FILE    "vpe0_port0_1920x1088_0000.yuv"

#define MAX_CHN_NEED_OSD        4

#define RAW_W                     384
#define RAW_H                     288
#define PANEL_DIVP_ROTATE        0

#define RGB_TO_CRYCB(r, g, b)                                                            \
        (((unsigned int)(( 0.439f * (r) - 0.368f * (g) - 0.071f * (b)) + 128.0f)) << 16) |    \
        (((unsigned int)(( 0.257f * (r) + 0.564f * (g) + 0.098f * (b)) + 16.0f)) << 8) |        \
        (((unsigned int)((-0.148f * (r) - 0.291f * (g) + 0.439f * (b)) + 128.0f)))

#define IQ_FILE_PATH    "/customer/iq_api.bin"
#define IQ_HDR_FILE_PATH    "/customer/hdr.bin"


#define MAX_RGN_COVER_W               8192
#define MAX_RGN_COVER_H               8192

#define RGN_OSD_HANDLE                    0
#define RGN_FOR_VDF_BEGIN                12

#define MAX_FULL_RGN_NULL                3

#ifdef ALIGN_UP
#undef ALIGN_UP
#define ALIGN_UP(x, align) (((x) + ((align) - 1)) & ~((align) - 1))
#else
#define ALIGN_UP(x, align) (((x) + ((align) - 1)) & ~((align) - 1))
#endif
#ifndef ALIGN_DOWN
#define ALIGN_DOWN(val, alignment) (((val)/(alignment))*(alignment))
#endif
struct ST_Stream_Attr_T
{
    MI_BOOL        bEnable;
    ST_Sys_Input_E enInput;
    MI_U32     u32InputChn;
    MI_U32     u32InputPort;
    MI_S32     vencChn;
    MI_VENC_ModType_e eType;
    float      f32Mbps;
    MI_U32     u32Width;
    MI_U32     u32Height;

    MI_U32 enFunc;
    const char    *pszStreamName;
    MI_SYS_BindType_e eBindType;
    MI_U32 u32BindPara;

    MI_BOOL bForceIdr;
    MI_U32 divpChn;
    MI_BOOL bUserVenc;
};

typedef struct
{
    MI_S32 s32UseOnvif;     //0:not use, else use
    MI_S32 s32UseVdf;         // 0: not use, else use
    MI_S32 s32LoadIQ;        // 0: not load, else load IQ file
    MI_S32 s32HDRtype;
    ST_Sensor_Type_T enSensorType;
    MI_SYS_Rotate_e enRotation;
    MI_VPE_3DNR_Level_e en3dNrLevel;
    MI_U8 u8SnrPad;
    MI_S8 s8SnrResIndex;
} ST_Config_S;

typedef struct {
    MI_U32  fcc;
    MI_U32  u32Width;
    MI_U32  u32Height;
    MI_U32  u32FrameRate;
    MI_SYS_ChnPort_t dstChnPort;
} ST_UvcSetting_Attr_T;

typedef struct VENC_STREAMS_s {
    bool used;
    MI_VENC_Stream_t stStream;
} VENC_STREAMS_t;

typedef struct {
    VENC_STREAMS_t *pstuserptr_stream;
} ST_Uvc_Resource_t;

typedef struct {
    char name[20];
    int dev_index;
    ST_UVC_Handle_h handle;
    ST_UvcSetting_Attr_T setting;
    ST_Uvc_Resource_t res;
} ST_UvcDev_t;

typedef struct {
    int devnum;
    ST_UvcDev_t dev[];
} ST_UvcSrc_t;

static MI_BOOL g_bEnableVideo = TRUE;
static MI_BOOL g_bExit = FALSE;
static ST_Config_S g_stConfig;
static ST_UvcSrc_t * g_UvcSrc;
static MI_U8 g_bitrate[MAX_UVC_DEV_NUM] = {0, 0, 0, 0};
static MI_U8 g_qfactor[MAX_UVC_DEV_NUM] = {0, 0, 0, 0};
static MI_U8 g_maxbuf_cnt = 3;
static MI_U8 g_enable_iqserver = 0;
static MI_U8 g_load_iq_bin = 0;
static MI_U32 g_device_num = 1;
static char g_IspBinPath[64] = {0};
static MI_BOOL g_bStartCapture = FALSE;

static char gaLdcBinPath[64] = {0};
static MI_BOOL gbLdcEnable = FALSE;
static MI_BOOL gEnableStTest = FALSE;
static MI_BOOL gEnableHdr = FALSE;
static MI_BOOL gbSnrPowerOnWhenPreview = FALSE;

//dump data before uvc xfer for debug
static char gaDumpFilePath[64] = {0};
static MI_BOOL gbDumpData = FALSE;
static FILE *gpDumpFile = NULL;

static struct ST_Stream_Attr_T g_stStreamAttr[] =
{
    [USB_CAMERA0_INDEX] =
    {
        .bEnable = FALSE,
        .enInput = ST_Sys_Input_VPE,
        .u32InputChn = 0,
        .u32InputPort = 0,
        .vencChn = 0,
        .eType = E_MI_VENC_MODTYPE_H264E,
        .f32Mbps = 2.0,
        .u32Width = 1920,
        .u32Height = 1080,
        .enFunc = ST_Sys_Func_UVC,
        .pszStreamName = UVC_STREAM0,
        .eBindType = E_MI_SYS_BIND_TYPE_REALTIME,
        .u32BindPara = 0,
        .bForceIdr = 0,
    },
    [USB_CAMERA1_INDEX] =
    {
        .bEnable = FALSE,
        .enInput = ST_Sys_Input_VPE,
        .u32InputChn = 0,
        .u32InputPort = 2,
        .vencChn = 1,
        .eType = E_MI_VENC_MODTYPE_JPEGE,
        .f32Mbps = 2.0,
        .u32Width = 1920,
        .u32Height = 1080,
        .enFunc = ST_Sys_Func_UVC,
        .pszStreamName = UVC_STREAM1,
        .eBindType = E_MI_SYS_BIND_TYPE_FRAME_BASE,
        .u32BindPara = 0,
        .bForceIdr = 0,
    },
    [USB_CAMERA2_INDEX] =
    {
        .bEnable = FALSE,
        .enInput = ST_Sys_Input_DIVP,
        .u32InputChn = 0,
        .u32InputPort = 1,
        .vencChn = 2,
        .eType = E_MI_VENC_MODTYPE_H264E,
        .f32Mbps = 2.0,
        .u32Width = 1920,
        .u32Height = 1080,
        .enFunc = ST_Sys_Func_UVC,
        .pszStreamName = UVC_STREAM2,
        .eBindType = E_MI_SYS_BIND_TYPE_FRAME_BASE,//E_MI_SYS_BIND_TYPE_HW_RING,
        .u32BindPara = 0,

        .bForceIdr = 0,
        .divpChn = 0,
    },
    [USB_CAMERA3_INDEX] =
    {
        .bEnable = FALSE,
        .enInput = ST_Sys_Input_DIVP,
        .u32InputChn = 0,
        .u32InputPort = 1,
        .vencChn = 3,
        .eType = E_MI_VENC_MODTYPE_H264E,
        .f32Mbps = 2.0,
        .u32Width = 1920,
        .u32Height = 1080,
        .enFunc = ST_Sys_Func_UVC,
        .pszStreamName = UVC_STREAM3,
        .eBindType = E_MI_SYS_BIND_TYPE_FRAME_BASE,//E_MI_SYS_BIND_TYPE_HW_RING,
        .u32BindPara = 0,

        .bForceIdr = 0,
        .divpChn = 1,
    },
};

/* audio related */
#if defined(__cplusplus) || defined(c_plusplus)
extern "C"{
#endif
#include <cerrno>
#include <stdlib.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>

#include "mi_sys_datatype.h"
#include "mi_sys.h"
#include "mi_ao.h"
#include "mi_ai.h"
#include "st_common.h"
#include "st_uac.h"
#include "pcm.h"
#include <sound/asound.h>
#include "mixer.h"
#if defined(__cplusplus) || defined(c_plusplus)
}
#endif

#define AI_DEV_ID_MAX   (4)
#define AO_DEV_ID_MAX   (1)

#define AI_DEV_AMIC     (0)
#define AI_DEV_DMIC     (1)
#define AI_DEV_I2S_RX   (2)
#define AI_DEV_LineIn   (3)
#define AI_DEV_I2S_RX_AND_SRC   (4)

#define AO_DEV_LineOut  (0)
#define AO_DEV_I2S_TX   (1)

#define AI_VOLUME_AMIC_MIN      (0)
#define AI_VOLUME_AMIC_MAX      (21)
#define AI_VOLUME_DMIC_MIN      (-60)
#define AI_VOLUME_DMIC_MAX      (30)
#define AI_VOLUME_LINEIN_MIN    (0)
#define AI_VOLUME_LINEIN_MAX    (7)

typedef struct {
    MI_BOOL bComfortNoiseEnable;
    MI_S16 s16DelaySample;
    MI_U32 u32AecSupfreq[6];
    MI_U32 u32AecSupIntensity[7];
    MI_S32 s32Reserved;
} ST_AI_AecConfig_t;

typedef struct {
    MI_AUDIO_AlgorithmMode_e eMode;
    MI_U32 u32NrIntensityBand[6];
    MI_U32   u32NrIntensity[7];
    MI_U32   u32NrSmoothLevel;
    MI_AUDIO_NrSpeed_e eNrSpeed;
}ST_AUDIO_AnrConfig_t;

MI_BOOL bEnableAI = FALSE;
MI_BOOL bEnableAO= FALSE;
MI_BOOL bAiEnableVqe = FALSE;
MI_BOOL bAiEnableNr = FALSE;
MI_BOOL  bAiEnableAec = FALSE;
MI_BOOL bAoEnableVqe = FALSE;
MI_BOOL bAoEnableNr = FALSE;
MI_BOOL bEnableMjpegRealtimeMode = FALSE; //just for 1 mjpeg video streaming


MI_U32 u32AiSampleRate = E_MI_AUDIO_SAMPLE_RATE_48000;
MI_U32 u32AoSampleRate = E_MI_AUDIO_SAMPLE_RATE_48000;
MI_S32 AiVolume;
MI_AUDIO_DEV AoDevId = AO_DEV_LineOut;
MI_AO_CHN AoChn;
MI_AUDIO_DEV AiDevId = AI_DEV_AMIC;
MI_AI_CHN AiChn;

ST_UAC_Handle_h p_handle;
ST_UAC_Config_t uac_playback_config;
ST_UAC_Handle_h c_handle;
ST_UAC_Config_t uac_capture_config;

pthread_t Aotid;
pthread_t Aitid;
pthread_t Voltid;

MI_AI_AecConfig_t stAecCfg = {
    .bComfortNoiseEnable = FALSE,
    .s16DelaySample = 0,
    .u32AecSupfreq = {4, 6, 36, 49, 50, 51},
    .u32AecSupIntensity = {5, 4, 4, 5, 10, 10, 10},
};

MI_AUDIO_AnrConfig_t stAnrCfg = {
    .eMode = E_MI_AUDIO_ALGORITHM_MODE_MUSIC,
    .u32NrIntensity = 15,
    .u32NrSmoothLevel = 10,
    .eNrSpeed = E_MI_AUDIO_NR_SPEED_MID,
};

#ifdef AUDIO_ENABLE

/* param's check is not complete */
MI_BOOL checkAudioParam(void)
{
    MI_BOOL bCheckPass = FALSE;

    do {
        if(bEnableAI)
        {
            if(AiDevId < 0 || AiDevId > AI_DEV_ID_MAX)
            {
                printf("Invalid AiDevId!!!\n");
                break;
            }

            if(
                    (E_MI_AUDIO_SAMPLE_RATE_8000 != u32AiSampleRate)
                &&  (E_MI_AUDIO_SAMPLE_RATE_16000 != u32AiSampleRate)
                &&  (E_MI_AUDIO_SAMPLE_RATE_32000 != u32AiSampleRate)
                &&  (E_MI_AUDIO_SAMPLE_RATE_48000 != u32AiSampleRate)
            )
            {
                printf("Invalid Ai Sample Rate!!!\n");
                break;
            }

            if(bAiEnableAec)
            {
                if((E_MI_AUDIO_SAMPLE_RATE_8000 != u32AiSampleRate)
                    &&  (E_MI_AUDIO_SAMPLE_RATE_16000 != u32AiSampleRate))
                {
                    printf("Aec only support 8K/16K!!!\n");
                    break;
                }

                if(AI_DEV_I2S_RX == AiDevId)
                {
                    printf("I2S RX not support AEC!!!\n");
                    break;
                }
            }

            if(bAiEnableVqe)
            {
                if(
                        (E_MI_AUDIO_SAMPLE_RATE_8000 != u32AiSampleRate)
                    &&  (E_MI_AUDIO_SAMPLE_RATE_16000 != u32AiSampleRate)
                    &&  (E_MI_AUDIO_SAMPLE_RATE_48000 != u32AiSampleRate)
                )
                {
                    printf("Vqe only support 8K/16/48K!!!\n");
                    break;
                }
            }
        }

        if(bEnableAO)
        {
            if((AoDevId < 0) || (AoDevId > AO_DEV_ID_MAX))
            {
                printf("Invalid AoDevId!!!\n");
                break;
            }

            if(
                    (E_MI_AUDIO_SAMPLE_RATE_8000 != u32AoSampleRate)
                &&  (E_MI_AUDIO_SAMPLE_RATE_16000 != u32AoSampleRate)
                &&  (E_MI_AUDIO_SAMPLE_RATE_32000 != u32AoSampleRate)
                &&  (E_MI_AUDIO_SAMPLE_RATE_48000 != u32AoSampleRate)
            )
            {
                printf("Invalid Ao Sample Rate!!!\n");
                break;
            }
        }

        bCheckPass = TRUE;
    }while(0);

    return bCheckPass;
}

MI_S32 ST_AO_Init(void)
{
    MI_S32 ret;
    MI_AUDIO_Attr_t stAttr;

    stAttr.eBitwidth = E_MI_AUDIO_BIT_WIDTH_16;
    stAttr.eWorkmode = E_MI_AUDIO_MODE_I2S_MASTER;
    stAttr.WorkModeSetting.stI2sConfig.bSyncClock = TRUE;
    stAttr.WorkModeSetting.stI2sConfig.eFmt = E_MI_AUDIO_I2S_FMT_I2S_MSB;
    stAttr.WorkModeSetting.stI2sConfig.eMclk = E_MI_AUDIO_I2S_MCLK_0;
    stAttr.WorkModeSetting.stI2sConfig.u32TdmSlots = 8;
    stAttr.WorkModeSetting.stI2sConfig.eI2sBitWidth = E_MI_AUDIO_BIT_WIDTH_16;
    stAttr.u32PtNumPerFrm = 1024;
    stAttr.u32ChnCnt = 2;   //u32ChnCnt=1 in master branch, u32ChnCnt=2 in stable branch
    stAttr.eSoundmode = E_MI_AUDIO_SOUND_MODE_STEREO;
    stAttr.eSamplerate = (MI_AUDIO_SampleRate_e)u32AoSampleRate;

    /* set ao public attr */
    ret = MI_AO_SetPubAttr(AoDevId, &stAttr);
    if(MI_SUCCESS != ret)
    {
        printf("set ao %d attr err:0x%x\n", AoDevId, ret);
        return ret;
    }

    /* enable ao device */
    ret = MI_AO_Enable(AoDevId);
    if(MI_SUCCESS != ret)
    {
        printf("enable ao dev %d err:0x%x\n", AoDevId, ret);
        return ret;
    }

    ret = MI_AO_EnableChn(AoDevId, AoChn);
    if(MI_SUCCESS != ret)
    {
        printf("enable ao dev %d chn %d err:0x%x\n", AoDevId, AoChn, ret);
        return ret;
    }

    if(bAoEnableVqe)
    {
        MI_AO_VqeConfig_t stAoVqeConfig;
        stAoVqeConfig.bAgcOpen = FALSE;
        stAoVqeConfig.bAnrOpen = bAoEnableNr;
        stAoVqeConfig.bEqOpen = FALSE;
        stAoVqeConfig.bHpfOpen = FALSE;
        stAoVqeConfig.s32FrameSample = 128;
        stAoVqeConfig.s32WorkSampleRate = (MI_AUDIO_SampleRate_e)u32AoSampleRate;
        memcpy(&stAoVqeConfig.stAnrCfg, &stAnrCfg, sizeof(MI_AUDIO_AnrConfig_t));

        ret = MI_AO_SetVqeAttr(AoDevId, AoChn, &stAoVqeConfig);
        if(MI_SUCCESS != ret)
        {
            printf("MI_AO_SetVqeAttr failed\n");
            return ret;
        }

        ret = MI_AO_EnableVqe(AoDevId, AoChn);
        if(MI_SUCCESS != ret)
        {
            printf("MI_AO_EnableVqe failed\n");
            return ret;
        }
    }

    return MI_SUCCESS;
}

MI_S32 ST_AO_Deinit(void)
{
     /* disable ao device */
     MI_S32 ret;

    if(bAoEnableVqe)
    {
        ret = MI_AO_DisableVqe(AoDevId, AoChn);
        if(MI_SUCCESS != ret)
        {
            printf("MI_AO_DisableVqe failed\n");
            return ret;
        }
    }

    ret = MI_AO_DisableChn(AoDevId, AoChn);
    if(MI_SUCCESS != ret)
    {
        printf("MI_AO_DisableChn failed\n");
        return ret;
    }

    ret = MI_AO_Disable(AoDevId);
    if(MI_SUCCESS != ret)
    {
        printf("disable ao dev %d err:0x%x\n", AoDevId, ret);
        return ret;
    }

    return MI_SUCCESS;
}

void* aoSendFrame(void* data)
{
    MI_S32 ret;
    ST_UAC_Frame_t stFrame ={};
    stFrame.length = ST_UAC_GetPcm_BufSize(p_handle);
    stFrame.data = malloc(stFrame.length);

    while(!g_bExit)
    {
        memset(stFrame.data, 0, ST_UAC_GetPcm_BufSize(p_handle));

        ret = ST_UAC_GetFrame(p_handle, &stFrame);
        if(MI_SUCCESS != ret)
        {
            if(EBUSY != errno)
                printf("ST_UAC_GetFrame failed:%s %d\n",  strerror(errno), errno);

            usleep(1000);
            continue;
        }

        MI_AUDIO_Frame_t stAoSendFrame;
        stAoSendFrame.u32Len = stFrame.length;
        stAoSendFrame.apVirAddr[0] = stFrame.data;
        stAoSendFrame.apVirAddr[1] = NULL;

        do {
            ret = MI_AO_SendFrame(AoDevId, AoChn, &stAoSendFrame, -1);
        }while(MI_AO_ERR_NOBUF == ret);

        if(MI_SUCCESS != ret)
        {
            printf("MI_AO_SendFrame failed!\n");
            break;
        }
    }

    free(stFrame.data);
    ret = 0;
    pthread_exit(&ret);
}

MI_S32 ST_AI_Init(void)
{
    MI_S32 ret;
    MI_AUDIO_Attr_t stAttr;
    MI_SYS_ChnPort_t stAiChnOutputPort;

    stAttr.eBitwidth = E_MI_AUDIO_BIT_WIDTH_16;
    stAttr.eSamplerate = (MI_AUDIO_SampleRate_e)u32AiSampleRate;
    stAttr.eSoundmode = E_MI_AUDIO_SOUND_MODE_STEREO;
    stAttr.eWorkmode = E_MI_AUDIO_MODE_I2S_MASTER;
    stAttr.u32ChnCnt = 1;
    stAttr.u32CodecChnCnt = 0; // useless
    stAttr.u32FrmNum = 16;
    stAttr.u32PtNumPerFrm = 1024;
    stAttr.WorkModeSetting.stI2sConfig.bSyncClock = TRUE;
    stAttr.WorkModeSetting.stI2sConfig.eFmt = E_MI_AUDIO_I2S_FMT_I2S_MSB;
    stAttr.WorkModeSetting.stI2sConfig.eMclk = E_MI_AUDIO_I2S_MCLK_0;
    stAttr.WorkModeSetting.stI2sConfig.u32TdmSlots = 8;
    stAttr.WorkModeSetting.stI2sConfig.eI2sBitWidth = E_MI_AUDIO_BIT_WIDTH_16;

    /* set public attribute of AI device */
    printf("Test point_AIinit\n\n");

    Ns_Inst = NsParaInit(nspara, FS);
    //Ns_Inst2 = NsParaInit(nspara, FS);
    Agc_Inst = AgcParaInit(agcpara, FS);
    //Agc_Inst2 = AgcParaInit(agcpara, FS);    
    ret = MI_AI_SetPubAttr(AiDevId, &stAttr);
    if(MI_SUCCESS != ret)
    {
        printf("set ai %d attr err:0x%x\n", AiDevId, ret);
        return ret;
    }

    /* enable AI device */
    ret = MI_AI_Enable(AiDevId);
    if(MI_SUCCESS != ret)
    {
        printf("enable ai %d err:0x%x\n", AiDevId, ret);
        return ret;
    }

    /* set buffer depth */
    stAiChnOutputPort.eModId = E_MI_MODULE_ID_AI;
    stAiChnOutputPort.u32DevId = AiDevId;
    stAiChnOutputPort.u32ChnId = AiChn;
    stAiChnOutputPort.u32PortId = 0;
    MI_SYS_SetChnOutputPortDepth(&stAiChnOutputPort, 4, 8);

    ret = MI_AI_SetVqeVolume(AI_DEV_AMIC, AiChn, AI_VOLUME_AMIC_PreSet);
    printf("Amicvolume is %d\n", AI_VOLUME_AMIC_PreSet);

    /* MI_AI_SetVqeVolume */
    switch(AiDevId)
    {
        case AI_DEV_AMIC:
            ret = MI_AI_SetVqeVolume(AiDevId, AiChn, AI_VOLUME_AMIC_PreSet);
            if(MI_SUCCESS != ret)
            {
                printf("MI_AI_SetVqeVolume failed\n");
                return ret;
            }
            break;

         case AI_DEV_DMIC:
            ret = MI_AI_SetVqeVolume(AiDevId, AiChn, AI_VOLUME_AMIC_PreSet);
            if(MI_SUCCESS != ret)
            {
                printf("MI_AI_SetVqeVolume failed\n");
                return ret;
            }
            break;

        case AI_DEV_LineIn:
            //ret = MI_AI_SetVqeVolume(AiDevId, AiChn, AI_VOLUME_LINEIN_MAX);
            ret = MI_AI_SetVqeVolume(AiDevId, AiChn, AI_VOLUME_AMIC_PreSet);
            if(MI_SUCCESS != ret)
            {
                printf("MI_AI_SetVqeVolume failed\n");
                return ret;
            }
            break;

        default:
            break;
    }

    /* enable AI Channel */
    ret = MI_AI_EnableChn(AiDevId, AiChn);
    if(MI_SUCCESS != ret)
    {
        printf("enable Dev%d Chn%d err:0x%x\n", AiDevId, AiChn, ret);
        return ret;
    }

    if(bAiEnableVqe)
    {
        MI_AI_VqeConfig_t   stAiVqeConfig;
        stAiVqeConfig.u32ChnNum = 1;
        stAiVqeConfig.bAecOpen = bAiEnableAec;
        stAiVqeConfig.bAgcOpen = FALSE;
        stAiVqeConfig.bAnrOpen = bAiEnableNr;
        stAiVqeConfig.bEqOpen = FALSE;
        stAiVqeConfig.bHpfOpen = FALSE;
        stAiVqeConfig.s32WorkSampleRate = (MI_AUDIO_SampleRate_e)u32AiSampleRate;
        memcpy(&stAiVqeConfig.stAecCfg, &stAecCfg, sizeof(MI_AI_AecConfig_t));
        memcpy(&stAiVqeConfig.stAnrCfg, &stAnrCfg, sizeof(MI_AUDIO_AnrConfig_t));

        ret = MI_AI_SetVqeAttr(AiDevId, AiChn, 0, 0, &stAiVqeConfig);
        if(MI_SUCCESS != ret)
        {
            printf("MI_AI_SetVqeAttr failed\n");
            return ret;
        }

        ret = MI_AI_EnableVqe(AiDevId, AiChn);
        if(MI_SUCCESS != ret)
        {
            printf("MI_AI_EnableVqe failed\n");
            return ret;
        }
    }

    return MI_SUCCESS;
}

MI_S32 ST_AI_Deinit(void)
{
    MI_S32 ret;

    if(bAiEnableVqe)
    {
        ret = MI_AI_DisableVqe(AiDevId, AiChn);
        if(MI_SUCCESS != ret)
        {
            printf("MI_AI_DisableVqe failed\n");
            return ret;
        }
    }

    /* disable AI Channel */
    ret = MI_AI_DisableChn(AiDevId, AiChn);
    if(MI_SUCCESS != ret)
    {
        printf("disable Dev%d Chn%d err:0x%x\n", AiDevId, AiChn, ret);
        return ret;
    }

    /* disable AI Device */
    ret = MI_AI_Disable(AiDevId);
    if(MI_SUCCESS != ret)
    {
        printf("disable ai %d err:0x%x\n", AiDevId, ret);
        return ret;
    }

    return MI_SUCCESS;
}

#define TestLength (2048)
void* aiGetFrame(void* data)
{
    MI_S32 ret;
    ST_UAC_Frame_t stFrame ={};
    printf("Test point_getframe\n");
    //printf("length = %d \n",stAiChFrame.u32Len);
    short tdata[TestLength];
    short dataL[LEN1024_PACKET];
    short dataR[LEN1024_PACKET];

    short data_buf[LEN1024_PACKET];
    // short data_buf2[LEN1024_PACKET];

    short HpSOutH[LEN512_PACKET], HpSOutL[LEN512_PACKET];
    short NSOutL[LEN512_PACKET], NSOutH[LEN512_PACKET];
    short AGCOutL[LEN512_PACKET], AGCOutH[LEN512_PACKET];

    // short HpSOutH2[LEN512_PACKET], HpSOutL2[LEN512_PACKET];
    // short NSOutL2[LEN512_PACKET], NSOutH2[LEN512_PACKET];
    // short AGCOutL2[LEN512_PACKET], AGCOutH2[LEN512_PACKET];
	short dualmicenh[LEN1024_PACKET];
	float gain_last = 0;
    int i,j;
    int beishu, label;
	beishu = 2;
	label = beishu - 2;

    while(!g_bExit)
    {
        MI_AUDIO_Frame_t stAiChFrame;
        ret = MI_AI_GetFrame(AiDevId, AiChn, &stAiChFrame, NULL, -1);
        if(MI_SUCCESS == ret)
        {
            stFrame.data = stAiChFrame.apVirAddr[0];
            stFrame.length = stAiChFrame.u32Len;
            memcpy(tdata,stFrame.data,sizeof(short)*TestLength);
            for (int i=0;i<stFrame.length/4;i++)
            {
                dataL[i] = tdata[2 * i];
                dataR[i] = tdata[2 * i + 1];
            }

		DualMicEnhH(dataL, dataR, dualmicenh, LEN1024_PACKET, 2);
		InnoTalkSpl_AnalysisQMF(dualmicenh, HpSOutL, HpSOutH, Analysis_state11, Analysis_state12);
		InnoTalkNs_ProcessCore(Ns_Inst, HpSOutL, HpSOutH, NSOutL, NSOutH);
        for (j = 0; j < LEN512_PACKET; j++)
        {
            NSOutH[j] = (short)((float)NSOutH[j]*0.5);
        }
		// for (j = 0; j < LEN512_PACKET; j++)
		// {
		// 	NSOutL[j] = INNO_AGC_SAT(NSOutL[j], para_c[label], para_a[label], para_b[label], beishu);
		// 	NSOutH[j] = INNO_AGC_SAT(NSOutH[j], para_c[label], para_a[label], para_b[label], beishu);
		// }
		InnoTalkAgc_Process(Agc_Inst, NSOutL, NSOutH, LEN256_PACKET, AGCOutL, AGCOutH, inMicLevel, &outMicLevel, 0, &saturationWarning);
		InnoTalkAgc_Process(Agc_Inst, &NSOutL[LEN256_PACKET], &NSOutH[LEN256_PACKET], LEN256_PACKET, &AGCOutL[LEN256_PACKET], &AGCOutH[LEN256_PACKET], inMicLevel, &outMicLevel, 0, &saturationWarning);
		InnoTalkSpl_SynthesisQMF(AGCOutL, AGCOutH, data_buf, Synthesis_state11, Synthesis_state12);
// #endif

            for (int i=0;i<stFrame.length/4;i++)
            {
                tdata[2 * i] = data_buf[i];
			    tdata[2 * i + 1] = data_buf[i];
            }


            memcpy(stFrame.data,tdata,sizeof(short)*TestLength);
            //printf("length = %d \n",stFrame.length);

            ret = ST_UAC_SendFrame(c_handle, &stFrame);
            if(MI_SUCCESS != ret)
            {
                if(EBUSY != errno)
                    printf("ST_UAC_SendFrame failed:%s %d\n",  strerror(errno), errno);

                MI_AI_ReleaseFrame(AiDevId, AiChn, &stAiChFrame, NULL);
                usleep(1000);
                continue;
            }

            MI_AI_ReleaseFrame(AiDevId, AiChn, &stAiChFrame, NULL);
        }
        else
        {
            printf("MI_AI_GetFrame failed!\n");
            break;
        }
    }

    ret = 0;
    pthread_exit(&ret);
}

MI_S32 inline Vol_to_Db(MI_S32 Vol, MI_S32 Min_Vol, MI_S32 Max_Vol)
{
    switch(AiDevId)
    {
        case AI_DEV_AMIC:
            return ((Vol - Min_Vol) * (AI_VOLUME_AMIC_MAX - AI_VOLUME_AMIC_MIN) / (Max_Vol - Min_Vol)) + AI_VOLUME_AMIC_MIN;
            break;

         case AI_DEV_DMIC:
            return ((Vol - Min_Vol) * (AI_VOLUME_DMIC_MAX - AI_VOLUME_DMIC_MIN) / (Max_Vol - Min_Vol)) + AI_VOLUME_DMIC_MIN;
            break;

        case AI_DEV_LineIn:
            return ((Vol - Min_Vol) * (AI_VOLUME_LINEIN_MAX - AI_VOLUME_LINEIN_MIN) / (Max_Vol - Min_Vol)) + AI_VOLUME_LINEIN_MIN;
            break;

        default:
            break;
    }

    return 0;
}

void* aiSetVol(void* data)
{
    MI_U32 ret = MI_SUCCESS;

    struct mixer *mixer = mixer_open(0);
    if(!mixer)
    {
        fprintf(stderr, "Failed to open mixer\n");
        ret = EXIT_FAILURE;
        pthread_exit(&ret);
    }

    struct mixer_ctl *ctl;
    int min, max;

    ret = mixer_subscribe_events(mixer, 1);
    if(MI_SUCCESS != ret)
    {
        fprintf(stderr, "Failed to subscribe events\n");
        pthread_exit(&ret);
    }

    ctl = mixer_get_ctl(mixer, 0);
    min = mixer_ctl_get_range_min(ctl);
    max = mixer_ctl_get_range_max(ctl);
    if(!ctl)
    {
        fprintf(stderr, "Invalid mixer control\n");
        ret = EXIT_FAILURE;
        pthread_exit(&ret);
    }

    while(!g_bExit)
    {
        ret = mixer_wait_event(mixer, 1000);
        if(ret == 1)
        {
            ret = mixer_ctl_get_event(ctl, 0);
            if(MI_SUCCESS != ret)
            {
                printf("mixer_ctl_get_event failed\n");
                break;
            }

            AiVolume = mixer_ctl_get_value(ctl, 0);

            /* MI_AI_SetVqeVolume */
            MI_S32 s32VolumeDb = Vol_to_Db(AiVolume, min, max);
            ret = MI_AI_SetVqeVolume(AiDevId, AiChn, s32VolumeDb);
            if(MI_SUCCESS != ret)
            {
                printf("MI_AI_SetVqeVolume failed\n");
                break;
            }
            printf("current volume is %d\n", s32VolumeDb);
        }
    }

    ret = mixer_subscribe_events(mixer, 0);
    if(MI_SUCCESS != ret)
    {
        fprintf(stderr, "Failed to subscribe events\n");
        pthread_exit(&ret);
    }

    mixer_close(mixer);
    pthread_exit(&ret);
}

MI_S32 ST_AudioModuleInit(void)
{
    if(bEnableAO)
    {
        MI_S32 ret;

        ret = ST_AO_Init();
        if(MI_SUCCESS != ret)
        {
            printf("ST_AO_Init failed!\n");
            return ret;
        }

        ST_UAC_Config(&uac_playback_config, PCM_IN, 2, u32AoSampleRate, 1024, 4, PCM_FORMAT_S16_LE);
        ret = ST_UAC_Init(&p_handle, &uac_playback_config);
        if(MI_SUCCESS != ret)
        {
            printf("ST_UAC_Init failed!\n");
            return ret;
        }

        pthread_create(&Aotid, NULL, aoSendFrame, NULL);
        printf("create audio playback thread.\n");
    }

    if(bEnableAI)
    {
        MI_U32 ret;

        ret = ST_AI_Init();
        if(MI_SUCCESS != ret)
        {
            printf("ST_AI_Init failed!\n");
            return ret;
        }

        ST_UAC_Config(&uac_capture_config, PCM_OUT, 2, u32AiSampleRate, 1024, 4, PCM_FORMAT_S16_LE);
        ret = ST_UAC_Init(&c_handle, &uac_capture_config);
        if(MI_SUCCESS != ret)
        {
            printf("ST_UAC_Init failed!\n");
            return ret;
        }

        pthread_create(&Aitid, NULL, aiGetFrame, NULL);
        printf("create audio capture thread.\n");

        if(AI_DEV_AMIC == AiDevId || AI_DEV_DMIC == AiDevId || AI_DEV_LineIn == AiDevId)
        {
            pthread_create(&Voltid, NULL, aiSetVol, NULL);
            printf("create volume control thread.\n");
        }
        else
        {
            printf("this mode do not support volume control.\n");
        }
    }

    return MI_SUCCESS;
}

MI_S32 ST_AudioModuleUnInit(void)
{
    if(bEnableAO)
    {
        pthread_join(Aotid, NULL);
        printf("join Ao thread done.\n");

        ST_UAC_Uninit(p_handle);
        ST_AO_Deinit();
    }

    if(bEnableAI)
    {
        if(AI_DEV_AMIC == AiDevId || AI_DEV_DMIC == AiDevId || AI_DEV_LineIn == AiDevId)
        {
            pthread_join(Voltid, NULL);
            printf("join Vol thread done.\n");
        }

        pthread_join(Aitid, NULL);
        printf("join Ai thread done.\n");

        ST_UAC_Uninit(c_handle);
        ST_AI_Deinit();
    }

    return MI_SUCCESS;
}

#else

MI_BOOL checkAudioParam(void)
{
    return TRUE;
}

MI_S32 ST_AudioModuleInit(void)
{
    return MI_SUCCESS;
}

MI_S32 ST_AudioModuleUnInit(void)
{
    return MI_SUCCESS;
}

#endif

void ST_Flush(void)
{
    char c;

    while((c = getchar()) != '\n' && c != EOF);
}

MI_BOOL ST_DoSetIqBin(MI_VPE_CHANNEL Vpechn,char *pConfigPath)
{
    CUS3A_ALGO_STATUS_t stCus3aStatus;
    memset(&stCus3aStatus, 0, sizeof(CUS3A_ALGO_STATUS_t));

    MI_U8  u8ispreadycnt = 0;
    if (strlen(pConfigPath) == 0)
    {
        ST_ERR("IQ Bin File path is NULL!\n");
        return FALSE;
    }

    while(1)
    {
        if(u8ispreadycnt > 100)
        {
            ST_ERR("ISP ready time out!\n");
            u8ispreadycnt = 0;
            break;
        }

        CUS3A_GetAlgoStatus(0, &stCus3aStatus);

        if((stCus3aStatus.Ae == E_ALGO_STATUS_RUNNING) && (stCus3aStatus.Awb == E_ALGO_STATUS_RUNNING))
        {
            ST_DBG("Ready to load IQ bin :%s u8ispreadycnt:%d\n",pConfigPath, u8ispreadycnt);
            MI_ISP_API_CmdLoadBinFile(Vpechn, (char *)pConfigPath, 1234);

            usleep(10*1000);

            u8ispreadycnt = 0;
            break;

        }
        else
        {
            usleep(10*1000);
            u8ispreadycnt++;
        }

    }

    MI_ISP_AE_FLICKER_TYPE_e Flicker= SS_AE_FLICKER_TYPE_50HZ;
    MI_ISP_AE_SetFlicker(0, &Flicker);

    ST_DBG("MI_ISP_AE_SetFlicker:%d !\n", (int)Flicker);
    return 0;
}

MI_BOOL ST_DoChangeRotate(MI_SYS_WindowRect_t stSrcCrop)
{
    MI_S32 s32Rotation = 0;
    MI_S32 s32Mirror = 0;
    MI_S32 s32Flip = 0;
    MI_U8 i=0;
    MI_U32 u32VifDev=0, vifChn=0, VifPort=0;
    MI_S32  VpeChn = 0;
    ST_Stream_Attr_T *pstStreamAttr = g_stStreamAttr;
    MI_U32 u32ArraySize = ARRAY_SIZE(g_stStreamAttr);
    MI_VPE_ChannelAttr_t stVpeAttr;
    memset(&stVpeAttr,0,sizeof(MI_VPE_ChannelAttr_t));
    //printf("rotation(0:0, 1:90, 2:180, 3:270):");
    //scanf("%d", &s32Rotation);
    //ST_Flush();

    //printf("bmirror 0: FALSE, 1:TRUE :");
    //scanf("%d", &s32Mirror);
    //ST_Flush();

    //printf("bFlip 0: FALSE, 1:TRUE :");
    //scanf("%d", &s32Flip);
    //ST_Flush();
    for(i = 0;i < 1;i++)
    {
        if(!pstStreamAttr[i].bEnable)continue;
        u32VifDev = VpeChn = pstStreamAttr[i].u32InputChn;
        vifChn = u32VifDev*4;
        /************************************************
        Step1: Stop Venc (Because rot will change preview resolution)
        *************************************************/
        if(pstStreamAttr[i].bUserVenc)
        {
            MI_VENC_CHN VencChn =0;
            MI_U32 u32DevId = 0;
            ST_Sys_BindInfo_T stBindInfo;
            ExecFunc(MI_VENC_GetChnDevid(VencChn, &u32DevId), MI_SUCCESS);
            memset(&stBindInfo, 0x0, sizeof(ST_Sys_BindInfo_T));
            stBindInfo.stSrcChnPort.eModId = E_MI_MODULE_ID_VPE;
            stBindInfo.stSrcChnPort.u32DevId = 0;
            stBindInfo.stSrcChnPort.u32ChnId = pstStreamAttr[i].u32InputChn;
            stBindInfo.stSrcChnPort.u32PortId = pstStreamAttr[i].u32InputPort;

            stBindInfo.stDstChnPort.eModId = E_MI_MODULE_ID_VENC;
            stBindInfo.stDstChnPort.u32DevId = u32DevId;
            stBindInfo.stDstChnPort.u32ChnId = VencChn;
            stBindInfo.stDstChnPort.u32PortId = 0;

            stBindInfo.u32SrcFrmrate = 30;
            stBindInfo.u32DstFrmrate = 30;
            STCHECKRESULT(ST_Sys_UnBind(&stBindInfo));


            STCHECKRESULT(ST_Venc_StopChannel(VencChn));
        }
        MI_VPE_GetChannelAttr(VpeChn,&stVpeAttr);
        /************************************************
        Step2: Stop Vpe (Wait driver all buffer done)
        *************************************************/
        STCHECKRESULT(MI_VPE_StopChannel(VpeChn));

        /************************************************
        Step3: Disable Vif Port(Realtime mode Change Rot Will Change Isp Cfg, need stop push vif stream)
        *************************************************/
        if(stVpeAttr.eRunningMode == E_MI_VPE_RUN_REALTIME_MODE)
            STCHECKRESULT(MI_VIF_DisableChnPort(vifChn, VifPort));

        /************************************************
        Step4: Set Vpe Rot/ChnMirror/ChnFlip
        *************************************************/
        STCHECKRESULT(MI_VPE_SetChannelRotation(VpeChn, (MI_SYS_Rotate_e)s32Rotation));

        MI_VPE_ChannelPara_t stChnParam;
        memset(&stChnParam, 0x0, sizeof(MI_VPE_ChannelPara_t));

        STCHECKRESULT(MI_VPE_GetChannelParam(VpeChn, &stChnParam));
        stChnParam.bMirror = s32Mirror;
        stChnParam.bFlip = s32Flip;
        STCHECKRESULT(MI_VPE_SetChannelParam(VpeChn, &stChnParam));

        /************************************************
        Step5: Rot set crop size
        *************************************************/
        //MI_SYS_WindowRect_t stOutCropInfo;
        STCHECKRESULT(MI_VPE_SetPortCrop(VpeChn, pstStreamAttr[i].u32InputPort, &stSrcCrop));

        /************************************************
        Step7: Start Vif
        *************************************************/
        if(stVpeAttr.eRunningMode == E_MI_VPE_RUN_REALTIME_MODE)
            STCHECKRESULT(MI_VIF_EnableChnPort(vifChn, VifPort));

        /************************************************
        Step8: Start Vpe
        *************************************************/
        STCHECKRESULT(MI_VPE_StartChannel (VpeChn));

        /************************************************
        Step9: Start Venc
        *************************************************/
        if(pstStreamAttr[i].bUserVenc)
        {
            MI_VENC_CHN VencChn =pstStreamAttr[i].vencChn;
            MI_U32 u32DevId = -1;
            ST_Sys_BindInfo_T stBindInfo;
            ExecFunc(MI_VENC_GetChnDevid(VencChn, &u32DevId), MI_SUCCESS);
            memset(&stBindInfo, 0x0, sizeof(ST_Sys_BindInfo_T));
            stBindInfo.stSrcChnPort.eModId = E_MI_MODULE_ID_VPE;
            stBindInfo.stSrcChnPort.u32DevId = 0;
            stBindInfo.stSrcChnPort.u32ChnId = pstStreamAttr[i].u32InputChn;
            stBindInfo.stSrcChnPort.u32PortId = pstStreamAttr[i].u32InputPort;

            stBindInfo.stDstChnPort.eModId = E_MI_MODULE_ID_VENC;
            stBindInfo.stDstChnPort.u32DevId = u32DevId;
            stBindInfo.stDstChnPort.u32ChnId = VencChn;
            stBindInfo.stDstChnPort.u32PortId = 0;

            stBindInfo.u32SrcFrmrate = 30;
            stBindInfo.u32DstFrmrate = 30;
            stBindInfo.eBindType = pstStreamAttr[i].eBindType;
            stBindInfo.u32BindParam = pstStreamAttr[i].u32BindPara;
            STCHECKRESULT(ST_Sys_Bind(&stBindInfo));


            STCHECKRESULT(ST_Venc_StartChannel(VencChn));
        }
    }
    return 0;
}

MI_S32 ST_Test()
{

    MI_S32 select = -1;
    MI_SYS_WindowRect_t stChnCropWin;
    MI_BOOL bMirror = FALSE;
    MI_BOOL bFlip = FALSE;
    memset(&stChnCropWin, 0x0, sizeof(stChnCropWin));

    while(!g_bExit)
    {

        /* sensor imx415 */
        printf("****************************************************************\n");
        printf("1:vpe-channel-crop 2: mirror 3: cus3a test 4:test vpe rotate/mirro/flip 0:exit \n");
        printf("****************************************************************\n");
        scanf("%d", &select);
        ST_Flush();

        switch(select){
           case 1:
                stChnCropWin.u16X = 0;
                stChnCropWin.u16Y = 0;
                stChnCropWin.u16Width = 1280;
                stChnCropWin.u16Height = 720;
                STCHECKRESULT(MI_VPE_SetChannelCrop(0, &stChnCropWin));
                break;

            case 2:
                bMirror = TRUE;
                bFlip = FALSE;
                STCHECKRESULT(MI_SNR_SetOrien(E_MI_SNR_PAD_ID_0, bMirror, bFlip));
                break;

            case 3:
                ST_EnableCustomize3A();
                break;

            case 4:
            //    ST_DoChangeRotate();
                break;

            case 0:
                g_bExit = TRUE;
                break;
            }
   }
        return MI_SUCCESS;

}

#ifdef CONFIG_SIGMASTAR_CHIP_I6E

#define LDC_MAX_VIEWNUM 4

MI_S32 LdcParseLibarayCfgFilePath(char *pLdcLibCfgPath, mi_eptz_config_param *ptconfig_para)
{
    mi_eptz_err err_state = MI_EPTZ_ERR_NONE;

    err_state = mi_eptz_config_parse(pLdcLibCfgPath, ptconfig_para);
    if (err_state != MI_EPTZ_ERR_NONE)
    {
        printf("confile file read error: %d\n", err_state);
        return err_state;
    }

    printf("ldc mode %d \n", ptconfig_para->ldc_mode);
    return 0;
}

MI_S32 LdcGetCfgViewNum(mi_LDC_MODE eLdcMode)
{
    MI_U32 u32ViewNum = 0;

    switch(eLdcMode)
    {
        case LDC_MODE_1R:
        case LDC_MODE_1P_CM:
        case LDC_MODE_1P_WM:
        case LDC_MODE_1O:
        case LDC_MODE_1R_WM:
            u32ViewNum = 1;
            break;
        case LDC_MODE_2P_CM:
        case LDC_MODE_2P_DM:
            u32ViewNum = 2;
            break;
        case LDC_MODE_4R_CM:
        case LDC_MODE_4R_WM:
            u32ViewNum = 4;
            break;
        default:
            printf("########### ldc mode %d err \n", eLdcMode);
            break;
    }

    printf("view num %d \n", u32ViewNum);
    return u32ViewNum;
}


MI_S32 LdcLibarayCreatBin(MI_S32 s32ViewId, mi_eptz_config_param *ptconfig_para,
                          LDC_BIN_HANDLE *ptldc_bin, MI_U32 *pu32LdcBinSize, MI_S32 s32Rot)
{
    unsigned char* pWorkingBuffer;
    int working_buf_len = 0;
    mi_eptz_err err_state = MI_EPTZ_ERR_NONE;
    EPTZ_DEV_HANDLE eptz_handle = NULL;

    mi_eptz_para teptz_para;
    memset(&teptz_para, 0x0, sizeof(mi_eptz_para));

    printf("view %d rot %d\n", s32ViewId, s32Rot);

    working_buf_len = mi_eptz_get_buffer_info(ptconfig_para);
    pWorkingBuffer = (unsigned char*)malloc(working_buf_len);
    if (pWorkingBuffer == NULL)
    {
        printf("strerror: %s\n", strerror(errno));
        printf("buffer allocate error,buf_len:%d\n",working_buf_len);
        return MI_EPTZ_ERR_MEM_ALLOCATE_FAIL;
    }

   // printf("%s:%d working_buf_len %d \n", __FUNCTION__, __LINE__, working_buf_len);

    //EPTZ init
    teptz_para.ptconfig_para = ptconfig_para; //ldc configure

  //  printf("%s:%d ptconfig_para %p, pWorkingBuffer %p, working_buf_len %d\n", __FUNCTION__, __LINE__, teptz_para.ptconfig_para,
    //    pWorkingBuffer, working_buf_len);

    eptz_handle =  mi_eptz_runtime_init(pWorkingBuffer, working_buf_len, &teptz_para);
    if (eptz_handle == NULL)
    {
        printf("EPTZ init error\n");
        return MI_EPTZ_ERR_NOT_INIT;
    }

    teptz_para.pan = 0;
    teptz_para.tilt = -60;
    if(ptconfig_para->ldc_mode == 1)
        teptz_para.tilt = 60;
    teptz_para.rotate = s32Rot;
    teptz_para.zoom = 150.00;
    teptz_para.out_rot = 0;
    teptz_para.view_index = s32ViewId;

    //Gen bin files from 0 to 360 degree
    switch (ptconfig_para->ldc_mode)
    {
        case LDC_MODE_4R_CM:  //LDC_MODE_4R_CM/Desk, if in desk mount mode, tilt is nagetive.
            teptz_para.view_index = s32ViewId;
            teptz_para.pan = 0;
            teptz_para.tilt = -50; //In CM mode, tilt is positive, but in desk mode, tilt is negative.
            teptz_para.rotate = s32Rot;
            teptz_para.zoom = 150;
            teptz_para.out_rot = 0;
            err_state = (mi_eptz_err)mi_eptz_runtime_map_gen(eptz_handle,(mi_eptz_para*)&teptz_para, ptldc_bin, (int *)pu32LdcBinSize);
            if (err_state != MI_EPTZ_ERR_NONE)
            {
                printf("[EPTZ ERR] =  %d !! \n", err_state);
            }
            break;
        case LDC_MODE_4R_WM:  //LDC_MODE_4R_WM
            teptz_para.view_index = s32ViewId;
            teptz_para.pan = 0;
            teptz_para.tilt = 50; //In CM mode, tilt is positive, but in desk mode, tilt is negative.
            teptz_para.rotate = s32Rot;
            teptz_para.zoom = 150;
            teptz_para.out_rot = 0;
            err_state = (mi_eptz_err)mi_eptz_runtime_map_gen(eptz_handle,(mi_eptz_para*)&teptz_para, ptldc_bin, (int *)pu32LdcBinSize);
            if (err_state != MI_EPTZ_ERR_NONE)
            {
                printf("[EPTZ ERR] =  %d !! \n", err_state);
            }
            break;
        case LDC_MODE_1R:  //LDC_MODE_1R CM/Desk,  if in desk mount mode, tilt is negative.
            teptz_para.view_index = s32ViewId;
            teptz_para.pan = 0;
            teptz_para.tilt = 0; //In CM mode, tilt is positive, but in desk mode, tilt is negative.
            teptz_para.rotate = s32Rot;
            teptz_para.zoom = 150;
            printf("****zoom:%f*****\n",teptz_para.zoom);
            teptz_para.out_rot = 0;
            err_state = (mi_eptz_err)mi_eptz_runtime_map_gen(eptz_handle,(mi_eptz_para*)&teptz_para, ptldc_bin, (int *)pu32LdcBinSize);
            if (err_state != MI_EPTZ_ERR_NONE)
            {
                printf("[EPTZ ERR] =  %d !! \n", err_state);
            }
            break;
        case LDC_MODE_1R_WM:  //LDC_MODE_1R WM
            teptz_para.view_index = s32ViewId;
            teptz_para.pan = 0;
            teptz_para.tilt = 50; //In CM mode, tilt is positive, but in desk mode, tilt is negative.
            teptz_para.rotate = s32Rot;
            teptz_para.zoom = 150;
            teptz_para.out_rot = 0;

            err_state = (mi_eptz_err)mi_eptz_runtime_map_gen(eptz_handle,(mi_eptz_para*)&teptz_para, ptldc_bin, (int *)pu32LdcBinSize);
            if (err_state != MI_EPTZ_ERR_NONE)
            {
                printf("[EPTZ ERR] =  %d !! \n", err_state);
            }
            break;

        case LDC_MODE_2P_CM:  //LDC_MODE_2P_CM
        case LDC_MODE_2P_DM:  //LDC_MODE_2P_DM
        case LDC_MODE_1P_CM:  //LDC_MODE_1P_CM
            //Set the input parameters for donut mode
            if(s32Rot > 180)
            {
                //Degree 180 ~ 360
                teptz_para.view_index = s32ViewId;
                teptz_para.r_inside = 550;
                teptz_para.r_outside = 10;
                teptz_para.theta_start = s32Rot;
                teptz_para.theta_end = s32Rot+360;
            }
            else
            {
                //Degree 180 ~ 0
                teptz_para.view_index = s32ViewId;
                teptz_para.r_inside = 10;
                teptz_para.r_outside = 550;
                teptz_para.theta_start = s32Rot;
                teptz_para.theta_end = s32Rot+360;
            }
            err_state = (mi_eptz_err)mi_donut_runtime_map_gen(eptz_handle, (mi_eptz_para*)&teptz_para, ptldc_bin, (int *)pu32LdcBinSize);
            if (err_state != MI_EPTZ_ERR_NONE)
            {
                printf("[EPTZ ERR] =  %d !! \n", err_state);
            }
            break;
        case LDC_MODE_1P_WM:  //LDC_MODE_1P wall mount.
            teptz_para.view_index = s32ViewId;
            teptz_para.pan = 0;
            teptz_para.tilt = 0;
            teptz_para.zoom_h = 100;
            teptz_para.zoom_v = 100;
            err_state = (mi_eptz_err)mi_erp_runtime_map_gen(eptz_handle,(mi_eptz_para*)&teptz_para, ptldc_bin, (int *)pu32LdcBinSize);
            if (err_state != MI_EPTZ_ERR_NONE)
            {
                printf("[EPTZ ERR] =  %d !! \n", err_state);
            }
            break;
        case LDC_MODE_1O:    //bypass mode
            teptz_para.view_index = 0; //view index
            printf("begin mi_bypass_runtime_map_gen \n");
            err_state = (mi_eptz_err)mi_bypass_runtime_map_gen(eptz_handle, (mi_eptz_para*)&teptz_para, ptldc_bin, (int *)pu32LdcBinSize);
            if (err_state != MI_EPTZ_ERR_NONE)
            {
                printf("[MODE %d ERR] =  %d !! \n", LDC_MODE_1O, err_state);
                return err_state;
            }

            printf("end mi_bypass_runtime_map_gen\n");
            break;
        default :
             printf("********************err ldc mode %d \n", ptconfig_para->ldc_mode);
             return 0;
    }

    free(pWorkingBuffer);

    return 0;
}


MI_S32 LdcReadTableBin(const char *pConfigPath, LDC_BIN_HANDLE *tldc_bin, MI_U32 *pu32BinSize)
{
    struct stat statbuff;
    MI_U8 *pBufData = NULL;
    MI_S32 s32Fd = 0;
    MI_U32 u32Size = 0;

    if (pConfigPath == NULL)
    {
        ST_ERR("File path null!\n");
        return MI_ERR_LDC_ILLEGAL_PARAM;
    }
    printf("Read file %s\n", pConfigPath);
    memset(&statbuff, 0, sizeof(struct stat));
    if(stat(pConfigPath, &statbuff) < 0)
    {
        ST_ERR("Bb table file not exit!\n");
        return MI_ERR_LDC_ILLEGAL_PARAM;
    }
    else
    {
        if (statbuff.st_size == 0)
        {
            ST_ERR("File size is zero!\n");
            return MI_ERR_LDC_ILLEGAL_PARAM;
        }
        u32Size = statbuff.st_size;
    }
    s32Fd = open(pConfigPath, O_RDONLY);
    if (s32Fd < 0)
    {
        ST_ERR("Open file[%d] error!\n", s32Fd);
        return MI_ERR_LDC_ILLEGAL_PARAM;
    }
    pBufData = (MI_U8 *)malloc(u32Size);
    if (!pBufData)
    {
        ST_ERR("Malloc error!\n");
        close(s32Fd);

        return MI_ERR_LDC_ILLEGAL_PARAM;
    }

    memset(pBufData, 0, u32Size);
    read(s32Fd, pBufData, u32Size);
    close(s32Fd);

    *tldc_bin = pBufData;
    *pu32BinSize = u32Size;

    printf("%d: &bin address %p, *binbuffer %p \n",__LINE__, tldc_bin, *tldc_bin);

    return MI_SUCCESS;
}


MI_S32 GetLdcBinBuffer(MI_BOOL bIsBinPath, char *pCfgFilePath, mi_eptz_config_param *pstCfgPara, MI_U32 *pu32ViewNum, LDC_BIN_HANDLE *phLdcBin, MI_U32 *pu32LdcBinSize, MI_S32 *ps32Rot)
{
    if(bIsBinPath)
    {
        STCHECKRESULT(LdcReadTableBin(pCfgFilePath, phLdcBin, pu32LdcBinSize));
        *pu32ViewNum = 1;
    }
    else
    {
        STCHECKRESULT(LdcParseLibarayCfgFilePath(pCfgFilePath, pstCfgPara));

        MI_U32 u32ViewNum = 0;

        u32ViewNum = LdcGetCfgViewNum(pstCfgPara->ldc_mode);
        for(MI_U32 i = 0; i < u32ViewNum; i++)
        {
            printf("phLdcBin[%d],%s\n",i,(char*)phLdcBin[i]);
            STCHECKRESULT(LdcLibarayCreatBin(i, pstCfgPara, &phLdcBin[i], &pu32LdcBinSize[i], ps32Rot[i]));
        }

        *pu32ViewNum = u32ViewNum;
    }

    return 0;
}

MI_S32 LdcCheckBinPath(char *pLdcCfgPath, MI_BOOL &bIsBinPath)
{
    ST_DBG("pLdcCfgPath: %s\n", pLdcCfgPath);
    char *pstr = NULL;

    pstr = strstr(pLdcCfgPath, ".bin");
    if(pstr != NULL)
    {
        bIsBinPath = TRUE;
        return MI_SUCCESS;
    }

    pstr = strstr(pLdcCfgPath, ".cfg");
    if(pstr != NULL)
    {
        bIsBinPath = FALSE;
        return MI_SUCCESS;
    }

    return MI_ERR_LDC_ILLEGAL_PARAM;
}


MI_S32 InitLdc(MI_VPE_CHANNEL VpeChannel, char *pLdcBinPath)
{
    MI_BOOL bIsBinPath = FALSE;
    mi_eptz_config_param stCfgPara;
    MI_U32 u32ViewNum = 0;
    LDC_BIN_HANDLE hLdcBin[LDC_MAX_VIEWNUM] = {NULL};
    MI_U32 u32LdcBinSize[LDC_MAX_VIEWNUM] = {0};
    MI_S32  s32Rot[LDC_MAX_VIEWNUM] = {0, 90, 180, 270};

    memset(&stCfgPara, 0, sizeof(mi_eptz_config_param));

    STCHECKRESULT(LdcCheckBinPath(pLdcBinPath, bIsBinPath));
    ST_DBG("bIsBinPath: %d\n", (int)bIsBinPath);

    STCHECKRESULT(GetLdcBinBuffer(bIsBinPath, pLdcBinPath, &stCfgPara,
        &u32ViewNum, hLdcBin, u32LdcBinSize, s32Rot));

    STCHECKRESULT(MI_VPE_LDCBegViewConfig(VpeChannel));

    for(MI_U32 i = 0; i < u32ViewNum; i++)
    {
        STCHECKRESULT(MI_VPE_LDCSetViewConfig(VpeChannel, hLdcBin[i], u32LdcBinSize[i]));

        if(mi_eptz_buffer_free(hLdcBin[i]) != MI_EPTZ_ERR_NONE)
        {
            ST_ERR("[MI EPTZ ERR]   %d !! \n", __LINE__);
        }
    }

    STCHECKRESULT(MI_VPE_LDCEndViewConfig(VpeChannel));

    return MI_SUCCESS;
}

#endif

MI_S32 ST_VideoModuleInit(ST_Config_S* pstConfig)
{
    MI_U32 u32CapWidth = 0, u32CapHeight = 0;
    MI_VIF_FrameRate_e eFrameRate = E_MI_VIF_FRAMERATE_FULL;
    MI_SYS_PixelFormat_e ePixFormat;
    ST_VPE_ChannelInfo_T stVpeChannelInfo;
    ST_Sys_BindInfo_T stBindInfo;
    MI_SNR_PADInfo_t  stPad0Info;
    MI_SNR_PlaneInfo_t stSnrPlane0Info;
    MI_VIF_WorkMode_e eVifWorkMode = E_MI_VIF_WORK_MODE_RGB_REALTIME;
    MI_VIF_HDRType_e eVifHdrType = E_MI_VIF_HDR_TYPE_OFF;
    MI_U32 u32ResCount =0;
    MI_U8 u8ResIndex =0;
    MI_SNR_Res_t stRes;
    MI_U32 u32ChocieRes =0;

    memset(&stPad0Info, 0x0, sizeof(MI_SNR_PADInfo_t));
    memset(&stSnrPlane0Info, 0x0, sizeof(MI_SNR_PlaneInfo_t));
    memset(&stRes, 0x0, sizeof(MI_SNR_Res_t));

    MI_SNR_PAD_ID_e      ePADId = (MI_SNR_PAD_ID_e)pstConfig->u8SnrPad;

    ST_DBG("Snr pad id:%d\n", (int)ePADId);

    if(gEnableHdr)
    {
        printf("Please select HDR type!\n 0 not use, 1 use VC, 2 use DOL, 3 use EMBEDDED, 4 use LI\n");
        printf("sony sensor(ex imx307) use DOL, sc sensor(ex sc4238) use VC\n");

        int select = 0;
        scanf("%d", &select);

        ST_Flush();
        printf("You select %d HDR\n", select);

        pstConfig->s32HDRtype = select;
        eVifWorkMode = E_MI_VIF_WORK_MODE_RGB_FRAMEMODE;//TO DO:because realtime mode has FIFO FULL issue
    }

    if(pstConfig->s32HDRtype > 0)
        MI_SNR_SetPlaneMode(ePADId, TRUE);
    else
        MI_SNR_SetPlaneMode(ePADId, FALSE);

    MI_SNR_QueryResCount(ePADId, &u32ResCount);
    for(u8ResIndex=0; u8ResIndex < u32ResCount; u8ResIndex++)
    {
        MI_SNR_GetRes(ePADId, u8ResIndex, &stRes);
        printf("index %d, Crop(%d,%d,%d,%d), outputsize(%d,%d), maxfps %d, minfps %d, ResDesc %s\n",
        u8ResIndex,
        stRes.stCropRect.u16X, stRes.stCropRect.u16Y, stRes.stCropRect.u16Width,stRes.stCropRect.u16Height,
        stRes.stOutputSize.u16Width, stRes.stOutputSize.u16Height,
        stRes.u32MaxFps,stRes.u32MinFps,
        stRes.strResDesc);
    }

    MI_S8 s8SnrResIndex = pstConfig->s8SnrResIndex;

    if(s8SnrResIndex < 0)
    {
        printf("choice which resolution use, cnt %d\n", u32ResCount);
        do
        {
            scanf("%d", &u32ChocieRes);
            ST_Flush();
            MI_SNR_QueryResCount(ePADId, &u32ResCount);
            if(u32ChocieRes >= u32ResCount)
            {
                printf("choice err res %d > =cnt %d\n", u32ChocieRes, u32ResCount);
            }
        }while(u32ChocieRes >= u32ResCount);
    }
    else
    {
        if((MI_U32)s8SnrResIndex >= u32ResCount)
        {
            ST_ERR("snr index:%d exceed u32ResCount:%d, set default index 0\n", s8SnrResIndex, u32ResCount);
            s8SnrResIndex = 0;
        }

        u32ChocieRes = s8SnrResIndex;
    }
    printf("You select %d res\n", u32ChocieRes);

    MI_SNR_SetRes(ePADId,u32ChocieRes);
    MI_SNR_Enable(ePADId);

    MI_SNR_GetPadInfo(ePADId, &stPad0Info);
    MI_SNR_GetPlaneInfo(ePADId, 0, &stSnrPlane0Info);

    u32CapWidth = stSnrPlane0Info.stCapRect.u16Width;
    u32CapHeight = stSnrPlane0Info.stCapRect.u16Height;
    eFrameRate = E_MI_VIF_FRAMERATE_FULL;
    ePixFormat = (MI_SYS_PixelFormat_e)RGB_BAYER_PIXEL(stSnrPlane0Info.ePixPrecision, stSnrPlane0Info.eBayerId);

    //g_stStreamAttr[0].u32Width = u32CapWidth;
    //g_stStreamAttr[0].u32Height = u32CapHeight;

    if(ePADId == E_MI_SNR_PAD_ID_1)
    {
        //Bind vif dev 0 to sensor pad 1, because only vif dev0 can use realtime mode.
        MI_VIF_Dev2SnrPadMuxCfg_t stDev2SnrPad[3];
        stDev2SnrPad[0].eSensorPadID = E_MI_VIF_SNRPAD_ID_1;
        stDev2SnrPad[0].u32PlaneID = 0xff;
        stDev2SnrPad[1].eSensorPadID = E_MI_VIF_SNRPAD_ID_2;
        stDev2SnrPad[1].u32PlaneID = 0xff;
        stDev2SnrPad[2].eSensorPadID = E_MI_VIF_SNRPAD_ID_0;
        stDev2SnrPad[2].u32PlaneID = 0xff;
        MI_VIF_SetDev2SnrPadMux(stDev2SnrPad, 3);
    }

    eVifHdrType = (MI_VIF_HDRType_e)pstConfig->s32HDRtype;
#ifdef CONFIG_SIGMASTAR_CHIP_I6E
    system("echo scllevel 1 > /proc/vpe/vpe_debug");
#endif

    STCHECKRESULT(ST_Vif_EnableDev(0, eVifWorkMode, eVifHdrType, &stPad0Info));

    ST_VIF_PortInfo_T stVifPortInfoInfo;
    memset(&stVifPortInfoInfo, 0, sizeof(ST_VIF_PortInfo_T));
    stVifPortInfoInfo.u32RectX = 0;
    stVifPortInfoInfo.u32RectY = 0;
    stVifPortInfoInfo.u32RectWidth = u32CapWidth;
    stVifPortInfoInfo.u32RectHeight = u32CapHeight;
    stVifPortInfoInfo.u32DestWidth = u32CapWidth;
    stVifPortInfoInfo.u32DestHeight = u32CapHeight;
    stVifPortInfoInfo.eFrameRate = eFrameRate;
    stVifPortInfoInfo.ePixFormat = ePixFormat;
    STCHECKRESULT(ST_Vif_CreatePort(0, 0, &stVifPortInfoInfo));
    STCHECKRESULT(ST_Vif_StartPort(0, 0, 0));
    //if (enRotation != E_MI_SYS_ROTATE_NONE)
    {
        MI_BOOL bMirror = FALSE, bFlip = FALSE;
        //ExecFunc(MI_VPE_SetChannelRotation(0, enRotation), MI_SUCCESS);
        //pstConfig->enRotation = E_MI_SYS_ROTATE_180;
        switch(pstConfig->enRotation)
        {
        case E_MI_SYS_ROTATE_NONE:
            bMirror= FALSE;
            bFlip = FALSE;
            break;
        case E_MI_SYS_ROTATE_90:
            bMirror = FALSE;
            bFlip = TRUE;
            break;
        case E_MI_SYS_ROTATE_180:
            bMirror = TRUE;
            bFlip = TRUE;
            break;
        case E_MI_SYS_ROTATE_270:
            bMirror = TRUE;
            bFlip = FALSE;
            break;
        default:
            break;
        }

        MI_SNR_SetOrien(ePADId, bMirror, bFlip);
        // MI_VPE_SetChannelRotation(0, pstConfig->enRotation);
    }

    //for (unsigned int i = 0; i < g_device_num;i++)
    for (unsigned int i = 0; i < 1; i++)
    {
        ST_Stream_Attr_T *pstStreamAttr = &g_stStreamAttr[i];

        memset(&stVpeChannelInfo, 0, sizeof(ST_VPE_ChannelInfo_T));
        memset(&stBindInfo, 0x0, sizeof(ST_Sys_BindInfo_T));

        stVpeChannelInfo.u16VpeMaxW = u32CapWidth;
        stVpeChannelInfo.u16VpeMaxH = u32CapHeight;
        stVpeChannelInfo.u32X = 0;
        stVpeChannelInfo.u32Y = 0;
        stVpeChannelInfo.u16VpeCropW = 0;
        stVpeChannelInfo.u16VpeCropH = 0;

        if(gEnableHdr)
        {
            stVpeChannelInfo.eRunningMode = E_MI_VPE_RUN_CAM_MODE;
            stBindInfo.eBindType = E_MI_SYS_BIND_TYPE_FRAME_BASE;
        }
        else
        {
            stBindInfo.eBindType = E_MI_SYS_BIND_TYPE_REALTIME;
            stVpeChannelInfo.eRunningMode = E_MI_VPE_RUN_REALTIME_MODE;
        }
        stVpeChannelInfo.eFormat = ePixFormat;
        stVpeChannelInfo.e3DNRLevel = pstConfig->en3dNrLevel;
        stVpeChannelInfo.eHDRtype = (MI_VPE_HDRType_e)pstConfig->s32HDRtype;
        stVpeChannelInfo.bRotation = TRUE;
        stVpeChannelInfo.eBindSensorId = (MI_VPE_SensorChannel_e)(ePADId + 1);

        if(gbLdcEnable)
        {
            ST_DBG("LDC enable!\n");
            stVpeChannelInfo.bEnableLdc = TRUE;
            stVpeChannelInfo.u32ChnPortMode = E_MI_VPE_ZOOM_LDC_MAX;
        }
        else
        {
            stVpeChannelInfo.bEnableLdc = FALSE;
            /* crop vpe channel need set zoom mode */
            stVpeChannelInfo.u32ChnPortMode = E_MI_VPE_ZOOM_LDC_NULL;
        }

        STCHECKRESULT(ST_Vpe_CreateChannel(pstStreamAttr->u32InputChn, &stVpeChannelInfo));

        //LDC Flow
        if(gbLdcEnable)
        {

            #ifdef CONFIG_SIGMASTAR_CHIP_I6E

            InitLdc(pstStreamAttr->u32InputChn, gaLdcBinPath);

            #endif
        }

        STCHECKRESULT(ST_Vpe_StartChannel(pstStreamAttr->u32InputChn));
        //MI_VPE_SetChannelRotation(0,E_MI_SYS_ROTATE_180);

        stBindInfo.stSrcChnPort.eModId = E_MI_MODULE_ID_VIF;
        stBindInfo.stSrcChnPort.u32DevId = 0;
        stBindInfo.stSrcChnPort.u32ChnId = 0;
        stBindInfo.stSrcChnPort.u32PortId = 0;
        stBindInfo.stDstChnPort.eModId = E_MI_MODULE_ID_VPE;
        stBindInfo.stDstChnPort.u32DevId = 0;
        stBindInfo.stDstChnPort.u32ChnId = pstStreamAttr->u32InputChn;
        stBindInfo.stDstChnPort.u32PortId = pstStreamAttr->u32InputPort;
        stBindInfo.u32SrcFrmrate = 30;
        stBindInfo.u32DstFrmrate = 30;
        STCHECKRESULT(ST_Sys_Bind(&stBindInfo));
    }

    if(g_enable_iqserver)
    {
        ST_DBG("MI_IQSERVER_Open...\n");
        STCHECKRESULT(MI_IQSERVER_Open(u32CapWidth, u32CapHeight, 0));
    }

    return MI_SUCCESS;
}

MI_S32 ST_VideoModuleUnInit(ST_Config_S* pstConfig)
{
    MI_SNR_PAD_ID_e      ePADId = (MI_SNR_PAD_ID_e)pstConfig->u8SnrPad;
    ST_Sys_BindInfo_T stBindInfo;

    memset(&stBindInfo, 0x0, sizeof(ST_Sys_BindInfo_T));
    stBindInfo.stSrcChnPort.eModId = E_MI_MODULE_ID_VIF;
    stBindInfo.stSrcChnPort.u32DevId = 0;
    stBindInfo.stSrcChnPort.u32ChnId = 0;
    stBindInfo.stSrcChnPort.u32PortId = 0;

    stBindInfo.stDstChnPort.eModId = E_MI_MODULE_ID_VPE;
    stBindInfo.stDstChnPort.u32DevId = 0;
    stBindInfo.stDstChnPort.u32ChnId = 0;
    stBindInfo.stDstChnPort.u32PortId = 0;

    stBindInfo.u32SrcFrmrate = 30;
    stBindInfo.u32DstFrmrate = 30;
    STCHECKRESULT(ST_Sys_UnBind(&stBindInfo));

    STCHECKRESULT(ST_Vpe_StopChannel(0));
    STCHECKRESULT(ST_Vpe_DestroyChannel(0));

    STCHECKRESULT(ST_Vif_StopPort(0, 0));
    STCHECKRESULT(ST_Vif_DisableDev(0));
    STCHECKRESULT(MI_SNR_Disable(ePADId));

    if(g_enable_iqserver)
    {
        MI_IQSERVER_Close();
    }

    return MI_SUCCESS;
}

void *ST_DIVPGetResult(void *args)
{
    MI_SYS_ChnPort_t stChnPort;
    MI_SYS_BufInfo_t stBufInfo;
    MI_SYS_BUF_HANDLE stBufHandle;
    MI_S32 s32Ret = MI_SUCCESS;
    MI_S32 s32Fd = 0;
    fd_set read_fds;
    struct timeval TimeoutVal;
    char szFileName[128];
    int fd = 0;
    MI_U32 u32GetFramesCount = 0;
    MI_BOOL _bWriteFile = TRUE;

    stChnPort.eModId = E_MI_MODULE_ID_DIVP;
    stChnPort.u32DevId = 0;
    stChnPort.u32ChnId = DIVP_CHN_FOR_VDF;
    stChnPort.u32PortId = 0;

    s32Ret = MI_SYS_GetFd(&stChnPort, &s32Fd);
    if(MI_SUCCESS != s32Ret)
    {
        ST_ERR("MI_SYS_GetFd 0, error, %X\n", s32Ret);
        return NULL;
    }
    s32Ret = MI_SYS_SetChnOutputPortDepth(&stChnPort, 2, 3);
    if (MI_SUCCESS != s32Ret)
    {
        ST_ERR("MI_SYS_SetChnOutputPortDepth err:%x, chn:%d,port:%d\n", s32Ret,
            stChnPort.u32ChnId, stChnPort.u32PortId);
        return NULL;
    }

    sprintf(szFileName, "divp%d.es", stChnPort.u32ChnId);
    printf("start to record %s\n", szFileName);
    fd = open(szFileName, O_RDWR | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
    if (fd < 0)
    {
        ST_ERR("create %s fail\n", szFileName);
    }

    while (1)
    {
        FD_ZERO(&read_fds);
        FD_SET(s32Fd, &read_fds);

        TimeoutVal.tv_sec  = 1;
        TimeoutVal.tv_usec = 0;

        s32Ret = select(s32Fd + 1, &read_fds, NULL, NULL, &TimeoutVal);

        if(s32Ret < 0)
        {
            ST_ERR("select failed!\n");
            //  usleep(10 * 1000);
            continue;
        }
        else if(s32Ret == 0)
        {
            ST_ERR("get divp frame time out\n");
            //usleep(10 * 1000);
            continue;
        }
        else
        {
            if(FD_ISSET(s32Fd, &read_fds))
            {
                s32Ret = MI_SYS_ChnOutputPortGetBuf(&stChnPort, &stBufInfo, &stBufHandle);

                if(MI_SUCCESS != s32Ret)
                {
                    //ST_ERR("MI_SYS_ChnOutputPortGetBuf err, %x\n", s32Ret);
                    continue;
                }

                // save one Frame YUV data
                if (fd > 0)
                {
                    if(_bWriteFile)
                    {
                        write(fd, stBufInfo.stFrameData.pVirAddr[0], stBufInfo.stFrameData.u16Height * stBufInfo.stFrameData.u32Stride[0] +
                            stBufInfo.stFrameData.u16Height * stBufInfo.stFrameData.u32Stride[1] /2);
                    }

                }

                ++u32GetFramesCount;
                printf("channelId[%u] u32GetFramesCount[%u]\n", stChnPort.u32ChnId, u32GetFramesCount);

                MI_SYS_ChnOutputPortPutBuf(stBufHandle);
            }
        }
    }

    if (fd > 0)
    {
        close(fd);
        fd = -1;
    }

    printf("exit record\n");
    return NULL;
}

void ST_DefaultConfig(ST_Config_S *pstConfig)
{
    pstConfig->s32UseOnvif     = 0;
    pstConfig->s32UseVdf    = 0;
    pstConfig->s32LoadIQ    = 0;
    pstConfig->s32HDRtype    = 0;
    pstConfig->enSensorType = ST_Sensor_Type_IMX291;
    pstConfig->enRotation = E_MI_SYS_ROTATE_180;
}

void ST_DefaultArgs(ST_Config_S *pstConfig)
{
    memset(pstConfig, 0, sizeof(ST_Config_S));
    ST_DefaultConfig(pstConfig);

    pstConfig->s32UseOnvif = 0;
    pstConfig->s32UseVdf = 0;
    pstConfig->s32LoadIQ = 0;
    pstConfig->enRotation = E_MI_SYS_ROTATE_180;
    pstConfig->en3dNrLevel = E_MI_VPE_3DNR_LEVEL1;
    pstConfig->s8SnrResIndex = -1;
}

void ST_HandleSig(MI_S32 signo)
{
    if(signo == SIGINT)
    {
        ST_INFO("catch Ctrl + C, exit normally\n");

        g_bExit = TRUE;
    }
}

static MI_S32 UVC_Init(void *uvc)
{
    return MI_SUCCESS;
}

static MI_S32 UVC_Deinit(void *uvc)
{
    return MI_SUCCESS;
}

static ST_UvcDev_t * Get_UVC_Device(void *uvc)
{
    ST_UVC_Device_t *pdev = (ST_UVC_Device_t*)uvc;

    for (int i = 0; i < g_UvcSrc->devnum;i++)
    {
        ST_UvcDev_t *dev = &g_UvcSrc->dev[i];
        if (!strcmp(dev->name, pdev->name))
        {
            return dev;
        }
    }
    return NULL;
}

static void UVC_ForceIdr(void *uvc)
{
    ST_UvcDev_t *dev = Get_UVC_Device(uvc);
    if ((dev->setting.fcc == V4L2_PIX_FMT_H264) || (dev->setting.fcc == V4L2_PIX_FMT_H265)) {
        g_stStreamAttr[dev->dev_index].bForceIdr = TRUE;
    }
}

static MI_S32 UVC_UP_FinishBuffer(void *uvc,ST_UVC_BufInfo_t *bufInfo)
{
    ST_UvcDev_t *dev = Get_UVC_Device(uvc);
    MI_S32 s32Ret = MI_SUCCESS;
    ST_Stream_Attr_T *pstStreamAttr = g_stStreamAttr;
    MI_SYS_BUF_HANDLE stBufHandle = 0;
    VENC_STREAMS_t * pUserptrStream = NULL;
    MI_U32 VencChn = pstStreamAttr[dev->dev_index].vencChn;

    if (!dev)
        return -1;

    switch(dev->setting.fcc) {
        case V4L2_PIX_FMT_YUYV:
            stBufHandle = bufInfo->b.handle;

            s32Ret = MI_SYS_ChnOutputPortPutBuf(stBufHandle);
            if(MI_SUCCESS!=s32Ret)
            {
                printf("%s Release Frame Failed\n", __func__);
                return s32Ret;
            }
            break;
        case V4L2_PIX_FMT_NV12:
            stBufHandle = bufInfo->b.handle;

            s32Ret = MI_SYS_ChnOutputPortPutBuf(stBufHandle);
            if(MI_SUCCESS!=s32Ret)
            {
                printf("%s Release Frame Failed\n", __func__);
                return s32Ret;
            }
            break;
        case V4L2_PIX_FMT_MJPEG:
        case V4L2_PIX_FMT_H264:
        case V4L2_PIX_FMT_H265:
            pUserptrStream = (VENC_STREAMS_t*)bufInfo->b.handle;

            s32Ret = MI_VENC_ReleaseStream(VencChn, &pUserptrStream->stStream);
            if (MI_SUCCESS != s32Ret)
            {
                printf("%s Release Frame Failed\n", __func__);
                return s32Ret;
            }

            pUserptrStream->used = false;
            break;
    }
    return MI_SUCCESS;
}

static MI_S32 UVC_UP_FillBuffer(void *uvc,ST_UVC_BufInfo_t *bufInfo)
{
    ST_UvcDev_t * dev = Get_UVC_Device(uvc);
    ST_Stream_Attr_T *pstStreamAttr = g_stStreamAttr;
    MI_S32 s32Ret = MI_SUCCESS;
    MI_SYS_BufInfo_t stBufInfo;
    MI_SYS_BUF_HANDLE stBufHandle;
    MI_SYS_ChnPort_t dstChnPort;
    VENC_STREAMS_t * pUserptrStream = NULL;
    MI_VENC_Stream_t * pstStream = NULL;
    MI_VENC_ChnStat_t stStat;
    MI_U32 VencChn = pstStreamAttr[dev->dev_index].vencChn;

    if (!dev)
        return -1;

    dstChnPort = dev->setting.dstChnPort;
    switch(dev->setting.fcc) {
        case V4L2_PIX_FMT_YUYV:
            memset(&stBufInfo, 0, sizeof(MI_SYS_BufInfo_t));
            memset(&stBufHandle, 0, sizeof(MI_SYS_BUF_HANDLE));

            s32Ret = MI_SYS_ChnOutputPortGetBuf(&dstChnPort, &stBufInfo, &stBufHandle);
            if(MI_SUCCESS!=s32Ret)
                return -EINVAL;

            bufInfo->b.start = (long unsigned int)stBufInfo.stFrameData.pVirAddr[0];
            bufInfo->length = stBufInfo.stFrameData.u16Height * stBufInfo.stFrameData.u32Stride[0];

            bufInfo->b.handle = (long unsigned int)stBufHandle;
            break;
        case V4L2_PIX_FMT_NV12:
            memset(&stBufInfo, 0, sizeof(MI_SYS_BufInfo_t));
            memset(&stBufHandle, 0, sizeof(MI_SYS_BUF_HANDLE));

            s32Ret = MI_SYS_ChnOutputPortGetBuf(&dstChnPort, &stBufInfo, &stBufHandle);
            if(MI_SUCCESS!=s32Ret)
                return -EINVAL;

            bufInfo->b.start = (long unsigned int)stBufInfo.stFrameData.pVirAddr[0];
            bufInfo->length = stBufInfo.stFrameData.u16Height
                     * (stBufInfo.stFrameData.u32Stride[0] + stBufInfo.stFrameData.u32Stride[1] / 2);
            bufInfo->b.handle = (long unsigned int)stBufHandle;
            break;
        case V4L2_PIX_FMT_MJPEG:
        case V4L2_PIX_FMT_H264:
        case V4L2_PIX_FMT_H265:
            for (int i = 0;i < g_maxbuf_cnt; i++)
            {
                if (!dev->res.pstuserptr_stream[i].used)
                {
                    pUserptrStream = &dev->res.pstuserptr_stream[i];
                    break;
                }
            }
            if (!pUserptrStream)
            {
                return -EINVAL;
            }
            pstStream = &pUserptrStream->stStream;
            memset(pstStream->pstPack, 0, sizeof(MI_VENC_Pack_t) * 4);

            s32Ret = MI_VENC_Query(VencChn, &stStat);
            if(s32Ret != MI_SUCCESS || stStat.u32CurPacks == 0)
                return -EINVAL;

            pstStream->u32PackCount = 1;//only need 1 packet

            s32Ret = MI_VENC_GetStream(VencChn, pstStream, 40);
            if (MI_SUCCESS != s32Ret)
                return -EINVAL;
            if (((dev->setting.fcc == V4L2_PIX_FMT_H264) && (pstStream->stH264Info.eRefType == E_MI_VENC_BASE_IDR)) ||
                ((dev->setting.fcc == V4L2_PIX_FMT_H265) && (pstStream->stH265Info.eRefType == E_MI_VENC_BASE_IDR))) {
                bufInfo->is_keyframe = true;
            }
            else {
                bufInfo->is_keyframe = false;
            }

            bufInfo->b.start = (long unsigned int)pstStream->pstPack[0].pu8Addr;
            bufInfo->length = pstStream->pstPack[0].u32Len;

            bufInfo->b.handle = (long unsigned int)pUserptrStream;
            pUserptrStream->used = true;

            if (pstStreamAttr[dev->dev_index].bForceIdr)
            {
                pstStreamAttr[dev->dev_index].bForceIdr = FALSE;
                MI_VENC_RequestIdr(VencChn, TRUE);
            }
            break;
        default:
            return -EINVAL;
    }
    return MI_SUCCESS;
}

static MI_S32 UVC_MM_FillBuffer(void *uvc,ST_UVC_BufInfo_t *bufInfo)
{
    ST_UvcDev_t * dev = Get_UVC_Device(uvc);
    ST_Stream_Attr_T *pstStreamAttr = g_stStreamAttr;
    MI_S32 s32Ret = MI_SUCCESS;
    MI_U32 u32Size, i;
    MI_SYS_BufInfo_t stBufInfo;
    MI_SYS_BUF_HANDLE stBufHandle;
    MI_SYS_ChnPort_t dstChnPort;
    MI_VENC_Stream_t stStream;
    MI_VENC_Pack_t stPack[4];
    MI_VENC_ChnStat_t stStat;

    MI_U8 *u8CopyData = (MI_U8 *)bufInfo->b.buf;
    MI_U32 *pu32length = (MI_U32 *)&bufInfo->length;
    MI_U32 VencChn = pstStreamAttr[dev->dev_index].vencChn;

    if (!dev)
        return -1;

    dstChnPort = dev->setting.dstChnPort;

    switch(dev->setting.fcc) {
        case V4L2_PIX_FMT_YUYV:
            memset(&stBufInfo, 0, sizeof(MI_SYS_BufInfo_t));
            memset(&stBufHandle, 0, sizeof(MI_SYS_BUF_HANDLE));

            s32Ret = MI_SYS_ChnOutputPortGetBuf(&dstChnPort, &stBufInfo, &stBufHandle);
            if(MI_SUCCESS!=s32Ret)
            {
                return -EINVAL;
            }

            *pu32length = stBufInfo.stFrameData.u16Height * stBufInfo.stFrameData.u32Stride[0];
            memcpy(u8CopyData, stBufInfo.stFrameData.pVirAddr[0], *pu32length);

            s32Ret = MI_SYS_ChnOutputPortPutBuf(stBufHandle);
            if(MI_SUCCESS!=s32Ret)
                printf("%s Release Frame Failed\n", __func__);

            break;
        case V4L2_PIX_FMT_NV12:
            memset(&stBufInfo, 0, sizeof(MI_SYS_BufInfo_t));
            memset(&stBufHandle, 0, sizeof(MI_SYS_BUF_HANDLE));

            s32Ret = MI_SYS_ChnOutputPortGetBuf(&dstChnPort, &stBufInfo, &stBufHandle);
            if(MI_SUCCESS!=s32Ret)
                return -EINVAL;

            *pu32length = stBufInfo.stFrameData.u16Height
                    * (stBufInfo.stFrameData.u32Stride[0] + stBufInfo.stFrameData.u32Stride[1] / 2);


            memcpy(u8CopyData, stBufInfo.stFrameData.pVirAddr[0],
                            stBufInfo.stFrameData.u16Height * stBufInfo.stFrameData.u32Stride[0]);
            u8CopyData += stBufInfo.stFrameData.u16Height * stBufInfo.stFrameData.u32Stride[0];
            memcpy(u8CopyData, stBufInfo.stFrameData.pVirAddr[1],
                            stBufInfo.stFrameData.u16Height * stBufInfo.stFrameData.u32Stride[1]/2);
            s32Ret = MI_SYS_ChnOutputPortPutBuf(stBufHandle);
            if(MI_SUCCESS!=s32Ret)
                printf("%s Release Frame Failed\n", __func__);
            break;
        case V4L2_PIX_FMT_MJPEG:
        case V4L2_PIX_FMT_H264:
        case V4L2_PIX_FMT_H265:
            memset(&stStream, 0, sizeof(MI_VENC_Stream_t));
            memset(&stPack, 0, sizeof(MI_VENC_Pack_t) * 4);
            stStream.pstPack = stPack;

            s32Ret = MI_VENC_Query(VencChn, &stStat);
            if(s32Ret != MI_SUCCESS || stStat.u32CurPacks == 0)
                return -EINVAL;

            stStream.u32PackCount = stStat.u32CurPacks;

            s32Ret = MI_VENC_GetStream(VencChn, &stStream, 40);
            if (MI_SUCCESS != s32Ret)
                return -EINVAL;
            if (((dev->setting.fcc == V4L2_PIX_FMT_H264) && (stStream.stH264Info.eRefType == E_MI_VENC_BASE_IDR)) ||
                ((dev->setting.fcc == V4L2_PIX_FMT_H265) && (stStream.stH265Info.eRefType == E_MI_VENC_BASE_IDR))) {
                bufInfo->is_keyframe = true;
            }
            else {
                bufInfo->is_keyframe = false;
            }

            for(i = 0;i < stStat.u32CurPacks; i++)
            {
                u32Size = stStream.pstPack[i].u32Len;
                memcpy(u8CopyData,stStream.pstPack[i].pu8Addr, u32Size);
                u8CopyData += u32Size;
            }
            *pu32length = u8CopyData - (MI_U8 *)bufInfo->b.buf;


            bufInfo->is_tail = true;//default is frameEnd
            s32Ret = MI_VENC_ReleaseStream(VencChn, &stStream);
            if (MI_SUCCESS != s32Ret)
                printf("%s Release Frame Failed\n", __func__);

            if (pstStreamAttr[dev->dev_index].bForceIdr)
            {
                pstStreamAttr[dev->dev_index].bForceIdr = FALSE;
                MI_VENC_RequestIdr(VencChn, TRUE);
            }
            break;
        default:
            printf("unknown format %d\n", dev->setting.fcc);
            return -EINVAL;
    }

    if(gbDumpData && gpDumpFile)
    {
        fwrite(bufInfo->b.buf, *pu32length, 1,  gpDumpFile);
    }

    return MI_SUCCESS;
}

ST_Rect_T rgnRect;
MI_RGN_HANDLE rgnHandle;
pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;
void * detection_manager;

pthread_t     g_detectThread;
pthread_t     g_alertThread;
MI_S32        s32DivpFd;
MI_S32        m_s32MaxFd;
MI_S32        flag = 1;


void *DetectThread(void *argc)
{
    printf("detect thread start\n");
    int type = *(int *)argc;
    fd_set read_fds;
    struct timeval tv;
    MI_S32 s32Ret = 0;
    MI_SYS_ChnPort_t stChnOutputPort;
    MI_SYS_BufInfo_t        stBufInfo;
    MI_SYS_BufInfo_t        stBufInfo1;
    MI_SYS_BUF_HANDLE       stBufHandle;
    tv.tv_sec = 0;
    tv.tv_usec = 100 * 1000;
    int count = 0;
	printf("**************************************** type=%d ************************************** \n",type);
    while(1)
    {
        /*if(g_bStartCapture == FALSE)
        {
            sleep(1);
            continue;
        }*/
        FD_ZERO(&read_fds);
        FD_SET(s32DivpFd, &read_fds);
        s32Ret = select(m_s32MaxFd + 1, &read_fds, NULL, NULL, &tv);
        if (s32Ret < 0)
        {
            printf("select failed\n");
        }
        else if (0 == s32Ret)
        {
            //printf("select timeout\n");
        }
        else
        {
         if(FD_ISSET(s32DivpFd, &read_fds))
            {
                stChnOutputPort.eModId      = E_MI_MODULE_ID_DIVP;
                stChnOutputPort.u32DevId    = 0;
                stChnOutputPort.u32ChnId    = 0;
                stChnOutputPort.u32PortId   = 0;
                memset(&stBufInfo, 0x0, sizeof(MI_SYS_BufInfo_t));
                s32Ret = MI_SYS_ChnOutputPortGetBuf(&stChnOutputPort, &stBufInfo, &stBufHandle);
				//printf("MI_SYS_ChnOutputPortGetBuf \n");
                if (MI_SUCCESS != s32Ret)
                {
                    printf("get divp buffer fail,ret:%x\n",s32Ret);
                    continue;
                }
                if(type==0)
                    doDetectFace(detection_manager, &stBufInfo);
                else if(type ==1)
                    doDetectPerson(detection_manager, &stBufInfo);
                else if(type ==2)
                    doDetect(detection_manager, &stBufInfo);
                MI_SYS_ChnOutputPortPutBuf(stBufHandle);    
            }
        }
   }
	
}

void MySystemDelay(unsigned int  msec)
{
    struct timeval tv;
    tv.tv_sec = msec/1000;
    tv.tv_usec = msec%1000 * 1000;
    select(0, NULL, NULL, NULL, &tv);
}





MI_S32 RgnInit()
{
    ExecFunc(ST_OSD_Init(), MI_SUCCESS);
    MI_RGN_Attr_t stRgnAttr;
    memset(&stRgnAttr, 0, sizeof(MI_RGN_Attr_t));
    stRgnAttr.eType = E_MI_RGN_TYPE_OSD;
    stRgnAttr.stOsdInitParam.ePixelFmt = E_MI_RGN_PIXEL_FORMAT_I4;

    stRgnAttr.stOsdInitParam.stSize.u32Width = DISP_WIDTH;
    stRgnAttr.stOsdInitParam.stSize.u32Height = DISP_HEIGHT;
    ExecFunc(ST_OSD_Create(rgnHandle, &stRgnAttr),MI_SUCCESS);
    MI_RGN_ChnPort_t stRgnChnPort;
    MI_RGN_ChnPortParam_t stRgnChnPortParam;
    memset(&stRgnChnPortParam, 0, sizeof(MI_RGN_ChnPortParam_t));
    memset(&stRgnChnPort, 0, sizeof(MI_RGN_ChnPort_t));
    stRgnChnPort.eModId = E_MI_RGN_MODID_VPE;
    stRgnChnPort.s32DevId = 0;
    stRgnChnPort.s32ChnId = 0;
    stRgnChnPort.s32OutputPortId = 0;

    stRgnChnPortParam.bShow = TRUE;
    stRgnChnPortParam.stPoint.u32X = rgnRect.u32X;
    stRgnChnPortParam.stPoint.u32Y = rgnRect.u32Y;
    stRgnChnPortParam.unPara.stOsdChnPort.u32Layer = 99;
    ExecFunc(MI_RGN_AttachToChn(rgnHandle, &stRgnChnPort, &stRgnChnPortParam), MI_SUCCESS);
    //memcpy(&g_stRgnInfo[rgnHandle].stRgnChnPort, &stRgnChnPort, sizeof(MI_RGN_ChnPort_t));

    //printf("handle:%d, virtAddr:%X, phyAddr:%X\n", rgnHandle, g_stRgnInfo[rgnHandle].stCanvasInfo.virtAddr,
			//g_stRgnInfo[rgnHandle].stCanvasInfo.phyAddr);

    return MI_SUCCESS; 
}


MI_U32 StreamInit()
{
    MI_S32 s32Ret = MI_SUCCESS;
    MI_VPE_PortMode_t stVpeMode;
    MI_DIVP_ChnAttr_t stDivpChnAttr;
    MI_DIVP_OutputPortAttr_t stDivpPortAttr;
    MI_SYS_ChnPort_t stChnOutputPort;
    ST_Sys_BindInfo_T stBindInfo;
    memset(&stDivpChnAttr, 0, sizeof(MI_DIVP_ChnAttr_t));
    memset(&stVpeMode, 0, sizeof(MI_VPE_PortMode_t));
    memset(&stDivpPortAttr, 0, sizeof(MI_DIVP_OutputPortAttr_t));
    memset(&stBindInfo, 0, sizeof(ST_Sys_BindInfo_T));
    memset(&stChnOutputPort, 0, sizeof(MI_SYS_ChnPort_t));
    
    ST_VPE_PortInfo_T stVpePortInfo;
    memset(&stVpePortInfo,0,sizeof(ST_VPE_PortInfo_T));
    stVpePortInfo.DepVpeChannel   = 0;
    stVpePortInfo.eCompressMode = E_MI_SYS_COMPRESS_MODE_NONE;
    printf("dla start\n");
    stVpePortInfo.ePixelFormat = E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420;
    stVpePortInfo.u16OutputWidth = MODEL_WIDTH;
    stVpePortInfo.u16OutputHeight = MODEL_HEIGHT;
    printf("vpe chn:%d\n",stVpePortInfo.DepVpeChannel);
    ST_Vpe_StartPort(1, &stVpePortInfo);
    ExecFunc(MI_VPE_GetPortMode(0, 1, &stVpeMode), MI_VPE_OK);

    // init divp channel
    stDivpChnAttr.bHorMirror = FALSE;
    stDivpChnAttr.bVerMirror = FALSE;
    stDivpChnAttr.eDiType = E_MI_DIVP_DI_TYPE_OFF;
    stDivpChnAttr.eRotateType = E_MI_SYS_ROTATE_NONE;
    stDivpChnAttr.eTnrLevel = E_MI_DIVP_TNR_LEVEL_OFF;
    stDivpChnAttr.stCropRect.u16X = 0;
    stDivpChnAttr.stCropRect.u16Y = 0;
    stDivpChnAttr.u32MaxWidth = stVpeMode.u16Width;
    stDivpChnAttr.u32MaxHeight = stVpeMode.u16Height;
    ExecFunc(MI_DIVP_CreateChn(0, &stDivpChnAttr), MI_SUCCESS);
    stDivpPortAttr.u32Width = MODEL_WIDTH;
    stDivpPortAttr.u32Height = MODEL_HEIGHT;
    stDivpPortAttr.eCompMode = E_MI_SYS_COMPRESS_MODE_NONE;
    stDivpPortAttr.ePixelFormat = E_MI_SYS_PIXEL_FRAME_ARGB8888;
    ExecFunc(MI_DIVP_SetOutputPortAttr(0, &stDivpPortAttr), MI_SUCCESS);
    ExecFunc(MI_DIVP_StartChn(0), MI_SUCCESS);

    stBindInfo.stSrcChnPort.eModId = E_MI_MODULE_ID_VPE;
    stBindInfo.stSrcChnPort.u32DevId = 0;
    stBindInfo.stSrcChnPort.u32ChnId = 0;
    stBindInfo.stSrcChnPort.u32PortId = 1;
    stBindInfo.stDstChnPort.eModId = E_MI_MODULE_ID_DIVP;
    stBindInfo.stDstChnPort.u32DevId = 0;
    stBindInfo.eBindType = E_MI_SYS_BIND_TYPE_FRAME_BASE;
    stBindInfo.stDstChnPort.u32ChnId = 0;
    stBindInfo.stDstChnPort.u32PortId = 0;
    stBindInfo.u32SrcFrmrate = 30;
    stBindInfo.u32DstFrmrate = 15;
    ExecFunc(MI_SYS_SetChnOutputPortDepth(&stBindInfo.stDstChnPort, 3, 4), MI_SUCCESS);
    STCHECKRESULT(ST_Sys_Bind(&stBindInfo));


    //init divp channel-1
    stDivpChnAttr.bHorMirror = FALSE;
    stDivpChnAttr.bVerMirror = FALSE;
    stDivpChnAttr.eDiType = E_MI_DIVP_DI_TYPE_OFF;
    stDivpChnAttr.eRotateType = E_MI_SYS_ROTATE_NONE;
    stDivpChnAttr.eTnrLevel = E_MI_DIVP_TNR_LEVEL_OFF;
    stDivpChnAttr.stCropRect.u16X = 0;
    stDivpChnAttr.stCropRect.u16Y = 0;
    stDivpChnAttr.u32MaxWidth = DIVP_INPUT_WIDTH;
    stDivpChnAttr.u32MaxHeight = DIVP_INPUT_WIDTH;
    ExecFunc(MI_DIVP_CreateChn(1, &stDivpChnAttr), MI_SUCCESS);
    stDivpPortAttr.u32Width = MODEL_WIDTH;
    stDivpPortAttr.u32Height = MODEL_HEIGHT;
    stDivpPortAttr.eCompMode = E_MI_SYS_COMPRESS_MODE_NONE;
    stDivpPortAttr.ePixelFormat = E_MI_SYS_PIXEL_FRAME_ARGB8888;
    ExecFunc(MI_DIVP_SetOutputPortAttr(1, &stDivpPortAttr), MI_SUCCESS);
    ExecFunc(MI_DIVP_StartChn(1), MI_SUCCESS);

    stBindInfo.stSrcChnPort.eModId = E_MI_MODULE_ID_VPE;
    stBindInfo.stSrcChnPort.u32DevId = 0;
    stBindInfo.stSrcChnPort.u32ChnId = 0;
    stBindInfo.stSrcChnPort.u32PortId = 0;
    stBindInfo.stDstChnPort.eModId = E_MI_MODULE_ID_DIVP;
    stBindInfo.stDstChnPort.u32DevId = 0;
    stBindInfo.eBindType = E_MI_SYS_BIND_TYPE_FRAME_BASE;
    stBindInfo.stDstChnPort.u32ChnId = 1;
    stBindInfo.stDstChnPort.u32PortId = 0;
    stBindInfo.u32SrcFrmrate = 30;
    stBindInfo.u32DstFrmrate = 15;
    ExecFunc(MI_SYS_SetChnOutputPortDepth(&stBindInfo.stDstChnPort, 3, 4), MI_SUCCESS);



    //get fd
    stChnOutputPort.eModId      = E_MI_MODULE_ID_DIVP;
    stChnOutputPort.u32DevId    = 0;
    stChnOutputPort.u32ChnId    = 0;
    stChnOutputPort.u32PortId   = 0;
    s32Ret = MI_SYS_GetFd(&stChnOutputPort, &s32DivpFd);
    if (s32Ret < 0)
    {
        printf("divp ch: %d, get fd. err\n", stChnOutputPort.u32ChnId);
        return -1;
    }

    m_s32MaxFd = MAX(m_s32MaxFd, s32DivpFd);

    return MI_SUCCESS;
}

typedef struct
{
    MI_RGN_HANDLE hHandle;
    MI_RGN_CanvasInfo_t stCanvasInfo;
    MI_RGN_PixelFormat_e ePixelFmt;
} ST_RGN_Info_T;

extern ST_RGN_Info_T g_stRgnInfo[1024];
void detect_callback(std::vector<Box_t> results)
{
    #define VPE_CROP_STEP_THRESHOLD 128
    #define VPE_CROP_STEPS 30
    #define NOBODY_CHEK_COUNTS 100
    #define RECT_BORDER_WIDTH 8
    MI_U32 BaseTop = 256;
    MI_U32 BaseLeft = 16;
    MI_U32 BaseH = 2;

    MI_RGN_CanvasInfo_t *pstCanvasInfo;
    ST_OSD_GetCanvasInfo(rgnHandle, &pstCanvasInfo);
    ST_OSD_Clear(rgnHandle, &rgnRect);

     int i = 0;
    ST_Rect_T rgnRect = {0,0,DISP_WIDTH,DISP_HEIGHT};

    //pthread_mutex_lock(&lock);
    //pthread_mutex_unlock(&lock);
    

    for(i = 0; i < results.size(); i++)
    {
        rgnRect.u32X = results[i].x;
        rgnRect.u32Y = results[i].y;
        rgnRect.u16PicW = results[i].width;
        rgnRect.u16PicH = results[i].height;
		//printf("--->3206 sorce: %f \n", results[i].score);
        // printf("results[i].class_id = %d ,--->3206 sorce: %f ,results[i].width = %d , results[i].height = %d\n", results[i].class_id,results[i].score,results[i].width, results[i].height);
        // printf("results[i].x = %d , results[i].y = %d ,rgnHandle = % d\n",results[i].x,results[i].y,rgnHandle);
		if(results[i].class_id == 0)
			ST_OSD_DrawRect(rgnHandle, rgnRect, RECT_BORDER_WIDTH, 8);   //brown
		else if(results[i].class_id == 1)
			ST_OSD_DrawRect(rgnHandle, rgnRect, RECT_BORDER_WIDTH, 2);  // green
		else if(results[i].class_id == 2)
			ST_OSD_DrawRect(rgnHandle, rgnRect, RECT_BORDER_WIDTH, 3);   // blue
		else if(results[i].class_id == 3)
			ST_OSD_DrawRect(rgnHandle, rgnRect, RECT_BORDER_WIDTH, 4);	 // yellow
		else if(results[i].class_id == 4)
			ST_OSD_DrawRect(rgnHandle, rgnRect, RECT_BORDER_WIDTH, 1);	 // red
		else if(results[i].class_id == 5)
			ST_OSD_DrawRect(rgnHandle, rgnRect, RECT_BORDER_WIDTH, 10);	 // voilet
    }   

    MI_S32 s32Ret = MI_SUCCESS;
    
    MI_SYS_WindowRect_t stSrcCrop;
    MI_VENC_CropCfg_t vencCrop;
    vencCrop.bEnable = False;

        
    if(results.size()==1)
    {
        //stSrcCrop.u16X=results[i].x;
        //stSrcCrop.u16Y=results[i].y;
        //stSrcCrop.u16Width=results[i].width;
        //stSrcCrop.u16Height=results[i].height;
        rgnRect.u32X = (rgnRect.u32X < 0) ? 0 : rgnRect.u32X;
        rgnRect.u32Y = (rgnRect.u32Y < 0) ? 0 : rgnRect.u32Y;
        rgnRect.u16PicW = ((rgnRect.u32X + rgnRect.u16PicW) > g_stRgnInfo[rgnHandle].stCanvasInfo.stSize.u32Width) ?
                            (g_stRgnInfo[rgnHandle].stCanvasInfo.stSize.u32Width - rgnRect.u32X) : rgnRect.u16PicW;
        rgnRect.u16PicH = ((rgnRect.u32Y + rgnRect.u16PicH) > g_stRgnInfo[rgnHandle].stCanvasInfo.stSize.u32Height) ?
                            (g_stRgnInfo[rgnHandle].stCanvasInfo.stSize.u32Height - rgnRect.u32Y) : rgnRect.u16PicH;



        vencCrop.stRect.u32Left=rgnRect.u32X - (rgnRect.u32X % BaseTop);
        vencCrop.stRect.u32Top=rgnRect.u32Y - (rgnRect.u32Y % BaseLeft);
        vencCrop.stRect.u32Width=rgnRect.u16PicW - (rgnRect.u16PicW%BaseH);
        vencCrop.stRect.u32Height=rgnRect.u16PicH - (rgnRect.u16PicH%BaseH);


        if (vencCrop.stRect.u32Width + vencCrop.stRect.u32Left > 3840 || vencCrop.stRect.u32Width <= 0 )
        {
            vencCrop.stRect.u32Width = 3840 - vencCrop.stRect.u32Left;
        }
        if (vencCrop.stRect.u32Height + vencCrop.stRect.u32Top > 2160 || vencCrop.stRect.u32Width <= 0 )
        {
            vencCrop.stRect.u32Height = 2160 - vencCrop.stRect.u32Top;
        }

        std::cout<<vencCrop.stRect.u32Left<<std::endl;
        std::cout<<vencCrop.stRect.u32Top<<std::endl;
        std::cout<<vencCrop.stRect.u32Width<<std::endl;
        std::cout<<vencCrop.stRect.u32Height<<std::endl;
        std::cout<<rgnRect.u16PicW<<std::endl;
        std::cout<<rgnRect.u16PicH<<std::endl;
        vencCrop.bEnable = True;
        MI_VENC_SetCrop(0,&vencCrop);
        // MI_VPE_DisablePort(0,0);
        // MI_VPE_SetPortCrop(0,0, &stSrcCrop);
        // MI_VPE_EnablePort(0,0);
        std::cout<<"++++++++++++++++++1++++++++++++"<<std::endl;

    }
    else
    {
        // stSrcCrop.u16X=0;
        // stSrcCrop.u16Y=0;
        // stSrcCrop.u16Width=DISP_WIDTH;
        // stSrcCrop.u16Height=DISP_HEIGHT;
        vencCrop.stRect.u32Left=0;
        vencCrop.stRect.u32Top=0;
        vencCrop.stRect.u32Width=3840;
        vencCrop.stRect.u32Height=2160;
        vencCrop.bEnable = True;
        MI_VENC_SetCrop(0,&vencCrop);
        std::cout<<"*****************2****"<<std::endl;
    }
 
    
    ST_OSD_Update(rgnHandle);

}

MI_S32 create_device_callback(MI_IPU_DevAttr_t* stDevAttr, char *pFirmwarePath)
{
	return MI_IPU_CreateDevice(stDevAttr, NULL, pFirmwarePath, 0);
}

void destory_device_callback()
{
	MI_IPU_DestroyDevice();
}

DetectionInfo_t face_detection_info = 
{
    "/config/dla/ipu_firmware.bin",
    "syfd360102_fixed_argb.sim_sgsimg.img",
    0.5, //threshold
    {DISP_WIDTH, DISP_HEIGHT},
	create_device_callback,
    detect_callback,
	destory_device_callback
};



DetectionInfo_t person_detection_info = 
{
    "/config/dla/ipu_firmware.bin",
    "sypd5480302_fixed.sim_sgsimg.img",
    0.6, //threshold
    {DISP_WIDTH, DISP_HEIGHT},
    create_device_callback,
    detect_callback,
	destory_device_callback
};


DetectionInfo_t pcn_detection_info = 
{
    "/config/dla/ipu_firmware.bin",
    "sypcnd_argb_fixed.sim_sgsimg.img",
    0.6, //threshold
    {DISP_WIDTH, DISP_HEIGHT},
	create_device_callback,
    detect_callback,
	destory_device_callback
};

DetectionInfo_t pcd_detection_info = 
{
    "/config/dla/ipu_firmware.bin",
    "sypcdd_argb_fixed.sim_sgsimg.img",
    0.5, //threshold
    {DISP_WIDTH, DISP_HEIGHT},
	create_device_callback,
    detect_callback,
	destory_device_callback
};



static MI_S32 UVC_StartCapture(void *uvc,Stream_Params_t format)
{
    ST_UvcDev_t *dev = Get_UVC_Device(uvc);
    ST_Stream_Attr_T *pstStreamAttr;
    MI_SYS_ChnPort_t *dstChnPort;
    ST_Sys_BindInfo_T stBindInfo[2];
    MI_U32 u32Width, u32Height;
	MI_RGN_ChnPort_t stRgnChnPort;

    /************************************************
    Step0:  Initial general param
    *************************************************/
#if 1
	stRgnChnPort.eModId = E_MI_RGN_MODID_VPE;
    stRgnChnPort.s32DevId = 0;
    stRgnChnPort.s32ChnId = 0;
    stRgnChnPort.s32OutputPortId = 0;

	// ExecFunc(MI_RGN_DetachFromChn(rgnHandle,&stRgnChnPort), MI_SUCCESS);
	// ExecFunc(ST_OSD_Destroy(rgnHandle),MI_SUCCESS);
	// flag = 0;
	// pthread_join(g_detectThread,NULL);

	// STCHECKRESULT(stopDetect(detection_manager));
	// STCHECKRESULT(putDetectionManager(detection_manager));

	//start face 
	STCHECKRESULT(getDetectionManager(&detection_manager));
    //int type=2;
    //STCHECKRESULT(initDetection(detection_manager, &pcn_detection_info));
    //int type=1;
    //STCHECKRESULT(initDetection(detection_manager, &person_detection_info));
    int type_face=1;
	person_detection_info.disp_size.width = format.width;
	person_detection_info.disp_size.height = format.height;
    STCHECKRESULT(initDetection(detection_manager, &person_detection_info));
    STCHECKRESULT(startDetect(detection_manager));
    STCHECKRESULT(StreamInit());
    pthread_create(&g_detectThread, NULL, DetectThread, &type_face);



	
    ExecFunc(ST_OSD_Init(), MI_SUCCESS);
	MI_RGN_Attr_t stRgnAttr;
    memset(&stRgnAttr, 0, sizeof(MI_RGN_Attr_t));
    stRgnAttr.eType = E_MI_RGN_TYPE_OSD;
    stRgnAttr.stOsdInitParam.ePixelFmt = E_MI_RGN_PIXEL_FORMAT_I4;

    stRgnAttr.stOsdInitParam.stSize.u32Width = format.width;
    stRgnAttr.stOsdInitParam.stSize.u32Height = format.height;
    ExecFunc(ST_OSD_Create(rgnHandle, &stRgnAttr),MI_SUCCESS);
    MI_RGN_ChnPortParam_t stRgnChnPortParam;
    memset(&stRgnChnPortParam, 0, sizeof(MI_RGN_ChnPortParam_t));
    memset(&stRgnChnPort, 0, sizeof(MI_RGN_ChnPort_t));
    stRgnChnPort.eModId = E_MI_RGN_MODID_VPE;
    stRgnChnPort.s32DevId = 0;
    stRgnChnPort.s32ChnId = 0;
    stRgnChnPort.s32OutputPortId = 0;

    stRgnChnPortParam.bShow = TRUE;
    stRgnChnPortParam.stPoint.u32X = rgnRect.u32X;
    stRgnChnPortParam.stPoint.u32Y = rgnRect.u32Y;
    stRgnChnPortParam.unPara.stOsdChnPort.u32Layer = 99;
    ExecFunc(MI_RGN_AttachToChn(rgnHandle, &stRgnChnPort, &stRgnChnPortParam), MI_SUCCESS);

	printf("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++width = %d   height = %d \n",format.width,format.height);
	
#endif
	printf("+++++++++++++++++++++++++++++++++++++++++++++++++++++++=++++++++zhe shi xuanze fen bianlv \n");
    if (!dev)
        return -1;

    if(gbSnrPowerOnWhenPreview)
    {
        STCHECKRESULT(ST_VideoModuleInit(&g_stConfig));
    }

    memset(&dev->setting, 0x00, sizeof(dev->setting));
    dev->setting.fcc = format.fcc;
    dev->setting.u32Width = format.width;
    dev->setting.u32Height = format.height;
    dev->setting.u32FrameRate = format.frameRate;

    dstChnPort = &dev->setting.dstChnPort;
    pstStreamAttr = g_stStreamAttr;
    u32Width = dev->setting.u32Width;
    u32Height = dev->setting.u32Height;

    /************************************************
    Step1:  start VPE port
    *************************************************/
    MI_U32 VpeChn = pstStreamAttr[dev->dev_index].u32InputChn;
    MI_U32 VpePortId = pstStreamAttr[dev->dev_index].u32InputPort;
    ST_VPE_PortInfo_T stVpePortInfo;

    stVpePortInfo.DepVpeChannel   = VpeChn;
    pstStreamAttr[dev->dev_index].bEnable = TRUE;
    pstStreamAttr[dev->dev_index].bUserVenc = FALSE;
    pstStreamAttr[dev->dev_index].u32Width = u32Width;
    pstStreamAttr[dev->dev_index].u32Height = u32Height;
    
    //pstStreamAttr[dev->dev_index].enInput = ST_Sys_Input_DIVP;

    if(pstStreamAttr[dev->dev_index].enInput == ST_Sys_Input_VPE)
    {
        stVpePortInfo.u16OutputWidth  = u32Width;
        stVpePortInfo.u16OutputHeight = u32Height;
    }
    else if(pstStreamAttr[dev->dev_index].enInput == ST_Sys_Input_DIVP)
    {
        stVpePortInfo.u16OutputWidth  = DIVP_INPUT_WIDTH;
        stVpePortInfo.u16OutputHeight = DIVP_INPUT_HEIGHT;
    }

    stVpePortInfo.eCompressMode = E_MI_SYS_COMPRESS_MODE_NONE;

    printf("--------------------dev->index=%d-----------------\n\n",dev->dev_index);
    printf("---------------------------pstStreamAttr[dev->dev_index].enInput == ST_Sys_Input_DIVP? = %d-----------------------\n\n",pstStreamAttr[dev->dev_index].enInput == ST_Sys_Input_DIVP);

    /************************************************
    Step2: Init Divp Option
    ************************************************/
    MI_DIVP_CHN DivpChn = pstStreamAttr[dev->dev_index].divpChn;
    printf("DivpChn=%d\n",DivpChn);
    printf("==========dev_index = %d=============\n",dev->dev_index);
    printf("==========divpChn = %d=============\n",DivpChn);

    MI_DIVP_ChnAttr_t stDivpChnAttr;
    MI_DIVP_OutputPortAttr_t stDivpOutputPortAttr;
    memset(&stDivpChnAttr,0,sizeof(MI_DIVP_ChnAttr_t));
    memset(&stDivpOutputPortAttr,0,sizeof(MI_DIVP_OutputPortAttr_t));

    stDivpChnAttr.u32MaxWidth = DIVP_INPUT_WIDTH;
    stDivpChnAttr.u32MaxHeight = DIVP_INPUT_HEIGHT;
    stDivpChnAttr.stCropRect.u16X = 0;
    stDivpChnAttr.stCropRect.u16Y = 0;
    stDivpChnAttr.stCropRect.u16Width = DIVP_INPUT_WIDTH;
    stDivpChnAttr.stCropRect.u16Height = DIVP_INPUT_HEIGHT;
    stDivpChnAttr.bHorMirror = false;
    stDivpChnAttr.bVerMirror = false;
    stDivpChnAttr.eDiType = E_MI_DIVP_DI_TYPE_OFF;
    stDivpChnAttr.eRotateType = E_MI_SYS_ROTATE_NONE;
    stDivpChnAttr.eTnrLevel = E_MI_DIVP_TNR_LEVEL_OFF;

    stDivpOutputPortAttr.eCompMode = E_MI_SYS_COMPRESS_MODE_NONE;
    //stDivpOutputPortAttr.ePixelFormat = E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420;
    stDivpOutputPortAttr.u32Width = u32Width;
    stDivpOutputPortAttr.u32Height = u32Height;

    /************************************************
    Step3: Init Venc Option
    ************************************************/
    MI_U32 VencChn = pstStreamAttr[dev->dev_index].vencChn;
    MI_U32 u32FrameRate = 0;
    MI_VENC_ChnAttr_t stChnAttr;
    MI_U32  u32VenBitRate =0;
    MI_U32  u32VenQfactor =80;
    MI_U32 VencDev = 0;
    MI_VENC_InputSourceConfig_t stVenInSrc;
    bool bByFrame = true;
    bool bHaftRing = true;

    memset(&stChnAttr, 0, sizeof(MI_VENC_ChnAttr_t));
    memset(&stVenInSrc, 0, sizeof(MI_VENC_InputSourceConfig_t));

    /************************************************
    Step4: Init Bind Option
    ************************************************/
    //pstStreamAttr[dev->dev_index].enInput = ST_Sys_Input_DIVP;
    if(pstStreamAttr[dev->dev_index].enInput == ST_Sys_Input_VPE)
    {
        memset(&stBindInfo[0], 0x0, sizeof(ST_Sys_BindInfo_T));
        stBindInfo[0].stSrcChnPort.eModId = E_MI_MODULE_ID_VPE;
        stBindInfo[0].stSrcChnPort.u32DevId = 0;
        stBindInfo[0].stSrcChnPort.u32ChnId = VpeChn;
        stBindInfo[0].stSrcChnPort.u32PortId = VpePortId;

        stBindInfo[0].stDstChnPort.eModId = E_MI_MODULE_ID_VENC;
        stBindInfo[0].stDstChnPort.u32ChnId = VencChn;
        stBindInfo[0].stDstChnPort.u32PortId = 0;

        stBindInfo[0].u32SrcFrmrate = dev->setting.u32FrameRate;
        stBindInfo[0].u32DstFrmrate = dev->setting.u32FrameRate;
        u32FrameRate =  dev->setting.u32FrameRate;
    }
    else if(pstStreamAttr[dev->dev_index].enInput == ST_Sys_Input_DIVP)
    {
        // stDivpChnAttr.bHorMirror = FALSE;
        // stDivpChnAttr.bVerMirror = FALSE;
        // stDivpChnAttr.eDiType = E_MI_DIVP_DI_TYPE_OFF;
        // stDivpChnAttr.eRotateType = E_MI_SYS_ROTATE_NONE;
        // stDivpChnAttr.eTnrLevel = E_MI_DIVP_TNR_LEVEL_OFF;
        // stDivpChnAttr.stCropRect.u16X = 0;
        // stDivpChnAttr.stCropRect.u16Y = 0;
        // stDivpChnAttr.u32MaxWidth = stVpeMode.u16Width;
        // stDivpChnAttr.u32MaxHeight = stVpeMode.u16Height;
        // ExecFunc(MI_DIVP_CreateChn(1, &stDivpChnAttr), MI_SUCCESS);
        // stDivpPortAttr.u32Width = MODEL_WIDTH;
        // stDivpPortAttr.u32Height = MODEL_HEIGHT;
        // stDivpPortAttr.eCompMode = E_MI_SYS_COMPRESS_MODE_NONE;
        // stDivpPortAttr.ePixelFormat = E_MI_SYS_PIXEL_FRAME_ARGB8888;
        // ExecFunc(MI_DIVP_SetOutputPortAttr(0, &stDivpPortAttr), MI_SUCCESS);
        // ExecFunc(MI_DIVP_StartChn(0), MI_SUCCESS);

        memset(&stBindInfo[0], 0x0, sizeof(ST_Sys_BindInfo_T));
        stBindInfo[0].stSrcChnPort.eModId = E_MI_MODULE_ID_VPE;
        stBindInfo[0].stSrcChnPort.u32DevId = 0;
        stBindInfo[0].stSrcChnPort.u32ChnId = VpeChn;
        stBindInfo[0].stSrcChnPort.u32PortId = VpePortId;

        stBindInfo[0].stDstChnPort.eModId = E_MI_MODULE_ID_DIVP;
        stBindInfo[0].stDstChnPort.u32ChnId = DivpChn;
        stBindInfo[0].stDstChnPort.u32PortId = 0;

        stBindInfo[0].u32SrcFrmrate = dev->setting.u32FrameRate;
        stBindInfo[0].u32DstFrmrate = dev->setting.u32FrameRate;

        memset(&stBindInfo[1], 0x0, sizeof(ST_Sys_BindInfo_T));
        stBindInfo[1].stSrcChnPort.eModId = E_MI_MODULE_ID_DIVP;
        stBindInfo[1].stSrcChnPort.u32DevId = 0;
        stBindInfo[1].stSrcChnPort.u32ChnId = DivpChn;
        stBindInfo[1].stSrcChnPort.u32PortId = 0;

        stBindInfo[1].stDstChnPort.eModId = E_MI_MODULE_ID_VENC;
        stBindInfo[1].stDstChnPort.u32ChnId = VencChn;
        stBindInfo[1].stDstChnPort.u32PortId = 0;

        stBindInfo[1].u32SrcFrmrate = dev->setting.u32FrameRate;
        stBindInfo[1].u32DstFrmrate = dev->setting.u32FrameRate;
        u32FrameRate =  dev->setting.u32FrameRate;
    }

    if (u32Width * u32Height > 2560 *1440)
    {
        u32VenBitRate = 1024 * 1024 * 8;
    }
    else if (u32Width * u32Height >= 1920 *1080)
    {
        u32VenBitRate = 1024 * 1024 * 4;
    }
    else if(u32Width * u32Height < 640*480)
    {
        u32VenBitRate = 1024 * 500;
        /* Use one buffer when resolution than less VGA */
        bHaftRing = false;
    }
    else
    {
        u32VenBitRate = 1024 * 1024 * 2;
    }

    if (g_bitrate[dev->dev_index])
        u32VenBitRate = g_bitrate[dev->dev_index] * 1024 * 1024;

    if (g_qfactor[dev->dev_index])
        u32VenQfactor = g_qfactor[dev->dev_index];

    if (!dev->res.pstuserptr_stream)
    {
        dev->res.pstuserptr_stream = (VENC_STREAMS_t*)calloc(g_maxbuf_cnt, sizeof(VENC_STREAMS_t));
        for(int i=0; i<g_maxbuf_cnt; i++)
        {
            dev->res.pstuserptr_stream[i].stStream.pstPack = (MI_VENC_Pack_t*)calloc(4, sizeof(MI_VENC_Pack_t));
        }
    }
    std::cout <<"=======dev->setting.fcc= %s ============"<< dev->setting.fcc <<std::endl;
    switch(dev->setting.fcc) {
        case V4L2_PIX_FMT_YUYV:
            stVpePortInfo.ePixelFormat = E_MI_SYS_PIXEL_FRAME_YUV422_YUYV;
            stDivpOutputPortAttr.ePixelFormat = E_MI_SYS_PIXEL_FRAME_YUV422_YUYV;

            if(pstStreamAttr[dev->dev_index].enInput == ST_Sys_Input_DIVP)
            {
                STCHECKRESULT(ST_Vpe_StartPort(VpePortId, &stVpePortInfo));

                STCHECKRESULT(MI_DIVP_CreateChn(DivpChn,&stDivpChnAttr));
                STCHECKRESULT(MI_DIVP_SetOutputPortAttr(DivpChn,&stDivpOutputPortAttr));

                stBindInfo[0].stDstChnPort.u32DevId = 0;
                stBindInfo[0].eBindType = E_MI_SYS_BIND_TYPE_FRAME_BASE;
                stBindInfo[0].u32BindParam = 0;
                STCHECKRESULT(ST_Sys_Bind(&stBindInfo[0]));

                STCHECKRESULT(MI_DIVP_StartChn(DivpChn));

                *dstChnPort = stBindInfo[0].stDstChnPort;
                STCHECKRESULT(MI_SYS_SetChnOutputPortDepth(dstChnPort, g_maxbuf_cnt+1, g_maxbuf_cnt+2));

                break;
            }
            else if(pstStreamAttr[dev->dev_index].enInput == ST_Sys_Input_VPE)
            {
                STCHECKRESULT(ST_Vpe_StartPort(VpePortId, &stVpePortInfo));

                *dstChnPort = stBindInfo[0].stSrcChnPort;
                STCHECKRESULT(MI_SYS_SetChnOutputPortDepth(dstChnPort, g_maxbuf_cnt+1, g_maxbuf_cnt+2));

                break;
            }

        case V4L2_PIX_FMT_NV12:
            stVpePortInfo.ePixelFormat = E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420;
            stDivpOutputPortAttr.ePixelFormat = E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420;

            if(pstStreamAttr[dev->dev_index].enInput == ST_Sys_Input_DIVP)
            {
                STCHECKRESULT(ST_Vpe_StartPort(VpePortId, &stVpePortInfo));

                STCHECKRESULT(MI_DIVP_CreateChn(DivpChn,&stDivpChnAttr));
                STCHECKRESULT(MI_DIVP_SetOutputPortAttr(DivpChn,&stDivpOutputPortAttr));

                stBindInfo[0].stDstChnPort.u32DevId = 0;
                stBindInfo[0].eBindType = E_MI_SYS_BIND_TYPE_FRAME_BASE;
                stBindInfo[0].u32BindParam = 0;
                STCHECKRESULT(ST_Sys_Bind(&stBindInfo[0]));

                STCHECKRESULT(MI_DIVP_StartChn(DivpChn));

                *dstChnPort = stBindInfo[0].stDstChnPort;
                STCHECKRESULT(MI_SYS_SetChnOutputPortDepth(dstChnPort, g_maxbuf_cnt+1, g_maxbuf_cnt+2));

                break;
            }
            else if(pstStreamAttr[dev->dev_index].enInput == ST_Sys_Input_VPE)
            {
                STCHECKRESULT(ST_Vpe_StartPort(VpePortId, &stVpePortInfo));

                *dstChnPort = stBindInfo[0].stSrcChnPort;
                STCHECKRESULT(MI_SYS_SetChnOutputPortDepth(dstChnPort, g_maxbuf_cnt+1, g_maxbuf_cnt+2));

                break;
            }

        case V4L2_PIX_FMT_MJPEG:
            pstStreamAttr[dev->dev_index].bUserVenc = TRUE;

            if(bEnableMjpegRealtimeMode)
            {
                ST_DBG("VPE->VENC Mjpeg use realtime(IMI) mode, Notice that just for 1 mjpeg video streaming!\n");
                stVpePortInfo.ePixelFormat = E_MI_SYS_PIXEL_FRAME_YUV422_YUYV;;
            }
            else
            {
                stVpePortInfo.ePixelFormat = E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420;
            }

            stDivpOutputPortAttr.ePixelFormat = E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420;
#if 1
            stChnAttr.stRcAttr.eRcMode = E_MI_VENC_RC_MODE_MJPEGFIXQP;
            stChnAttr.stRcAttr.stAttrMjpegFixQp.u32SrcFrmRateNum = u32FrameRate;
            stChnAttr.stRcAttr.stAttrMjpegFixQp.u32SrcFrmRateDen = 1;
            stChnAttr.stVeAttr.stAttrJpeg.u32BufSize = ALIGN_UP(u32Width*u32Height*3/4, 16);
            stChnAttr.stVeAttr.stAttrJpeg.u32PicWidth = u32Width;
            stChnAttr.stVeAttr.stAttrJpeg.u32PicHeight = u32Height;
            stChnAttr.stVeAttr.stAttrJpeg.u32MaxPicWidth =  u32Width;
            stChnAttr.stVeAttr.stAttrJpeg.u32MaxPicHeight = ALIGN_UP(u32Height, 16);
            stChnAttr.stVeAttr.stAttrJpeg.bByFrame = true;
            stChnAttr.stVeAttr.eType = E_MI_VENC_MODTYPE_JPEGE;
            stChnAttr.stRcAttr.stAttrMjpegFixQp.u32Qfactor = u32VenQfactor; //default 80, [0~90]
#else
            stChnAttr.stVeAttr.stAttrJpeg.u32PicWidth = u32Width;
            stChnAttr.stVeAttr.stAttrJpeg.u32PicHeight = u32Height;
            stChnAttr.stVeAttr.stAttrJpeg.u32MaxPicWidth = u32Width;
            stChnAttr.stVeAttr.stAttrJpeg.u32MaxPicHeight = ALIGN_UP(u32Height, 16);;
            stChnAttr.stVeAttr.stAttrJpeg.bByFrame = bByFrame;

            stChnAttr.stRcAttr.eRcMode = E_MI_VENC_RC_MODE_MJPEGCBR;
            stChnAttr.stRcAttr.stAttrMjpegCbr.u32BitRate = 1024*1024*15*u32VenBitRate;
            stChnAttr.stRcAttr.stAttrMjpegCbr.u32SrcFrmRateNum = 30;
            stChnAttr.stRcAttr.stAttrMjpegCbr.u32SrcFrmRateDen = 1;
            stChnAttr.stVeAttr.eType = E_MI_VENC_MODTYPE_JPEGE;
#endif

            if(pstStreamAttr[dev->dev_index].enInput == ST_Sys_Input_DIVP)
            {
                STCHECKRESULT(ST_Vpe_StartPort(VpePortId, &stVpePortInfo));

                STCHECKRESULT(MI_DIVP_CreateChn(DivpChn,&stDivpChnAttr));
                STCHECKRESULT(MI_DIVP_SetOutputPortAttr(DivpChn,&stDivpOutputPortAttr));

                stBindInfo[0].stDstChnPort.u32DevId = 0;
                stBindInfo[0].eBindType = E_MI_SYS_BIND_TYPE_FRAME_BASE;
                stBindInfo[0].u32BindParam = 0;
                STCHECKRESULT(ST_Sys_Bind(&stBindInfo[0]));

                STCHECKRESULT(MI_DIVP_StartChn(DivpChn));

                STCHECKRESULT(ST_Venc_CreateChannel(VencChn, &stChnAttr));
                STCHECKRESULT(MI_VENC_GetChnDevid(VencChn, &VencDev));

                stBindInfo[1].stDstChnPort.u32DevId = VencDev;
                pstStreamAttr[dev->dev_index].eBindType = stBindInfo[1].eBindType = E_MI_SYS_BIND_TYPE_FRAME_BASE;
                pstStreamAttr[dev->dev_index].u32BindPara = stBindInfo[1].u32BindParam = 0;
                STCHECKRESULT(ST_Sys_Bind(&stBindInfo[1]));

                STCHECKRESULT(MI_VENC_SetMaxStreamCnt(VencChn, g_maxbuf_cnt+1));
                STCHECKRESULT(ST_Venc_StartChannel(VencChn));
                *dstChnPort = stBindInfo[1].stDstChnPort;

                break;
            }
            else if(pstStreamAttr[dev->dev_index].enInput == ST_Sys_Input_VPE)
            {
                STCHECKRESULT(ST_Vpe_StartPort(VpePortId, &stVpePortInfo));

                STCHECKRESULT(ST_Venc_CreateChannel(VencChn, &stChnAttr));
                STCHECKRESULT(MI_VENC_GetChnDevid(VencChn, &VencDev));

                stBindInfo[0].stDstChnPort.u32DevId = VencDev;

                if(bEnableMjpegRealtimeMode)
                {
                    pstStreamAttr[dev->dev_index].eBindType = stBindInfo[0].eBindType = E_MI_SYS_BIND_TYPE_REALTIME;
                }
                else
                {
                    pstStreamAttr[dev->dev_index].eBindType = stBindInfo[0].eBindType = E_MI_SYS_BIND_TYPE_FRAME_BASE;
                }

                pstStreamAttr[dev->dev_index].u32BindPara = stBindInfo[0].u32BindParam = 0;
                STCHECKRESULT(ST_Sys_Bind(&stBindInfo[0]));

                STCHECKRESULT(MI_VENC_SetMaxStreamCnt(VencChn, g_maxbuf_cnt+1));
                STCHECKRESULT(ST_Venc_StartChannel(VencChn));
                *dstChnPort = stBindInfo[0].stDstChnPort;

                break;
            }

        case V4L2_PIX_FMT_H264:
            pstStreamAttr[dev->dev_index].bUserVenc = TRUE;
            stVpePortInfo.ePixelFormat = E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420;
            stDivpOutputPortAttr.ePixelFormat = E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420;

            stChnAttr.stVeAttr.stAttrH264e.u32PicWidth = u32Width;
            stChnAttr.stVeAttr.stAttrH264e.u32PicHeight = u32Height;
            stChnAttr.stVeAttr.stAttrH264e.u32MaxPicWidth = u32Width;
            stChnAttr.stVeAttr.stAttrH264e.u32MaxPicHeight = u32Height;
            stChnAttr.stVeAttr.stAttrH264e.bByFrame = bByFrame;
            stChnAttr.stVeAttr.stAttrH264e.u32BFrameNum = 2;
            stChnAttr.stVeAttr.stAttrH264e.u32Profile = 1;
            stChnAttr.stRcAttr.eRcMode = E_MI_VENC_RC_MODE_H264CBR;
            stChnAttr.stRcAttr.stAttrH264Cbr.u32BitRate = u32VenBitRate;
            stChnAttr.stRcAttr.stAttrH264Cbr.u32FluctuateLevel = 0;
            stChnAttr.stRcAttr.stAttrH264Cbr.u32Gop = 30;
            stChnAttr.stRcAttr.stAttrH264Cbr.u32SrcFrmRateNum = u32FrameRate;
            stChnAttr.stRcAttr.stAttrH264Cbr.u32SrcFrmRateDen = 1;
            stChnAttr.stRcAttr.stAttrH264Cbr.u32StatTime = 0;
            stChnAttr.stVeAttr.eType = E_MI_VENC_MODTYPE_H264E;

            if(pstStreamAttr[dev->dev_index].enInput == ST_Sys_Input_DIVP)
            {
                STCHECKRESULT(ST_Vpe_StartPort(VpePortId, &stVpePortInfo));

                STCHECKRESULT(MI_DIVP_CreateChn(DivpChn,&stDivpChnAttr));
                STCHECKRESULT(MI_DIVP_SetOutputPortAttr(DivpChn,&stDivpOutputPortAttr));

                stBindInfo[0].stDstChnPort.u32DevId = 0;
                stBindInfo[0].eBindType = E_MI_SYS_BIND_TYPE_FRAME_BASE;
                stBindInfo[0].u32BindParam = 0;
                STCHECKRESULT(ST_Sys_Bind(&stBindInfo[0]));

                STCHECKRESULT(MI_DIVP_StartChn(DivpChn));

                STCHECKRESULT(ST_Venc_CreateChannel(VencChn, &stChnAttr));
                {
                    MI_VENC_ParamH264SliceSplit_t stSliceSplit = {true, 17};
                    MI_VENC_SetH264SliceSplit(VencChn, &stSliceSplit);
                }
                STCHECKRESULT(MI_VENC_GetChnDevid(VencChn, &VencDev));

                stBindInfo[1].stDstChnPort.u32DevId = VencDev;
                pstStreamAttr[dev->dev_index].eBindType = stBindInfo[1].eBindType = E_MI_SYS_BIND_TYPE_FRAME_BASE;
                pstStreamAttr[dev->dev_index].u32BindPara = stBindInfo[1].u32BindParam = 0;
#if (defined(CONFIG_SIGMASTAR_CHIP_I6B0))
                if(g_device_num == 1)
                {
                    pstStreamAttr[dev->dev_index].eBindType = stBindInfo[1].eBindType = E_MI_SYS_BIND_TYPE_HW_RING;
                    pstStreamAttr[dev->dev_index].u32BindPara = stBindInfo[1].u32BindParam = bHaftRing ? u32Height/2 : u32Height;
                     if (bHaftRing)
                    {
                        stVenInSrc.eInputSrcBufferMode = E_MI_VENC_INPUT_MODE_RING_HALF_FRM;
                        MI_VENC_SetInputSourceConfig(VencChn, &stVenInSrc);
                    }
                    else
                    {
                        stVenInSrc.eInputSrcBufferMode = E_MI_VENC_INPUT_MODE_RING_ONE_FRM;
                        MI_VENC_SetInputSourceConfig(VencChn, &stVenInSrc);
                    }
                }
#endif
                STCHECKRESULT(ST_Sys_Bind(&stBindInfo[1]));

                STCHECKRESULT(MI_VENC_SetMaxStreamCnt(VencChn, g_maxbuf_cnt+1));
                STCHECKRESULT(ST_Venc_StartChannel(VencChn));
                *dstChnPort = stBindInfo[1].stDstChnPort;

                break;
            }
            else if(pstStreamAttr[dev->dev_index].enInput == ST_Sys_Input_VPE)
            {
                STCHECKRESULT(ST_Vpe_StartPort(VpePortId, &stVpePortInfo));

                STCHECKRESULT(ST_Venc_CreateChannel(VencChn, &stChnAttr));
                {
                    MI_VENC_ParamH264SliceSplit_t stSliceSplit = {true, 17};
                    MI_VENC_SetH264SliceSplit(VencChn, &stSliceSplit);
                }
                STCHECKRESULT(MI_VENC_GetChnDevid(VencChn, &VencDev));

                stBindInfo[0].stDstChnPort.u32DevId = VencDev;
                pstStreamAttr[dev->dev_index].eBindType = stBindInfo[0].eBindType = E_MI_SYS_BIND_TYPE_FRAME_BASE;
                pstStreamAttr[dev->dev_index].u32BindPara = stBindInfo[0].u32BindParam = 0;
#if (defined(CONFIG_SIGMASTAR_CHIP_I6B0))
                if(g_device_num == 1)
                {
                    pstStreamAttr[dev->dev_index].eBindType = stBindInfo[0].eBindType = E_MI_SYS_BIND_TYPE_HW_RING;
                    pstStreamAttr[dev->dev_index].u32BindPara = stBindInfo[0].u32BindParam = bHaftRing ? u32Height/2 : u32Height;
                    if (bHaftRing)
                    {
                        stVenInSrc.eInputSrcBufferMode = E_MI_VENC_INPUT_MODE_RING_HALF_FRM;
                        MI_VENC_SetInputSourceConfig(VencChn, &stVenInSrc);
                    }
                    else
                    {
                        stVenInSrc.eInputSrcBufferMode = E_MI_VENC_INPUT_MODE_RING_ONE_FRM;
                        MI_VENC_SetInputSourceConfig(VencChn, &stVenInSrc);
                    }
                }
#endif
                STCHECKRESULT(ST_Sys_Bind(&stBindInfo[0]));

                STCHECKRESULT(MI_VENC_SetMaxStreamCnt(VencChn, g_maxbuf_cnt+1));
                STCHECKRESULT(ST_Venc_StartChannel(VencChn));
                *dstChnPort = stBindInfo[0].stDstChnPort;

                break;
            }

        case V4L2_PIX_FMT_H265:
            pstStreamAttr[dev->dev_index].bUserVenc = TRUE;
            stVpePortInfo.ePixelFormat = E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420;
            stDivpOutputPortAttr.ePixelFormat = E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420;

            stChnAttr.stVeAttr.stAttrH265e.u32PicWidth = u32Width;
            stChnAttr.stVeAttr.stAttrH265e.u32PicHeight = u32Height;
            stChnAttr.stVeAttr.stAttrH265e.u32MaxPicWidth = u32Width;
            stChnAttr.stVeAttr.stAttrH265e.u32MaxPicHeight = u32Height;
            stChnAttr.stVeAttr.stAttrH265e.bByFrame = bByFrame;
            stChnAttr.stRcAttr.eRcMode = E_MI_VENC_RC_MODE_H265CBR;
            stChnAttr.stRcAttr.stAttrH265Cbr.u32BitRate = u32VenBitRate;
            stChnAttr.stRcAttr.stAttrH265Cbr.u32SrcFrmRateNum = u32FrameRate;
            stChnAttr.stRcAttr.stAttrH265Cbr.u32SrcFrmRateDen = 1;
            stChnAttr.stRcAttr.stAttrH265Cbr.u32Gop = 30;
            stChnAttr.stRcAttr.stAttrH265Cbr.u32FluctuateLevel = 0;
            stChnAttr.stRcAttr.stAttrH265Cbr.u32StatTime = 0;
            stChnAttr.stVeAttr.eType = E_MI_VENC_MODTYPE_H265E;

            if(pstStreamAttr[dev->dev_index].enInput == ST_Sys_Input_DIVP)
            {
                STCHECKRESULT(ST_Vpe_StartPort(VpePortId, &stVpePortInfo));

                STCHECKRESULT(MI_DIVP_CreateChn(DivpChn,&stDivpChnAttr));
                STCHECKRESULT(MI_DIVP_SetOutputPortAttr(DivpChn,&stDivpOutputPortAttr));

                stBindInfo[0].stDstChnPort.u32DevId = 0;
                stBindInfo[0].eBindType = E_MI_SYS_BIND_TYPE_FRAME_BASE;
                stBindInfo[0].u32BindParam = 0;
                STCHECKRESULT(ST_Sys_Bind(&stBindInfo[0]));

                STCHECKRESULT(MI_DIVP_StartChn(DivpChn));

                STCHECKRESULT(ST_Venc_CreateChannel(VencChn, &stChnAttr));
                STCHECKRESULT(MI_VENC_GetChnDevid(VencChn, &VencDev));

                stBindInfo[1].stDstChnPort.u32DevId = VencDev;
                pstStreamAttr[dev->dev_index].eBindType = stBindInfo[1].eBindType = E_MI_SYS_BIND_TYPE_FRAME_BASE;
                pstStreamAttr[dev->dev_index].u32BindPara = stBindInfo[1].u32BindParam = 0;
#if (defined(CONFIG_SIGMASTAR_CHIP_I6B0))
                if(g_device_num == 1)
                {
                    pstStreamAttr[dev->dev_index].eBindType = stBindInfo[1].eBindType = E_MI_SYS_BIND_TYPE_HW_RING;
                    pstStreamAttr[dev->dev_index].u32BindPara = stBindInfo[1].u32BindParam = bHaftRing ? u32Height/2 : u32Height;
                    if (bHaftRing)
                    {
                        stVenInSrc.eInputSrcBufferMode = E_MI_VENC_INPUT_MODE_RING_HALF_FRM;
                        MI_VENC_SetInputSourceConfig(VencChn, &stVenInSrc);
                    }
                    else
                    {
                        stVenInSrc.eInputSrcBufferMode = E_MI_VENC_INPUT_MODE_RING_ONE_FRM;
                        MI_VENC_SetInputSourceConfig(VencChn, &stVenInSrc);
                    }
                }
#endif
                STCHECKRESULT(ST_Sys_Bind(&stBindInfo[1]));

                STCHECKRESULT(MI_VENC_SetMaxStreamCnt(VencChn, g_maxbuf_cnt+1));
                STCHECKRESULT(ST_Venc_StartChannel(VencChn));
                *dstChnPort = stBindInfo[1].stDstChnPort;

                break;
            }
            else if(pstStreamAttr[dev->dev_index].enInput == ST_Sys_Input_VPE)
            {
                STCHECKRESULT(ST_Vpe_StartPort(VpePortId, &stVpePortInfo));

                STCHECKRESULT(ST_Venc_CreateChannel(VencChn, &stChnAttr));
                STCHECKRESULT(MI_VENC_GetChnDevid(VencChn, &VencDev));

                stBindInfo[0].stDstChnPort.u32DevId = VencDev;
                pstStreamAttr[dev->dev_index].eBindType = stBindInfo[0].eBindType = E_MI_SYS_BIND_TYPE_FRAME_BASE;
                pstStreamAttr[dev->dev_index].u32BindPara = stBindInfo[0].u32BindParam = 0;
#if (defined(CONFIG_SIGMASTAR_CHIP_I6B0))
                if(g_device_num == 1)
                {
                    pstStreamAttr[dev->dev_index].eBindType = stBindInfo[0].eBindType = E_MI_SYS_BIND_TYPE_HW_RING;
                    pstStreamAttr[dev->dev_index].u32BindPara = stBindInfo[0].u32BindParam = bHaftRing ? u32Height/2 : u32Height;
                    if (bHaftRing)
                    {
                        stVenInSrc.eInputSrcBufferMode = E_MI_VENC_INPUT_MODE_RING_HALF_FRM;
                        MI_VENC_SetInputSourceConfig(VencChn, &stVenInSrc);
                    }
                    else
                    {
                        stVenInSrc.eInputSrcBufferMode = E_MI_VENC_INPUT_MODE_RING_ONE_FRM;
                        MI_VENC_SetInputSourceConfig(VencChn, &stVenInSrc);
                    }
                }
#endif
                STCHECKRESULT(ST_Sys_Bind(&stBindInfo[0]));

                STCHECKRESULT(MI_VENC_SetMaxStreamCnt(VencChn, g_maxbuf_cnt+1));
                STCHECKRESULT(ST_Venc_StartChannel(VencChn));
                *dstChnPort = stBindInfo[0].stDstChnPort;

                break;
            }

        default:
            return -EINVAL;
    }

    // //init divp channel-1
    // stDivpChnAttr.bHorMirror = FALSE;
    // stDivpChnAttr.bVerMirror = FALSE;
    // stDivpChnAttr.eDiType = E_MI_DIVP_DI_TYPE_OFF;
    // stDivpChnAttr.eRotateType = E_MI_SYS_ROTATE_NONE;
    // stDivpChnAttr.eTnrLevel = E_MI_DIVP_TNR_LEVEL_OFF;
    // stDivpChnAttr.stCropRect.u16X = 0;
    // stDivpChnAttr.stCropRect.u16Y = 0;
    // stDivpChnAttr.u32MaxWidth = DIVP_INPUT_WIDTH;
    // stDivpChnAttr.u32MaxHeight = DIVP_INPUT_WIDTH;
    // ExecFunc(MI_DIVP_CreateChn(1, &stDivpChnAttr), MI_SUCCESS);
    // stDivpPortAttr.u32Width = MODEL_WIDTH;
    // stDivpPortAttr.u32Height = MODEL_HEIGHT;
    // stDivpPortAttr.eCompMode = E_MI_SYS_COMPRESS_MODE_NONE;
    // stDivpPortAttr.ePixelFormat = E_MI_SYS_PIXEL_FRAME_ARGB8888;
    // ExecFunc(MI_DIVP_SetOutputPortAttr(1, &stDivpPortAttr), MI_SUCCESS);
    // ExecFunc(MI_DIVP_StartChn(1), MI_SUCCESS);

    // // stBindInfo.stSrcChnPort.eModId = E_MI_MODULE_ID_VPE;
    // // stBindInfo.stSrcChnPort.u32DevId = 0;
    // // stBindInfo.stSrcChnPort.u32ChnId = 0;
    // // stBindInfo.stSrcChnPort.u32PortId = 0;
    // // stBindInfo.stDstChnPort.eModId = E_MI_MODULE_ID_DIVP;
    // // stBindInfo.stDstChnPort.u32DevId = 1;
    // // stBindInfo.eBindType = E_MI_SYS_BIND_TYPE_FRAME_BASE;
    // // stBindInfo.stDstChnPort.u32ChnId = 0;
    // // stBindInfo.stDstChnPort.u32PortId = 0;
    // // stBindInfo.u32SrcFrmrate = 30;
    // // stBindInfo.u32DstFrmrate = 15;
    // // ExecFunc(MI_SYS_SetChnOutputPortDepth(&stBindInfo.stDstChnPort, 3, 4), MI_SUCCESS);
    // // STCHECKRESULT(ST_Sys_Bind(&stBindInfo));

    if(g_load_iq_bin)
    {
        ST_DoSetIqBin(0, g_IspBinPath);
        g_load_iq_bin = 0;
    }

    g_bStartCapture = TRUE;

    printf("Capture u32Width: %d, u32height: %d, format: %s\n",u32Width,u32Height,
        dev->setting.fcc==V4L2_PIX_FMT_YUYV ? "YUYV":(dev->setting.fcc==V4L2_PIX_FMT_NV12 ? "NV12":
        (dev->setting.fcc==V4L2_PIX_FMT_MJPEG ?"MJPEG":(dev->setting.fcc==V4L2_PIX_FMT_H264 ? "H264":"H265"))));

    if(gbDumpData)
    {
        char fileName[128] = {0};
        sprintf(fileName, "%s/%s",gaDumpFilePath, "dump.es");
        gpDumpFile = fopen(fileName, "w");
        if(!gpDumpFile)
        {

            ST_ERR("fopen %s failed!!!\n", fileName);
        }
        else
        {
            ST_DBG("dump file %s open success!\n", fileName);
        }
    }

    return MI_SUCCESS;
}

static MI_S32 UVC_StopCapture(void *uvc)
{
    ST_UvcDev_t *dev = Get_UVC_Device(uvc);
    ST_Sys_BindInfo_T stBindInfo[2];
    ST_Stream_Attr_T *pstStreamAttr = g_stStreamAttr;
    MI_U32 VpeChn, VpePortId, VencDev, VencChn, DivpChn;

    if (!dev)
        return -1;

    /************************************************
    Step0:  General Param Set
    *************************************************/
    VpeChn = pstStreamAttr[dev->dev_index].u32InputChn;
    VpePortId = pstStreamAttr[dev->dev_index].u32InputPort,
    VencChn = pstStreamAttr[dev->dev_index].vencChn,
    DivpChn = pstStreamAttr[dev->dev_index].divpChn;
    pstStreamAttr[dev->dev_index].bUserVenc = FALSE;
    pstStreamAttr[dev->dev_index].bEnable = FALSE;
    if(pstStreamAttr[dev->dev_index].enInput == ST_Sys_Input_VPE)
    {
        memset(&stBindInfo[0], 0x0, sizeof(ST_Sys_BindInfo_T));
        stBindInfo[0].stSrcChnPort.eModId = E_MI_MODULE_ID_VPE;
        stBindInfo[0].stSrcChnPort.u32DevId = 0;
        stBindInfo[0].stSrcChnPort.u32ChnId = VpeChn;
        stBindInfo[0].stSrcChnPort.u32PortId = VpePortId;

        stBindInfo[0].stDstChnPort.eModId = E_MI_MODULE_ID_VENC;
        stBindInfo[0].stDstChnPort.u32ChnId = VencChn;
        stBindInfo[0].stDstChnPort.u32PortId = 0;

        stBindInfo[0].u32SrcFrmrate = 30;
        stBindInfo[0].u32DstFrmrate = dev->setting.u32FrameRate;
    }
    else if(pstStreamAttr[dev->dev_index].enInput == ST_Sys_Input_DIVP)
    {
        memset(&stBindInfo[0], 0x0, sizeof(ST_Sys_BindInfo_T));

        stBindInfo[0].stSrcChnPort.eModId = E_MI_MODULE_ID_VPE;
        stBindInfo[0].stSrcChnPort.u32DevId = 0;
        stBindInfo[0].stSrcChnPort.u32ChnId = VpeChn;
        stBindInfo[0].stSrcChnPort.u32PortId = VpePortId;

        stBindInfo[0].stDstChnPort.eModId = E_MI_MODULE_ID_DIVP;
        stBindInfo[0].stDstChnPort.u32ChnId = DivpChn;
        stBindInfo[0].stDstChnPort.u32PortId = 0;

        stBindInfo[0].u32SrcFrmrate = 30;
        stBindInfo[0].u32DstFrmrate = dev->setting.u32FrameRate;

        memset(&stBindInfo[1], 0x0, sizeof(ST_Sys_BindInfo_T));
        stBindInfo[1].stSrcChnPort.eModId = E_MI_MODULE_ID_DIVP;
        stBindInfo[1].stSrcChnPort.u32DevId = 0;
        stBindInfo[1].stSrcChnPort.u32ChnId = DivpChn;
        stBindInfo[1].stSrcChnPort.u32PortId = 0;

        stBindInfo[1].stDstChnPort.eModId = E_MI_MODULE_ID_VENC;
        stBindInfo[1].stDstChnPort.u32ChnId = VencChn;
        stBindInfo[1].stDstChnPort.u32PortId = 0;

        stBindInfo[1].u32SrcFrmrate = 30;
        stBindInfo[1].u32DstFrmrate = dev->setting.u32FrameRate;
    }

    if (dev->res.pstuserptr_stream)
    {
        for(int i=0; i<g_maxbuf_cnt; i++)
        {
            free(dev->res.pstuserptr_stream[i].stStream.pstPack);
        }
        free(dev->res.pstuserptr_stream);
        dev->res.pstuserptr_stream = NULL;
    }

    /************************************************
    Step1:  Stop All Other Modules
    *************************************************/
    switch(dev->setting.fcc) {
        case V4L2_PIX_FMT_YUYV:
        case V4L2_PIX_FMT_NV12:
            if(pstStreamAttr[dev->dev_index].enInput == ST_Sys_Input_DIVP)
            {
                STCHECKRESULT(ST_Sys_UnBind(&stBindInfo[0]));
                STCHECKRESULT(MI_DIVP_StopChn(DivpChn));
                STCHECKRESULT(MI_DIVP_DestroyChn(DivpChn));
                STCHECKRESULT(ST_Vpe_StopPort(VpeChn, VpePortId));
                break;
            }
            else if(pstStreamAttr[dev->dev_index].enInput == ST_Sys_Input_VPE)
            {
                STCHECKRESULT(ST_Vpe_StopPort(VpeChn, VpePortId));
                break;
            }

        case V4L2_PIX_FMT_MJPEG:
        case V4L2_PIX_FMT_H264:
        case V4L2_PIX_FMT_H265:
            if(pstStreamAttr[dev->dev_index].enInput == ST_Sys_Input_DIVP)
            {
                STCHECKRESULT(MI_VENC_GetChnDevid(VencChn, &VencDev));
                stBindInfo[1].stDstChnPort.u32DevId = VencDev;
                STCHECKRESULT(ST_Sys_UnBind(&stBindInfo[0]));
                STCHECKRESULT(ST_Sys_UnBind(&stBindInfo[1]));
                STCHECKRESULT(ST_Vpe_StopPort(VpeChn, VpePortId));
                STCHECKRESULT(MI_DIVP_StopChn(DivpChn));
                STCHECKRESULT(MI_DIVP_DestroyChn(DivpChn));
                STCHECKRESULT(ST_Venc_StopChannel(VencChn));
                STCHECKRESULT(ST_Venc_DestoryChannel(VencChn));
                break;
            }
            else if(pstStreamAttr[dev->dev_index].enInput == ST_Sys_Input_VPE)
            {
                STCHECKRESULT(MI_VENC_GetChnDevid(VencChn, &VencDev));
                stBindInfo[0].stDstChnPort.u32DevId = VencDev;
                STCHECKRESULT(ST_Sys_UnBind(&stBindInfo[0]));
                STCHECKRESULT(ST_Vpe_StopPort(VpeChn, VpePortId));
                STCHECKRESULT(ST_Venc_StopChannel(VencChn));
                STCHECKRESULT(ST_Venc_DestoryChannel(VencChn));
                break;
            }

        default:
            return -EINVAL;
    }

    if(gbDumpData)
    {
        if(gpDumpFile)
        {
            int ret = fclose(gpDumpFile);

            if(!ret)
            {
                ST_DBG("close file success!\n");
            }
            else
            {
                ST_DBG("close file failed!\n");
            }

            gpDumpFile = NULL;

        }
    }

    if(gbSnrPowerOnWhenPreview)
    {
        STCHECKRESULT(ST_VideoModuleUnInit(&g_stConfig));
    }

    return MI_SUCCESS;
}

MI_S32 ST_UvcDeinit()
{
    if (!g_UvcSrc)
        return -1;

    for (int i = 0; i < g_UvcSrc->devnum; i++)
    {
        ST_UvcDev_t *dev = &g_UvcSrc->dev[i];
        STCHECKRESULT(ST_UVC_StopDev((dev->handle)));
        STCHECKRESULT(ST_UVC_DestroyDev(dev->handle));
        STCHECKRESULT(ST_UVC_Uninit(dev->handle));
    }
    return MI_SUCCESS;
}


MI_S32 ST_UvcInitDev(ST_UvcDev_t *dev, MI_U32 maxpacket, MI_U8 mult, MI_U8 burst, MI_U8 c_intf, MI_U8 s_intf, MI_S32 mode, MI_S32 type)
{
    ST_UVC_Setting_t pstSet={g_maxbuf_cnt, maxpacket, mult, burst, c_intf, s_intf, (UVC_IO_MODE_e)mode, (Transfer_Mode_e)type};
    ST_UVC_MMAP_BufOpts_t m = {UVC_MM_FillBuffer};
    ST_UVC_USERPTR_BufOpts_t u = {UVC_UP_FillBuffer, UVC_UP_FinishBuffer};

    ST_UVC_OPS_t fops = { UVC_Init,
                          UVC_Deinit,
                          {{}},
                          UVC_StartCapture,
                          UVC_StopCapture,
                          UVC_ForceIdr};
    if (mode==UVC_MEMORY_MMAP)
        fops.m = m;
    else
        fops.u = u;

    printf(ASCII_COLOR_YELLOW "ST_UvcInitDev: name:%s bufcnt:%d mult:%d burst:%d ci:%d si:%d, Mode:%s, Type:%s" ASCII_COLOR_END "\n",
                    dev->name, g_maxbuf_cnt, mult, burst, c_intf, s_intf, mode==UVC_MEMORY_MMAP?"mmap":"userptr", type==USB_ISOC_MODE?"isoc":"bulk");

    ST_UVC_ChnAttr_t pstAttr ={pstSet,fops};
    STCHECKRESULT(ST_UVC_Init(dev->name, &dev->handle));
    STCHECKRESULT(ST_UVC_CreateDev(dev->handle, &pstAttr));
    STCHECKRESULT(ST_UVC_StartDev(dev->handle));
    return MI_SUCCESS;
}

MI_S32 ST_UvcInit(MI_S32 devnum, MI_U32 *maxpacket, MI_U8 *mult, MI_U8 *burst, MI_U8 *intf, MI_S32 mode, MI_S32 type)
{
    char devnode[20] = "/dev/video0";

    if (devnum > MAX_UVC_DEV_NUM)
    {
        printf(ASCII_COLOR_YELLOW "%s Max Uvc Dev Num %d\n" ASCII_COLOR_END "\n", __func__, MAX_UVC_DEV_NUM);
        devnum = MAX_UVC_DEV_NUM;
    }

    g_UvcSrc = (ST_UvcSrc_t*)malloc(sizeof(g_UvcSrc) + sizeof(ST_UvcDev_t) * devnum);
    memset(g_UvcSrc, 0x0, sizeof(g_UvcSrc) + sizeof(ST_UvcDev_t) * devnum);
    g_UvcSrc->devnum = devnum;

    for (int i = 0; i < devnum; i++)
    {
        ST_UvcDev_t *dev = &g_UvcSrc->dev[i];
        sprintf(devnode, "/dev/video%d", i);
        dev->dev_index = i;
        memcpy(dev->name, devnode, sizeof(devnode));
        ST_UvcInitDev(dev, maxpacket[i], mult[i], burst[i], intf[2*i], intf[2*i+1], mode, type);
    }
    return MI_SUCCESS;
}

void ST_DoExitProc(void *args)
{
    g_bExit = TRUE;
}

static void help_message(char **argv)
{
    printf("\n");
    printf("usage: %s \n", argv[0]);
    printf(" -a set sensor pad\n");
    printf(" -A set sensor resolution index\n");
    printf(" -b bitrate\n");
    printf(" -B burst\n");
    printf(" -c set 3DNR level, 0:OFF 1:8bit 2:12bit\n");
    printf(" -C enable ST test\n");
    printf(" -e set dump file path,ex: -e /customer/ . if set \"-e null\",then set path after UVC init.\n");
    printf(" -m mult\n");
    printf(" -p maxpacket\n");
    printf(" -L enable LDC and set bin path,ex: -L /config/AMTK_FHD/cfg/AMTK_1R_fhd.cfg\n");
    printf(" -M 0:mmap, 1:userptr\n");
    printf(" -N num of uvc stream\n");
    printf(" -i set iq api.bin,ex: -i /customer/imx415_api.bin \n");
    printf(" -I c_intf,s_intf\n");
    printf("    c_intf: control interface\n");
    printf("    s_intf: streaming interface\n");
    printf(" -t Trace level (0-6) \n");
    printf(" -q open iqserver\n");
    printf(" -T 0:Isoc, 1:Bulk\n");
    printf(" -Q qfactor\n");
    printf(" -V 0:Disable video, 1:Enable video(Default)\n");
    printf(" -d : AI Device Id: Amic[0] Dmic[1] I2S RX[2] Linein[3]\n");
    printf(" -s : AI Samle Rate \n\t ex:./prog_uac -I -s48000\n");
    printf(" -D : AO Device Id: Lineout[0] I2S TX[1]\n");
    printf(" -S : AO Samle Rate \n\t ex:./prog_uac -O -S48000\n");
    printf(" -h help message\n");
    printf(" multi stram param, using ',' to seperate\n");
    printf("\nExample: %s -N2 -m0,0 -M1 -I0,1,2,3\n", argv[0]);
    printf("\n");
}

#define PARSE_PARM(instance)\
{\
    int i = 0; \
    do { \
        if (!i) {\
            p = strtok(optarg, ","); \
        } \
        if (p) {\
            instance[i++] = atoi(p); \
        } \
    } while((p = strtok(NULL, ","))!=NULL); \
}

int main(int argc, char **argv)
{
    char *p;
    MI_U32 maxpacket[MAX_UVC_DEV_NUM] = {1024, 1024, 192, 64};
    MI_U8 mult[MAX_UVC_DEV_NUM] = {2, 2, 2, 2},
          burst[MAX_UVC_DEV_NUM] = {13, 13, 0, 0};
    MI_U8 intf[2 * MAX_UVC_DEV_NUM] = {0};
    MI_S32 result = 0, mode = UVC_MEMORY_MMAP, type = USB_ISOC_MODE;
    MI_S32 trace_level = UVC_DBG_ERR;
    struct sigaction sigAction;
    sigAction.sa_handler = ST_HandleSig;
    sigemptyset(&sigAction.sa_mask);
    sigAction.sa_flags = 0;
    sigaction(SIGINT, &sigAction, NULL);

    ST_DefaultArgs(&g_stConfig);
    //set bitrate
    while((result = getopt(argc, argv, "a:A:b:B:c:CD:d:e:EHhi:I:L:M:m:N:p:q:Q:rs:S:t:T:V:")) != -1)
    {
        switch(result)
        {
            case 'a':
                g_stConfig.u8SnrPad  = strtol(optarg, NULL, 10);
                break;

            case 'A':
                g_stConfig.s8SnrResIndex = strtol(optarg, NULL, 10);
                break;

            case 'b':
                PARSE_PARM(g_bitrate);
                break;

            case 'B':
                PARSE_PARM(burst);
                break;

            case 'c':
                g_stConfig.en3dNrLevel = (MI_VPE_3DNR_Level_e)strtol(optarg, NULL, 10);
                ST_DBG("3DNR level:%d\n", g_stConfig.en3dNrLevel);
                break;

            case 'C':
                gEnableStTest = TRUE;
                ST_DBG("Enable ST test!\n");
                break;

            case 'e':
                strcpy(gaDumpFilePath, optarg);
                ST_DBG("gaDumpFilePath:%s\n", gaDumpFilePath);

                gbDumpData = TRUE;
                break;

            case 'E':
                ST_DBG("sensor power on when preview!\n");

                gbSnrPowerOnWhenPreview = TRUE;
                break;

            case 'H':
                gEnableHdr  = TRUE;
                ST_DBG("Enable HDR!\n");
                break;

            case 'L':

                #ifdef CONFIG_SIGMASTAR_CHIP_I6E
                    strcpy(gaLdcBinPath, optarg);
                    ST_DBG("gaLdcBinPath:%s\n", gaLdcBinPath);
                    gbLdcEnable = TRUE;
                #else
                    ST_ERR("Not support LDC!!!\n");
                #endif

            case 'm':
                PARSE_PARM(mult);
                break;

            case 'M':
                mode = strtol(optarg, NULL, 10);
                break;

            case 'N':
                g_device_num = strtol(optarg, NULL, 10);
                break;

            case 'p':
                PARSE_PARM(maxpacket);
                break;

            case 'I':
                PARSE_PARM(intf);
                break;

            case 't':
                trace_level = strtol(optarg, NULL, 10);
                break;

            case 'q':
                g_enable_iqserver = TRUE;
                break;

            case 'T':
                type = strtol(optarg, NULL, 10);
                break;

            case 'Q':
                PARSE_PARM(g_qfactor);
                break;

            case 'i':
                strcpy(g_IspBinPath, optarg);
                ST_DBG("g_IspBinPath:%s\n", g_IspBinPath);
                g_load_iq_bin = TRUE;
                break;

            case 'V':
                g_bEnableVideo = strtol(optarg, NULL, 10);
                break;

            case 'd':
                bEnableAI = TRUE;
                AiDevId = strtol(optarg, NULL, 10);
                break;

            case 'r':
                bEnableMjpegRealtimeMode = TRUE;
                break;

            case 's':
                u32AiSampleRate = strtol(optarg, NULL, 10);
                break;

            case 'O':
                bEnableAO = TRUE;
                break;

            case 'D':
                bEnableAO = TRUE;
                AoDevId = strtol(optarg, NULL, 10);
                break;

            case 'S':
                u32AoSampleRate = strtol(optarg, NULL, 10);
                break;

            case 'h':
                help_message(argv);
                return 0;

            default:
                break;
        }
    }

    if(!checkAudioParam())
    {
        help_message(argv);
        return -1;
    }

    STCHECKRESULT(ST_Sys_Init());
    STCHECKRESULT(ST_AudioModuleInit());
    ST_UVC_SetTraceLevel(trace_level);
    if(g_bEnableVideo)
    {
        if(!gbSnrPowerOnWhenPreview)
        {
            STCHECKRESULT(ST_VideoModuleInit(&g_stConfig));
        }

        ST_UvcInit(g_device_num, maxpacket, mult, burst, intf, mode, type);
    }

    if(gbDumpData && !strcmp(gaDumpFilePath, "null"))
    {
        printf("Please input the dump file path:\n");

        scanf("%s", gaDumpFilePath);

        printf("gaDumpFilePath：%s\n", gaDumpFilePath);
        ST_Flush();
    }

	// STCHECKRESULT(getDetectionManager(&detection_manager));
    // //int type=2;
    // //STCHECKRESULT(initDetection(detection_manager, &pcn_detection_info));
    // int type_person=1;
    // STCHECKRESULT(initDetection(detection_manager, &person_detection_info));
    // // int type_face=0;
    // // STCHECKRESULT(initDetection(detection_manager, &face_detection_info));
    // STCHECKRESULT(startDetect(detection_manager));
    // STCHECKRESULT(StreamInit());
	// printf("=====================================zhe shi zhu hanshu======================================== \n");
    // pthread_create(&g_detectThread, NULL, DetectThread, &type_person);
    // pthread_setname_np(g_detectThread, "Detect_Task");

    // RgnInit();
    while(!g_bExit)
    {
        usleep(100 * 1000);

        if(gEnableStTest)
        {
            ST_Test();
        }
    }

    usleep(100 * 1000);

    if(g_bEnableVideo)
    {
        ST_UvcDeinit();

        if(!gbSnrPowerOnWhenPreview)
        {
            STCHECKRESULT(ST_VideoModuleUnInit(&g_stConfig));
        }
    }
    STCHECKRESULT(ST_AudioModuleUnInit());
    STCHECKRESULT(ST_Sys_Exit());

    return 0;
}

