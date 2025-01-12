#ifndef _PLAYM4_H_ 
#define _PLAYM4_H_

#if defined( _WINDLL)
    #define PLAYM4_API  extern "C" __declspec(dllexport) 
#else 
    #define PLAYM4_API  extern "C"
#endif

//Max channel numbers
#define PLAYM4_MAX_SUPPORTS 500
//Wave coef range;
#define MIN_WAVE_COEF -100
#define MAX_WAVE_COEF 100

//Timer type
#define TIMER_1 1 //Only 16 timers for every process.Default TIMER;
#define TIMER_2 2 //Not limit;But the precision less than TIMER_1; 

//BUFFER AND DATA TYPE
#define BUF_VIDEO_SRC               (1) //mixed input,total src buffer size;splited input,video src buffer size 
#define BUF_AUDIO_SRC               (2) //mixed input,not defined;splited input,audio src buffer size
#define BUF_VIDEO_RENDER            (3) //video render node count 
#define BUF_AUDIO_RENDER            (4) //audio render node count 
#define BUF_VIDEO_DECODED           (5) //video decoded node count to render
#define BUF_AUDIO_DECODED           (6) //audio decoded node count to render
#define BUF_DISPLAY_NODE            (7) //display node

//Error code
#define  PLAYM4_NOERROR                            0    //no error
#define     PLAYM4_PARA_OVER                        1    //input parameter is invalid;
#define  PLAYM4_ORDER_ERROR                        2    //The order of the function to be called is error.
#define     PLAYM4_TIMER_ERROR                        3    //Create multimedia clock failed;
#define  PLAYM4_DEC_VIDEO_ERROR                    4    //Decode video data failed.
#define  PLAYM4_DEC_AUDIO_ERROR                    5    //Decode audio data failed.
#define     PLAYM4_ALLOC_MEMORY_ERROR                6    //Allocate memory failed.
#define  PLAYM4_OPEN_FILE_ERROR                    7    //Open the file failed.
#define  PLAYM4_CREATE_OBJ_ERROR                8    //Create thread or event failed
#define  PLAYM4_CREATE_DDRAW_ERROR                9    //Create DirectDraw object failed.
#define  PLAYM4_CREATE_OFFSCREEN_ERROR            10    //failed when creating off-screen surface.
#define  PLAYM4_BUF_OVER                        11    //buffer is overflow
#define  PLAYM4_CREATE_SOUND_ERROR                12    //failed when creating audio device.    
#define     PLAYM4_SET_VOLUME_ERROR                13    //Set volume failed
#define  PLAYM4_SUPPORT_FILE_ONLY                14    //The function only support play file.
#define  PLAYM4_SUPPORT_STREAM_ONLY                15    //The function only support play stream.
#define  PLAYM4_SYS_NOT_SUPPORT                    16    //System not support.
#define  PLAYM4_FILEHEADER_UNKNOWN                17    //No file header.
#define  PLAYM4_VERSION_INCORRECT                18    //The version of decoder and encoder is not adapted.  
#define  PLAYM4_INIT_DECODER_ERROR                19    //Initialize decoder failed.
#define  PLAYM4_CHECK_FILE_ERROR                20    //The file data is unknown.
#define  PLAYM4_INIT_TIMER_ERROR                21    //Initialize multimedia clock failed.
#define     PLAYM4_BLT_ERROR                        22    //Blt failed.
#define  PLAYM4_UPDATE_ERROR                    23    //Update failed.
#define  PLAYM4_OPEN_FILE_ERROR_MULTI            24   //openfile error, streamtype is multi
#define  PLAYM4_OPEN_FILE_ERROR_VIDEO            25   //openfile error, streamtype is video
#define  PLAYM4_JPEG_COMPRESS_ERROR                26   //JPEG compress error
#define  PLAYM4_EXTRACT_NOT_SUPPORT                27    //Don't support the version of this file.
#define  PLAYM4_EXTRACT_DATA_ERROR                28    //extract video data failed.
#define  PLAYM4_SECRET_KEY_ERROR                29    //Secret key is error //add 20071218
#define  PLAYM4_DECODE_KEYFRAME_ERROR            30   //add by hy 20090318
#define  PLAYM4_NEED_MORE_DATA                    31   //add by hy 20100617
#define  PLAYM4_INVALID_PORT                    32    //add by cj 20100913
#define  PLAYM4_NOT_FIND                        33    //add by cj 20110428
#define  PLAYM4_NEED_LARGER_BUFFER              34  //add by pzj 20130528
#define  PLAYM4_FAIL_UNKNOWN                    99   //Fail, but the reason is unknown;    

//���۹��ܴ�����
#define PLAYM4_FEC_ERR_ENABLEFAIL                100 // ����ģ�����ʧ��
#define PLAYM4_FEC_ERR_NOTENABLE                101 // ����ģ��û�м���
#define PLAYM4_FEC_ERR_NOSUBPORT                102 // �Ӷ˿�û�з���
#define PLAYM4_FEC_ERR_PARAMNOTINIT                103 // û�г�ʼ����Ӧ�˿ڵĲ���
#define PLAYM4_FEC_ERR_SUBPORTOVER                104 // �Ӷ˿��Ѿ�����
#define PLAYM4_FEC_ERR_EFFECTNOTSUPPORT            105 // �ð�װ��ʽ������Ч����֧��
#define PLAYM4_FEC_ERR_INVALIDWND                106 // �Ƿ��Ĵ���
#define PLAYM4_FEC_ERR_PTZOVERFLOW                107 // PTZλ��Խ��
#define PLAYM4_FEC_ERR_RADIUSINVALID            108 // Բ�Ĳ����Ƿ�
#define PLAYM4_FEC_ERR_UPDATENOTSUPPORT            109 // ָ���İ�װ��ʽ�ͽ���Ч�����ò������²�֧��
#define PLAYM4_FEC_ERR_NOPLAYPORT                110 // ���ſ�˿�û������
#define PLAYM4_FEC_ERR_PARAMVALID                111 // ����Ϊ��
#define PLAYM4_FEC_ERR_INVALIDPORT                112 // �Ƿ��Ӷ˿�
#define PLAYM4_FEC_ERR_PTZZOOMOVER                113 // PTZ������ΧԽ��
#define PLAYM4_FEC_ERR_OVERMAXPORT                114  // ����ͨ�����ͣ����֧�ֵĽ���ͨ��Ϊ�ĸ�
#define PLAYM4_FEC_ERR_ENABLED                  115  //�ö˿��Ѿ�����������ģ��
#define PLAYM4_FEC_ERR_D3DACCENOTENABLE            116 // D3D����û�п���


//Max display regions.
#define MAX_DISPLAY_WND 4

//Display type
#define DISPLAY_NORMAL                0x00000001   
#define DISPLAY_QUARTER                0x00000002
#define DISPLAY_YC_SCALE            0x00000004    //add by gb 20091116
#define DISPLAY_NOTEARING            0x00000008
//Display buffers
#define MAX_DIS_FRAMES 50
#define MIN_DIS_FRAMES 1

//Locate by
#define BY_FRAMENUM  1
#define BY_FRAMETIME 2

//Source buffer
#define SOURCE_BUF_MAX    1024*100000
#define SOURCE_BUF_MIN    1024*50

//Stream type
#define STREAME_REALTIME 0
#define STREAME_FILE     1

//frame type
#define T_AUDIO16    101
#define T_AUDIO8    100
#define T_UYVY        1
#define T_YV12        3
#define T_RGB32        7

//capability
#define SUPPORT_DDRAW        1 
#define SUPPORT_BLT         2 
#define SUPPORT_BLTFOURCC   4 
#define SUPPORT_BLTSHRINKX  8 
#define SUPPORT_BLTSHRINKY  16
#define SUPPORT_BLTSTRETCHX 32
#define SUPPORT_BLTSTRETCHY 64
#define SUPPORT_SSE         128
#define SUPPORT_MMX            256 

// ���º궨������HIK_MEDIAINFO�ṹ
#define FOURCC_HKMI            0x484B4D49    // "HKMI" HIK_MEDIAINFO�ṹ���
// ϵͳ��װ��ʽ    
#define SYSTEM_NULL            0x0                // û��ϵͳ�㣬����Ƶ������Ƶ��    
#define SYSTEM_HIK          0x1                // �����ļ���
#define SYSTEM_MPEG2_PS     0x2                // PS��װ
#define SYSTEM_MPEG2_TS     0x3                // TS��װ
#define SYSTEM_RTP          0x4                // rtp��װ
#define SYSTEM_RTPHIK       0x401                // rtp��װ

// ��Ƶ��������
#define VIDEO_NULL          0x0 // û����Ƶ
#define VIDEO_H264          0x1 // ����H.264
#define VIDEO_MPEG4         0x3 // ��׼MPEG4
#define VIDEO_MJPEG            0x4
#define VIDEO_AVC264        0x0100

// ��Ƶ��������
#define AUDIO_NULL          0x0000 // û����Ƶ
#define AUDIO_ADPCM         0x1000 // ADPCM 
#define AUDIO_MPEG          0x2000 // MPEG ϵ����Ƶ��������������Ӧ����MPEG��Ƶ
#define AUDIO_AAC            0X2001 // AAC ����
// Gϵ����Ƶ
#define AUDIO_RAW_DATA8        0x7000      //������Ϊ8k��ԭʼ����
#define AUDIO_RAW_UDATA16    0x7001      //������Ϊ16k��ԭʼ���ݣ���L16
#define AUDIO_G711_U        0x7110
#define AUDIO_G711_A        0x7111
#define AUDIO_G722_1        0x7221
#define AUDIO_G723_1        0x7231
#define AUDIO_G726_U        0x7260
#define AUDIO_G726_A        0x7261
#define AUDIO_G726_16       0x7262 
#define AUDIO_G729          0x7290
#define AUDIO_AMR_NB        0x3000

#define SYNCDATA_VEH        1 //ͬ������:������Ϣ    
#define SYNCDATA_IVS        2 //ͬ������:������Ϣ

//motion flow type
#define    MOTION_FLOW_NONE            0
#define MOTION_FLOW_CPU                1
#define MOTION_FLOW_GPU                2

//����Ƶ��������
#define ENCRYPT_AES_3R_VIDEO     1 
#define ENCRYPT_AES_10R_VIDEO    2 
#define ENCRYPT_AES_3R_AUDIO     1  
#define ENCRYPT_AES_10R_AUDIO    2

typedef struct tagSystemTime
{
    short wYear;
    short wMonth;
    short wDayOfWeek;
    short wDay;
    short wHour;
    short wMinute;
    short wSecond;
    short wMilliseconds;
}SYSTEMTIME;

typedef struct tagHKRect
{
    unsigned long left;
    unsigned long top;
    unsigned long right;
    unsigned long bottom;
}HKRECT;

//Frame position
typedef struct{
    long nFilePos;
    long nFrameNum;
    long nFrameTime;
    long nErrorFrameNum;
    SYSTEMTIME *pErrorTime;
    long nErrorLostFrameNum;
    long nErrorFrameSize;
}FRAME_POS,*PFRAME_POS;


//Frame Info
typedef struct{
    long nWidth;
    long nHeight;
    long nStamp;
    long nType;
    long nFrameRate;
    DWORD dwFrameNum;
}FRAME_INFO;

typedef struct
{
    long         nPort;        //ͨ����
    char         *pBuf;        //���صĵ�һ·ͼ������ָ��
    unsigned int nBufLen;      //���صĵ�һ·ͼ�����ݴ�С
    char         *pBuf1;       //���صĵڶ�·ͼ������ָ��
    unsigned int nBufLen1;     //���صĵڶ�·ͼ�����ݴ�С
    char         *pBuf2;       //���صĵ���·ͼ������ָ��
    unsigned int nBufLen2;     //���صĵ���·ͼ�����ݴ�С
    unsigned int nWidth;       //�����
    unsigned int nHeight;      //�����
    unsigned int nStamp;       //ʱ����Ϣ����λ����
    unsigned int nType;        //��������
    void         *pUser;       //�û�����
    unsigned int reserved[4];  //����
}DISPLAY_INFO_YUV;

//Frame 
typedef struct{
    char *pDataBuf;
    long  nSize;
    long  nFrameNum;
    BOOL  bIsAudio;
    long  nReserved;
}FRAME_TYPE;

//Watermark Info    //add by gb 080119
typedef struct{
    char *pDataBuf;
    long  nSize;
    long  nFrameNum;
    BOOL  bRsaRight;
    long  nReserved;
}WATERMARK_INFO;

typedef struct SYNCDATA_INFO 
{
    DWORD dwDataType;        //����������ͬ���ĸ�����Ϣ���ͣ�Ŀǰ�У�������Ϣ��������Ϣ
    DWORD dwDataLen;        //������Ϣ���ݳ���
    BYTE* pData;            //ָ������Ϣ���ݽṹ��ָ��,����IVS_INFO�ṹ
} SYNCDATA_INFO;

#ifndef _HIK_MEDIAINFO_FLAG_
#define _HIK_MEDIAINFO_FLAG_
typedef struct _HIK_MEDIAINFO_                // modified by gb 080425
{
    unsigned int    media_fourcc;            // "HKMI": 0x484B4D49 Hikvision Media Information
    unsigned short  media_version;            // �汾�ţ�ָ����Ϣ�ṹ�汾�ţ�ĿǰΪ0x0101,��1.01�汾��01�����汾�ţ�01���Ӱ汾�š�
    unsigned short  device_id;                // �豸ID�����ڸ���/����            
    
    unsigned short  system_format;          // ϵͳ��װ��
    unsigned short  video_format;           // ��Ƶ��������

    unsigned short  audio_format;           // ��Ƶ��������
    unsigned char   audio_channels;         // ͨ����  
    unsigned char   audio_bits_per_sample;  // ��λ��
    unsigned int    audio_samplesrate;      // ������ 
    unsigned int    audio_bitrate;          // ѹ����Ƶ����,��λ��bit

    unsigned int    reserved[4];            // ����
}HIK_MEDIAINFO;
#endif

typedef struct  
{
    long nPort;
    char * pBuf;
    long nBufLen;
    long nWidth;
    long nHeight;
    long nStamp;
    long nType;
    long nUser;
}DISPLAY_INFO;

typedef struct
{
    long nPort;
    char *pVideoBuf;
    long nVideoBufLen;
    char *pPriBuf;
    long nPriBufLen;
    long nWidth;
    long nHeight;
    long nStamp;
    long nType;
    long nUser;
}DISPLAY_INFOEX;

typedef struct PLAYM4_SYSTEM_TIME //����ʱ�� 
{
    DWORD dwYear;    //��
    DWORD dwMon;    //��
    DWORD dwDay;    //��
    DWORD dwHour;    //ʱ
    DWORD dwMin;    //��
    DWORD dwSec;    //��
    DWORD dwMs;        //����
} PLAYM4_SYSTEM_TIME;

//ENCRYPT Info
typedef struct{
    long nVideoEncryptType;  //��Ƶ��������
    long nAudioEncryptType;  //��Ƶ��������
    long nSetSecretKey;      //�Ƿ����ã�1��ʾ������Կ��0��ʾû��������Կ
}ENCRYPT_INFO;

//////////////////////////////////////////////////////////////////////////////
//API
//////////////////////////////////////////////////////////////////////////////

////////////////ver 1.0///////////////////////////////////////
//Initialize DirecDraw.Now invalid.
PLAYM4_API BOOL __stdcall  PlayM4_InitDDraw(HWND hWnd);
//Release directDraw; Now invalid.
PLAYM4_API BOOL __stdcall PlayM4_RealeseDDraw();
PLAYM4_API BOOL __stdcall PlayM4_OpenFile(LONG nPort,LPSTR sFileName);
PLAYM4_API BOOL __stdcall PlayM4_CloseFile(LONG nPort);
PLAYM4_API BOOL __stdcall PlayM4_Play(LONG nPort, HWND hWnd);
PLAYM4_API BOOL __stdcall PlayM4_Stop(LONG nPort);
PLAYM4_API BOOL __stdcall PlayM4_Pause(LONG nPort,DWORD nPause);
PLAYM4_API BOOL __stdcall PlayM4_Fast(LONG nPort);
PLAYM4_API BOOL __stdcall PlayM4_Slow(LONG nPort);
PLAYM4_API BOOL __stdcall PlayM4_OneByOne(LONG nPort);
PLAYM4_API BOOL __stdcall PlayM4_SetPlayPos(LONG nPort,float fRelativePos);
PLAYM4_API float __stdcall PlayM4_GetPlayPos(LONG nPort);
PLAYM4_API BOOL __stdcall PlayM4_SetFileEndMsg(LONG nPort,HWND hWnd,UINT nMsg);
PLAYM4_API BOOL __stdcall PlayM4_SetVolume(LONG nPort,WORD nVolume);
PLAYM4_API BOOL __stdcall PlayM4_StopSound();
PLAYM4_API BOOL __stdcall PlayM4_PlaySound(LONG nPort);
PLAYM4_API BOOL __stdcall PlayM4_OpenStream(LONG nPort,PBYTE pFileHeadBuf,DWORD nSize,DWORD nBufPoolSize);
PLAYM4_API BOOL __stdcall PlayM4_InputData(LONG nPort,PBYTE pBuf,DWORD nSize);
PLAYM4_API BOOL __stdcall PlayM4_CloseStream(LONG nPort);
PLAYM4_API int  __stdcall  PlayM4_GetCaps();
PLAYM4_API DWORD __stdcall PlayM4_GetFileTime(LONG nPort);
PLAYM4_API DWORD __stdcall PlayM4_GetPlayedTime(LONG nPort);
PLAYM4_API DWORD __stdcall PlayM4_GetPlayedFrames(LONG nPort);

//23
////////////////ver 2.0 added///////////////////////////////////////
PLAYM4_API BOOL __stdcall    PlayM4_SetDecCallBack(LONG nPort,void (CALLBACK* DecCBFun)(long nPort,char * pBuf,long nSize,FRAME_INFO * pFrameInfo, long nReserved1,long nReserved2));
PLAYM4_API BOOL __stdcall    PlayM4_SetDisplayCallBackYUV(LONG nPort, void (CALLBACK* DisplayCBFun)(DISPLAY_INFO_YUV *pstDisplayInfo), BOOL bTrue, void* pUser);
PLAYM4_API BOOL __stdcall    PlayM4_SetDisplayCallBack(LONG nPort,void (CALLBACK* DisplayCBFun)(long nPort,char * pBuf,long nSize,long nWidth,long nHeight,long nStamp,long nType,long nReserved));
PLAYM4_API BOOL __stdcall    PlayM4_ConvertToBmpFile(char * pBuf,long nSize,long nWidth,long nHeight,long nType,char *sFileName);
PLAYM4_API DWORD __stdcall    PlayM4_GetFileTotalFrames(LONG nPort);
PLAYM4_API DWORD __stdcall    PlayM4_GetCurrentFrameRate(LONG nPort);
PLAYM4_API DWORD __stdcall    PlayM4_GetPlayedTimeEx(LONG nPort);
PLAYM4_API BOOL __stdcall    PlayM4_SetPlayedTimeEx(LONG nPort,DWORD nTime);
PLAYM4_API DWORD __stdcall    PlayM4_GetCurrentFrameNum(LONG nPort);
PLAYM4_API BOOL __stdcall    PlayM4_SetStreamOpenMode(LONG nPort,DWORD nMode);
PLAYM4_API DWORD __stdcall    PlayM4_GetFileHeadLength();
PLAYM4_API DWORD __stdcall    PlayM4_GetSdkVersion();
//11
////////////////ver 2.2 added///////////////////////////////////////
PLAYM4_API DWORD __stdcall  PlayM4_GetLastError(LONG nPort);
PLAYM4_API BOOL __stdcall PlayM4_RefreshPlay(LONG nPort);
PLAYM4_API BOOL __stdcall PlayM4_SetOverlayMode(LONG nPort,BOOL bOverlay,COLORREF colorKey);
PLAYM4_API BOOL __stdcall PlayM4_GetPictureSize(LONG nPort,LONG *pWidth,LONG *pHeight);
PLAYM4_API BOOL __stdcall PlayM4_SetPicQuality(LONG nPort,BOOL bHighQuality);
PLAYM4_API BOOL __stdcall PlayM4_PlaySoundShare(LONG nPort);
PLAYM4_API BOOL __stdcall PlayM4_StopSoundShare(LONG nPort);
//7
////////////////ver 2.4 added///////////////////////////////////////
PLAYM4_API LONG __stdcall PlayM4_GetStreamOpenMode(LONG nPort);
PLAYM4_API LONG __stdcall PlayM4_GetOverlayMode(LONG nPort);
PLAYM4_API COLORREF __stdcall PlayM4_GetColorKey(LONG nPort);
PLAYM4_API WORD __stdcall PlayM4_GetVolume(LONG nPort);
PLAYM4_API BOOL __stdcall PlayM4_GetPictureQuality(LONG nPort,BOOL *bHighQuality);
PLAYM4_API DWORD __stdcall PlayM4_GetSourceBufferRemain(LONG nPort);
PLAYM4_API BOOL __stdcall PlayM4_ResetSourceBuffer(LONG nPort);
PLAYM4_API BOOL __stdcall PlayM4_SetSourceBufCallBack(LONG nPort,DWORD nThreShold,void (CALLBACK * SourceBufCallBack)(long nPort,DWORD nBufSize,DWORD dwUser,void*pResvered),DWORD dwUser,void *pReserved);
PLAYM4_API BOOL __stdcall PlayM4_ResetSourceBufFlag(LONG nPort);
PLAYM4_API BOOL __stdcall PlayM4_SetDisplayBuf(LONG nPort,DWORD nNum);
PLAYM4_API DWORD __stdcall PlayM4_GetDisplayBuf(LONG nPort);
PLAYM4_API BOOL __stdcall PlayM4_OneByOneBack(LONG nPort);
PLAYM4_API BOOL __stdcall PlayM4_SetFileRefCallBack(LONG nPort, void (__stdcall *pFileRefDone)(DWORD nPort,DWORD nUser),DWORD nUser);
PLAYM4_API BOOL __stdcall PlayM4_SetCurrentFrameNum(LONG nPort,DWORD nFrameNum);
PLAYM4_API BOOL __stdcall PlayM4_GetKeyFramePos(LONG nPort,DWORD nValue, DWORD nType, PFRAME_POS pFramePos);
PLAYM4_API BOOL __stdcall PlayM4_GetNextKeyFramePos(LONG nPort,DWORD nValue, DWORD nType, PFRAME_POS pFramePos);
#if (WINVER >= 0x0400)
//Note: These funtion must be builded under win2000 or above with Microsoft Platform sdk.
//        You can download the sdk from "http://www.microsoft.com/msdownload/platformsdk/sdkupdate/";
PLAYM4_API BOOL __stdcall PlayM4_InitDDrawDevice();
PLAYM4_API void __stdcall PlayM4_ReleaseDDrawDevice();
PLAYM4_API DWORD __stdcall PlayM4_GetDDrawDeviceTotalNums();
PLAYM4_API BOOL __stdcall PlayM4_SetDDrawDevice(LONG nPort,DWORD nDeviceNum);
//PLAYM4_API BOOL __stdcall PlayM4_GetDDrawDeviceInfo(DWORD nDeviceNum,LPSTR  lpDriverDescription,DWORD nDespLen,LPSTR lpDriverName ,DWORD nNameLen,HMONITOR *hhMonitor);
PLAYM4_API int  __stdcall  PlayM4_GetCapsEx(DWORD nDDrawDeviceNum);
#endif
PLAYM4_API BOOL __stdcall PlayM4_ThrowBFrameNum(LONG nPort,DWORD nNum);
//23
////////////////ver 2.5 added///////////////////////////////////////
PLAYM4_API BOOL __stdcall PlayM4_SetDisplayType(LONG nPort,LONG nType);
PLAYM4_API long __stdcall PlayM4_GetDisplayType(LONG nPort);
//2
////////////////ver 3.0 added///////////////////////////////////////
PLAYM4_API BOOL __stdcall PlayM4_SetDecCBStream(LONG nPort,DWORD nStream);
PLAYM4_API BOOL __stdcall PlayM4_SetDisplayRegion(LONG nPort,DWORD nRegionNum, HKRECT *pSrcRect, HWND hDestWnd, BOOL bEnable);
PLAYM4_API BOOL __stdcall PlayM4_RefreshPlayEx(LONG nPort,DWORD nRegionNum);
#if (WINVER >= 0x0400)
//Note: The funtion must be builded under win2000 or above with Microsoft Platform sdk.
//        You can download the sdk from http://www.microsoft.com/msdownload/platformsdk/sdkupdate/;
PLAYM4_API BOOL __stdcall PlayM4_SetDDrawDeviceEx(LONG nPort,DWORD nRegionNum,DWORD nDeviceNum);
#endif
//4
/////////////////v3.2 added/////////////////////////////////////////

PLAYM4_API BOOL __stdcall PlayM4_GetRefValue(LONG nPort,BYTE *pBuffer, DWORD *pSize);
PLAYM4_API BOOL __stdcall PlayM4_SetRefValue(LONG nPort,BYTE *pBuffer, DWORD nSize);
PLAYM4_API BOOL __stdcall PlayM4_OpenStreamEx(LONG nPort,PBYTE pFileHeadBuf,DWORD nSize,DWORD nBufPoolSize);
PLAYM4_API BOOL __stdcall PlayM4_CloseStreamEx(LONG nPort);
PLAYM4_API BOOL __stdcall PlayM4_InputVideoData(LONG nPort,PBYTE pBuf,DWORD nSize);
PLAYM4_API BOOL __stdcall PlayM4_InputAudioData(LONG nPort,PBYTE pBuf,DWORD nSize);
PLAYM4_API BOOL __stdcall PlayM4_RegisterDrawFun(LONG nPort,void (CALLBACK* DrawFun)(long nPort,HDC hDc,LONG nUser),LONG nUser);
PLAYM4_API BOOL __stdcall PlayM4_RigisterDrawFun(LONG nPort,void (CALLBACK* DrawFun)(long nPort,HDC hDc,LONG nUser),LONG nUser);
//8
//////////////////v3.4/////////////////////////////////////////////////////
PLAYM4_API BOOL __stdcall PlayM4_SetTimerType(LONG nPort,DWORD nTimerType,DWORD nReserved);
PLAYM4_API BOOL __stdcall PlayM4_GetTimerType(LONG nPort,DWORD *pTimerType,DWORD *pReserved);
PLAYM4_API BOOL __stdcall PlayM4_ResetBuffer(LONG nPort,DWORD nBufType);
PLAYM4_API DWORD __stdcall PlayM4_GetBufferValue(LONG nPort,DWORD nBufType);

//////////////////V3.6/////////////////////////////////////////////////////////
PLAYM4_API BOOL __stdcall PlayM4_AdjustWaveAudio(LONG nPort,LONG nCoefficient);
PLAYM4_API BOOL __stdcall PlayM4_SetVerifyCallBack(LONG nPort, DWORD nBeginTime, DWORD nEndTime, void (__stdcall * funVerify)(long nPort, FRAME_POS * pFilePos, DWORD bIsVideo, DWORD nUser),  DWORD  nUser);
PLAYM4_API BOOL __stdcall PlayM4_SetAudioCallBack(LONG nPort, void (__stdcall * funAudio)(long nPort, char * pAudioBuf, long nSize, long nStamp, long nType, long nUser), long nUser);
PLAYM4_API BOOL __stdcall PlayM4_SetEncTypeChangeCallBack(LONG nPort,void(CALLBACK *funEncChange)(long nPort,long nUser),long nUser);
PLAYM4_API BOOL __stdcall PlayM4_SetColor(LONG nPort, DWORD nRegionNum, int nBrightness, int nContrast, int nSaturation, int nHue);
PLAYM4_API BOOL __stdcall PlayM4_GetColor(LONG nPort, DWORD nRegionNum, int *pBrightness, int *pContrast, int *pSaturation, int *pHue);
PLAYM4_API BOOL __stdcall PlayM4_SetEncChangeMsg(LONG nPort,HWND hWnd,UINT nMsg);
PLAYM4_API BOOL __stdcall PlayM4_GetOriginalFrameCallBack(LONG nPort, BOOL bIsChange,BOOL bNormalSpeed,long nStartFrameNum,long nStartStamp,long nFileHeader,void(CALLBACK *funGetOrignalFrame)(long nPort,FRAME_TYPE *frameType, long nUser),long nUser);
PLAYM4_API BOOL __stdcall PlayM4_GetFileSpecialAttr(LONG nPort, DWORD *pTimeStamp,DWORD *pFileNum ,DWORD *pReserved);
PLAYM4_API DWORD __stdcall PlayM4_GetSpecialData(LONG nPort);
PLAYM4_API BOOL __stdcall PlayM4_SetCheckWatermarkCallBack(LONG nPort,void(CALLBACK* funCheckWatermark)(long nPort,WATERMARK_INFO* pWatermarkInfo,DWORD nUser),DWORD nUser);
PLAYM4_API BOOL __stdcall PlayM4_SetImageSharpen(LONG nPort,DWORD nLevel);
PLAYM4_API BOOL __stdcall PlayM4_SetDecodeFrameType(LONG nPort,DWORD nFrameType);
PLAYM4_API BOOL __stdcall PlayM4_SetPlayMode(LONG nPort,BOOL bNormal);
PLAYM4_API BOOL __stdcall PlayM4_SetOverlayFlipMode(LONG nPort,BOOL bTrue);
PLAYM4_API BOOL __stdcall PlayM4_SetOverlayPriInfoFlag(LONG nPort, DWORD nIntelType, BOOL bTrue,const char *pFontPath);

//PLAYM4_API DWORD __stdcall PlayM4_GetAbsFrameNum(LONG nPort); 

//////////////////V4.7.0.0//////////////////////////////////////////////////////
////convert yuv to jpeg
PLAYM4_API BOOL __stdcall PlayM4_ConvertToJpegFile(char * pBuf,long nSize,long nWidth,long nHeight,long nType,char *sFileName);
PLAYM4_API BOOL __stdcall PlayM4_SetJpegQuality(long nQuality);
//set deflash
PLAYM4_API BOOL __stdcall PlayM4_SetDeflash(LONG nPort,BOOL bDefalsh);
//PLAYM4_API BOOL __stdcall PlayM4_SetDecCallBackEx(LONG nPort,void (CALLBACK* DecCBFun)(long nPort,char * pBuf,long nSize,FRAME_INFO * pFrameInfo, long nReserved1,long nReserved2), char* pDest, long nDestSize);
//////////////////V4.8.0.0/////////////////////////////////////////////////////////
//check discontinuous frame number as error data?
PLAYM4_API BOOL __stdcall PlayM4_CheckDiscontinuousFrameNum(LONG nPort, BOOL bCheck);
//get bmp or jpeg
PLAYM4_API BOOL __stdcall PlayM4_GetBMP(LONG nPort,PBYTE pBitmap,DWORD nBufSize,DWORD* pBmpSize);
PLAYM4_API BOOL __stdcall PlayM4_GetJPEG(LONG nPort,PBYTE pJpeg,DWORD nBufSize,DWORD* pJpegSize);
//dec call back mend
PLAYM4_API BOOL __stdcall PlayM4_SetDecCallBackMend(LONG nPort,void (CALLBACK* DecCBFun)(long nPort,char * pBuf,long nSize,FRAME_INFO * pFrameInfo, long nUser,long nReserved2), long nUser);
PLAYM4_API BOOL __stdcall PlayM4_SetSecretKey(LONG nPort, LONG lKeyType, char *pSecretKey, LONG lKeyLen);

// add by gb 2007-12-23
PLAYM4_API BOOL __stdcall PlayM4_SetFileEndCallback(LONG nPort, void(CALLBACK*FileEndCallback)(long nPort, void *pUser), void *pUser);

// add by gb 080131 version 4.9.0.1
PLAYM4_API BOOL __stdcall PlayM4_GetPort(LONG* nPort);
PLAYM4_API BOOL __stdcall PlayM4_FreePort(LONG nPort);
PLAYM4_API BOOL __stdcall PlayM4_SetDisplayCallBackEx(LONG nPort,void (CALLBACK* DisplayCBFun)(DISPLAY_INFO *pstDisplayInfo), long nUser);
PLAYM4_API BOOL __stdcall PlayM4_SkipErrorData(LONG nPort, BOOL bSkip);
PLAYM4_API BOOL __stdcall PlayM4_SetDecCallBackExMend(LONG nPort, void (CALLBACK* DecCBFun)(long nPort, char* pBuf, long nSize, FRAME_INFO* pFrameInfo, 
                                                      long nUser, long nReserved2), char* pDest, long nDestSize, long nUser);
//reverse play add by chenjie 110609
PLAYM4_API BOOL __stdcall PlayM4_ReversePlay(LONG nPort);
PLAYM4_API BOOL __stdcall PlayM4_GetSystemTime(LONG nPort, PLAYM4_SYSTEM_TIME *pstSystemTime);

//PLAYM4_API BOOL __stdcall PlayM4_SetDecodeERC(long nPort, unsigned int nLevel);

#ifndef PLAYM4_SESSION_INFO_TAG
#define PLAYM4_SESSION_INFO_TAG
//nProtocolType
#define PLAYM4_PROTOCOL_RTSP    1
//nSessionInfoType
#define PLAYM4_SESSION_INFO_SDP 1

typedef struct _PLAYM4_SESSION_INFO_     //������Ϣ�ṹ
{
      int            nSessionInfoType;   //������Ϣ���ͣ�����SDP�����纣��˽����Ϣͷ
      int            nSessionInfoLen;    //������Ϣ����
      unsigned char* pSessionInfoData;   //������Ϣ����

} PLAYM4_SESSION_INFO;
#endif

PLAYM4_API BOOL __stdcall PlayM4_OpenStreamAdvanced(LONG nPort, int nProtocolType, PLAYM4_SESSION_INFO* pstSessionInfo, DWORD nBufPoolSize);

#define R_ANGLE_0   -1  //����ת
#define R_ANGLE_L90  0  //������ת90��
#define R_ANGLE_R90  1  //������ת90��
#define R_ANGLE_180  2  //��ת180��

PLAYM4_API BOOL __stdcall PlayM4_SetRotateAngle(LONG nPort, DWORD nRegionNum, DWORD dwType);

#ifndef PLAYM4_ADDITION_INFO_TAG
#define PLAYM4_ADDITION_INFO_TAG
typedef struct _PLAYM4_ADDITION_INFO_     //������Ϣ�ṹ
{
    BYTE*   pData;            //��������
    DWORD   dwDatalen;        //�������ݳ���
    DWORD    dwDataType;        //��������
    DWORD    dwTimeStamp;    //���ʱ���
} PLAYM4_ADDITION_INFO;
#endif

//dwGroupIndex ��Լ��ȡֵ0~3����һ�汾ȡ��ͬ��ֻ��ͬ��closestream����
PLAYM4_API BOOL __stdcall PlayM4_SetSycGroup(LONG nPort, DWORD dwGroupIndex);
//�ݲ�ʵ�ִ˺�����ͬ�������õ���ʼʱ�䲻һ�£�����С��ʱ����Ϊ������㣬ͬһ���ֻ��һ·
PLAYM4_API BOOL __stdcall PlayM4_SetSycStartTime(LONG nPort, PLAYM4_SYSTEM_TIME *pstSystemTime);


// ����ʵ��������صĽӿ�
#ifndef FISH_EYE_TAG
#define FISH_EYE_TAG

// ��װ����
typedef enum tagFECPlaceType
{
    FEC_PLACE_WALL    = 0x1,        // ��װ��ʽ        (����ˮƽ)
    FEC_PLACE_FLOOR   = 0x2,        // ���氲װ        (��������)
    FEC_PLACE_CEILING = 0x3,        // ��װ��ʽ        (��������)

}FECPLACETYPE;

typedef enum tagFECCorrectType
{
    FEC_CORRECT_PTZ = 0x100,        // PTZ
    FEC_CORRECT_180 = 0x200,        // 180�Ƚ���  ����Ӧ2P��
    FEC_CORRECT_360 = 0x300,        // 360ȫ������ ����Ӧ1P��
    FEC_CORRECT_LAT = 0x400         //γ��չ��

}FECCORRECTTYPE;

typedef struct tagCycleParam
{
    float    fRadiusLeft;    // Բ�������X����
    float    fRadiusRight;    // Բ�����ұ�X����
    float   fRadiusTop;        // Բ�����ϱ�Y����
    float   fRadiusBottom;    // Բ�����±�Y����

}CYCLEPARAM;

typedef struct tagPTZParam
{
    float fPTZPositionX;        // PTZ ��ʾ������λ�� X����
    float fPTZPositionY;        // PTZ ��ʾ������λ�� Y����    

}PTZPARAM;


// ������
/*********************************************
     

 ********************************************/


// ���±�Ǳ�������
 

#define         FEC_UPDATE_RADIUS             0x1
#define         FEC_UPDATE_PTZZOOM             0x2
#define         FEC_UPDATE_WIDESCANOFFSET     0x4
#define         FEC_UPDATE_PTZPARAM             0x8


typedef struct tagFECParam
{

    
    unsigned int     nUpDateType;            // ���µ�����

    unsigned int    nPlaceAndCorrect;        // ��װ��ʽ�ͽ�����ʽ��ֻ�����ڻ�ȡ��SetParam��ʱ����Ч,��ֵ��ʾ��װ��ʽ�ͽ�����ʽ�ĺ�

    PTZPARAM        stPTZParam;                // PTZ У���Ĳ���

    CYCLEPARAM        stCycleParam;            // ����ͼ��Բ�Ĳ���

    float            fZoom;                    // PTZ ��ʾ�ķ�Χ����

    float            fWideScanOffset;        // 180����360��У����ƫ�ƽǶ�

    int                nResver[16];            // �����ֶ�

}FISHEYEPARAM;

typedef void (__stdcall * FISHEYE_CallBack )( void* pUser  , unsigned int  nSubPort , unsigned int nCBType , void * hDC ,   unsigned int nWidth , unsigned int nHeight); 

#endif
// ��������
PLAYM4_API BOOL __stdcall PlayM4_FEC_Enable(LONG nPort);

// �ر�����ģ��
PLAYM4_API BOOL __stdcall PlayM4_FEC_Disable(LONG nPort);

// ��ȡ���۽��������Ӷ˿� [1~31] 
PLAYM4_API BOOL  __stdcall PlayM4_FEC_GetPort(LONG nPort, unsigned int* nSubPort,FECPLACETYPE emPlaceType,FECCORRECTTYPE emCorrectType);

// ɾ�����۽��������Ӷ˿�
PLAYM4_API BOOL __stdcall PlayM4_FEC_DelPort(LONG nPort , unsigned int nSubPort);

// �������۽�������
PLAYM4_API BOOL __stdcall PlayM4_FEC_SetParam(LONG nPort , unsigned int nSubPort , FISHEYEPARAM * pPara);

// ��ȡ���۽�������
PLAYM4_API BOOL __stdcall PlayM4_FEC_GetParam(LONG nPort , unsigned int nSubPort , FISHEYEPARAM * pPara);

// ������ʾ���ڣ�������ʱ�л�
PLAYM4_API BOOL __stdcall PlayM4_FEC_SetWnd(LONG nPort , unsigned int nSubPort , void * hWnd);

// �������۴��ڵĻ�ͼ�ص�
PLAYM4_API BOOL __stdcall PlayM4_FEC_SetCallBack(LONG nPort , unsigned int nSubPort , FISHEYE_CallBack cbFunc , void * pUser);

//motionflow
PLAYM4_API BOOL __stdcall PlayM4_MotionFlow(LONG nPort, DWORD dwAdjustType);


//ͼ����ǿ���
#ifndef PLAYM4_HIKVIE_TAG
#define PLAYM4_HIKVIE_TAG

typedef struct _PLAYM4_VIE_DYNPARAM_
{
    int moduFlag;      //���õ��㷨����ģ�飬��PLAYM4_VIE_MODULES�ж���
    //�� PLAYM4_VIE_MODU_ADJ | PLAYM4_VIE_MODU_EHAN
    //ģ�����ú󣬱���������Ӧ�Ĳ�����
    //PLAYM4_VIE_MODU_ADJ
    int brightVal;     //���ȵ���ֵ��[-255, 255]
    int contrastVal;   //�Աȶȵ���ֵ��[-256, 255]
    int colorVal;      //���Ͷȵ���ֵ��[-256, 255]
    //PLAYM4_VIE_MODU_EHAN
    int toneScale;     //�˲���Χ��[0, 100]
    int toneGain;      //�Աȶȵ��ڣ�ȫ�ֶԱȶ�����ֵ��[-256, 255]
    int toneOffset;    //���ȵ��ڣ�����ƽ��ֵƫ�ƣ�[-255, 255]
    int toneColor;     //��ɫ���ڣ���ɫ����ֵ��[-256, 255]
    //PLAYM4_VIE_MODU_DEHAZE
    int dehazeLevel;   //ȥ��ǿ�ȣ�[0, 255]
    int dehazeTrans;   //͸��ֵ��[0, 255]
    int dehazeBright;  //���Ȳ�����[0, 255]
    //PLAYM4_VIE_MODU_DENOISE
    int denoiseLevel;  //ȥ��ǿ�ȣ�[0, 255]
    //PLAYM4_VIE_MODU_SHARPEN
    int usmAmount;     //��ǿ�ȣ�[0, 255]
    int usmRadius;     //�񻯰뾶��[1, 15]
    int usmThreshold;  //����ֵ��[0, 255]
    //PLAYM4_VIE_MODU_DEBLOCK
    int deblockLevel;  //ȥ��ǿ�ȣ�[0, 100]
    //PLAYM4_VIE_MODU_LENS
    int lensWarp;      //��������[-256, 255]
    int lensZoom;      //��������[-256, 255]
    //PLAYM4_VIE_MODU_CRB
    //����Ӧ����
} PLAYM4_VIE_PARACONFIG;

typedef enum _PLAYM4_VIE_MODULES
{
    PLAYM4_VIE_MODU_ADJ      = 0x00000001, //ͼ���������
    PLAYM4_VIE_MODU_EHAN     = 0x00000002, //�ֲ���ǿģ��
    PLAYM4_VIE_MODU_DEHAZE   = 0x00000004, //ȥ��ģ��
    PLAYM4_VIE_MODU_DENOISE  = 0x00000008, //ȥ��ģ��
    PLAYM4_VIE_MODU_SHARPEN  = 0x00000010, //��ģ��
    PLAYM4_VIE_MODU_DEBLOCK  = 0x00000020, //ȥ���˲�ģ��
    PLAYM4_VIE_MODU_CRB      = 0x00000040, //ɫ��ƽ��ģ��
    PLAYM4_VIE_MODU_LENS     = 0x00000080, //��ͷ�������ģ��
}PLAYM4_VIE_MODULES;
#endif

//���ùر�/����ģ��
//dwModuFlag��ӦPLAYM4_VIE_MODULES��,�����
//������ģ�鿪����������ģ��������ڼ����Ĭ�ϵĲ���;
//�ر�ģ����ϴ����õĲ������
//�����ӿڵ��ã������ڸýӿڿ���ģ��󣻷��򣬷��ش���
PLAYM4_API BOOL __stdcall PlayM4_VIE_SetModuConfig(LONG lPort,int nModuFlag,BOOL bEnable);

//����ͼ����ǿ����NULLȫͼ������ȫͼ������ȫͼ����С����16*16����
//��֧�������������Ƚ�˵4������һ���汾����ֻ֧��һ�����������Ҫ�����ص������ص��ͱ���
PLAYM4_API BOOL __stdcall PlayM4_VIE_SetRegion(LONG lPort,LONG lRegNum,HKRECT* pRect);

//��ȡ����ģ��
PLAYM4_API BOOL __stdcall PlayM4_VIE_GetModuConfig(LONG lPort,int* pdwModuFlag);

//���ò���
//δ����ģ��Ĳ������ñ�����
PLAYM4_API BOOL __stdcall PlayM4_VIE_SetParaConfig(LONG lPort,PLAYM4_VIE_PARACONFIG* pParaConfig);

//��ȡ����ģ��Ĳ���
PLAYM4_API BOOL __stdcall PlayM4_VIE_GetParaConfig(LONG lPort,PLAYM4_VIE_PARACONFIG* pParaConfig);

//����Ƶͬ���ӿ�
PLAYM4_API BOOL __stdcall PlayM4_SyncToAudio(LONG nPort, BOOL bSyncToAudio);

// ˽����Ϣģ������
typedef enum _PLAYM4_PRIDATA_RENDER
{    
    PLAYM4_RENDER_ANA_INTEL_DATA   = 0x00000001, //���ܷ���
    PLAYM4_RENDER_MD               = 0x00000002, //�ƶ����
    PLAYM4_RENDER_ADD_POS          = 0x00000004, //POS��Ϣ�����        
    PLAYM4_RENDER_ADD_PIC          = 0x00000008, //ͼƬ����
    PLAYM4_RENDER_FIRE_DETCET      = 0x00000010,  //�ȳ�����Ϣ
    PLAYM4_RENDER_TEM              = 0x00000020,   //�¶���Ϣ
    PLAYM4_RENDER_TRACK_TEM        = 0x00000040,    //�켣��Ϣ
    PLAYM4_RENDER_THERMAL          = 0x00000080 //���������̻�������Ϣ
}PLAYM4_PRIDATA_RENDER;

typedef enum _PLAYM4_THERMAL_FLAG
{
    PLAYM4_THERMAL_FIREMASK = 0x00000001, //�̻�����
    PLAYM4_THERMAL_RULEGAS = 0x00000002, //����������
    PLAYM4_THERMAL_TARGETGAS = 0x00000004 //Ŀ��������
}PLAYM4_THERMAL_FLAG;

typedef enum _PLAYM4_FIRE_ALARM{
    PLAYM4_FIRE_FRAME_DIS           = 0x00000001, //������ʾ
    PLAYM4_FIRE_MAX_TEMP            = 0x00000002, //����¶�
    PLAYM4_FIRE_MAX_TEMP_POSITION   = 0x00000004, //����¶�λ����ʾ
    PLAYM4_FIRE_DISTANCE            = 0x00000008, //����¶Ⱦ���}PLAYM4_FIRE_ALARM
}PLAYM4_FIRE_ALARM;

typedef enum _PLAYM4_TEM_FLAG{
    PLAYM4_TEM_REGION_BOX           = 0x00000001, //�����
    PLAYM4_TEM_REGION_LINE          = 0x00000002, //�߲���
    PLAYM4_TEM_REGION_POINT         = 0x00000004, //�����}PLAYM4_TEM_FLAG
}PLAYM4_TEM_FLAG;

typedef enum _PLAYM4_TRACK_FLAG
{
    PLAYM4_TRACK_PEOPLE = 0x00000001, //�˹켣
    PLAYM4_TRACK_VEHICLE = 0x00000002, //���켣
}PLAYM4_TRACK_FLAG;

typedef struct TI_PTZ_INFO_
{
    unsigned short dwDefVer;    //�ṹ��汾
    unsigned short dwLength;    //PTZ_info���ȣ���8�ֽ�Ϊ��λ
    DWORD          dwP;    //P��0~3600��
    DWORD          dwT;         //T��0~3600��
    DWORD          dwZ;         //Z��0~3600��
    BYTE        chFSMState; //����״̬
    BYTE           bClearFocusState; //�۽�����״̬��0,1��
    BYTE        reserved[6]; //6���ֽڱ���
}PTZ_INFO;


// ������Ϣ����
PLAYM4_API BOOL __stdcall PlayM4_RenderPrivateData(LONG nPort, int nIntelType, BOOL bTrue);

PLAYM4_API BOOL __stdcall PlayM4_RenderPrivateDataEx(LONG nPort, int nIntelType, int nSubType, BOOL bTrue);

// ���������ص�,nType=0��ʾ�������ܱ��λ�����仯�ͻص���nType=1��ʾ�����м���λ�����ص�
PLAYM4_API BOOL __stdcall PlayM4_SetEncryptTypeCallBack(LONG nPort, DWORD nType,
                                                        void (CALLBACK* EncryptTypeCBFun)(long nPort, ENCRYPT_INFO* pEncryptInfo, long nUser, long nReserved2), long nUser);
//lType: 1 ��ʾ��ȡ��ǰ��ʾ֡PTZ��Ϣ�����ض��ṹ����ʽ�洢��pInfo�ڣ�plLen���س�����Ϣ
PLAYM4_API BOOL __stdcall PlayM4_GetStreamAdditionalInfo(LONG nPort, LONG lType, BYTE* pInfo, LONG* plLen);


#endif //_PLAYM4_H_
