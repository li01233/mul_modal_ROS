#include "hik_camera.h"

void HikCamera::P2PDataCollet(const ros::TimerEvent& e)
{
    //抓热图
    if (!NET_DVR_CaptureJPEGPicture_WithAppendData(user_id, 2, &struJpegWithAppendAata))
    {
        ROS_ERROR("NET_DVR_CaptureJPEGPicture_WithAppendData failed, error code: %d\n", NET_DVR_GetLastError());
    }
    else
    {
        if (struJpegWithAppendAata.dwP2PDataLen > 0 && struJpegWithAppendAata.pP2PDataBuff != NULL)
        {
            hik_tem::P2PData p2pData;
            p2pData.header.stamp = ros::Time::now();
            p2pData.header.frame_id = "hik_tem_cam";
            p2pData.data.resize(struJpegWithAppendAata.dwP2PDataLen);
            memcpy(&p2pData.data[0], struJpegWithAppendAata.pP2PDataBuff, struJpegWithAppendAata.dwP2PDataLen);
            p2p_data_pub.publish(p2pData);
        }
    }
}

bool HikCamera::initThemCam()
{
    //全屏测温参数配置
    //输入参数
    NET_DVR_XML_CONFIG_INPUT struInput = { 0 };
    struInput.dwSize = sizeof(struInput);

    //输出参数
    NET_DVR_XML_CONFIG_OUTPUT struOutputParam = { 0 };
    struOutputParam.dwSize = sizeof(struOutputParam);

    //获取参数
    char szUrl[512];
    memset(szUrl, 0, sizeof(szUrl));
    sprintf(szUrl, "%s", "GET /ISAPI/Thermal/channels/2/thermometry/pixelToPixelParam\r\n"); 

    //不同功能对应不同URL，具体参考相应的协议文档
    struInput.lpRequestUrl = szUrl;
    struInput.dwRequestUrlLen = strlen(szUrl);

    //获取时输入为空
    struInput.lpInBuffer = NULL;
    struInput.dwInBufferSize = 0;

    //分配输出内存
    char szGetOutput[8 * 1024] = { 0 };
    struOutputParam.lpOutBuffer = szGetOutput;
    struOutputParam.dwOutBufferSize = sizeof(szGetOutput);

    //输出状态
    char szStatusBuf[1024] = { 0 };
    struOutputParam.lpStatusBuffer = szStatusBuf;
    struOutputParam.dwStatusSize = sizeof(szStatusBuf);

    if (!NET_DVR_STDXMLConfig(user_id, &struInput, &struOutputParam))
    {
        ROS_INFO("NET_DVR_STDXMLConfig failed, error code: %d\n", NET_DVR_GetLastError());
        return false;
    }
    else
    {
        ROS_INFO("NET_DVR_STDXMLConfig successfully!\n");
        ROS_INFO("%s\n", szGetOutput);
    }

    //设置参数
    memset(szUrl, 0, sizeof(szUrl));
    sprintf(szUrl, "%s", "PUT /ISAPI/Thermal/channels/2/thermometry/pixelToPixelParam\r\n");

    //输入JSON数据
    char pBuf[2 * 1024] = { 0 };
    strcpy(pBuf, "<?xml version=\"1.0\" encoding=\"UTF - 8\"?>"
        "<PixelToPixelParam version = \"2.0\" xmlns = \"http://www.hikvision.com/ver20/XMLSchema\">"
        "<id>2</id>"
        "<maxFrameRate>400</maxFrameRate>"
        "<reflectiveEnable>false</reflectiveEnable>"
        "<reflectiveTemperature>20.00</reflectiveTemperature>"
        "<emissivity>0.98</emissivity>"
        "<distance>3000</distance>"
        "<refreshInterval>50</refreshInterval>"
        "<distanceUnit>centimeter</distanceUnit>"
        "<temperatureDataLength>4</temperatureDataLength>"
        "<JpegPictureWithAppendData>"
        "<jpegPicEnabled>true</jpegPicEnabled>"
        "<visiblePicEnabled>true</visiblePicEnabled>"
        "</JpegPictureWithAppendData>"
        "</PixelToPixelParam>");//中文字符需要使用UTF-8字符集

    //输入参数
    struInput.lpInBuffer = pBuf;
    struInput.dwInBufferSize = sizeof(pBuf);

    //输出结果
    char szOutput[8 * 1024] = { 0 };
    struOutputParam.lpOutBuffer = szOutput;
    struOutputParam.dwOutBufferSize = sizeof(szOutput);

    //输出状态
    char szStatusBuff[1024] = { 0 };
    struOutputParam.lpStatusBuffer = szStatusBuff;
    struOutputParam.dwStatusSize = sizeof(szStatusBuff);

    if (!NET_DVR_STDXMLConfig(user_id, &struInput, &struOutputParam))
    {
        ROS_INFO("NET_DVR_STDXMLConfig failed, error code: %d\n", NET_DVR_GetLastError());
        return false;
    }
    else
    {
        ROS_INFO("NET_DVR_STDXMLConfig successfully!\n");
        ROS_INFO("lpOutBuffer: %s\n", szOutput);
        ROS_INFO("lpStatusBuffer: %s\n", szStatusBuff);
    }
    return true;
}

void HikCamera::PtzCtrlCallback(const hik_tem::PtzCtrl::ConstPtr& msg)
{
    NET_DVR_PTZPOS ptz_pos;
    unsigned int errCode;

    ptz_pos.wAction = 1;
    ptz_pos.wPanPos = dec_to_hex((*msg).Pan * 10);
    ptz_pos.wTiltPos = dec_to_hex((*msg).Tilt * 10);
    
    // 不变焦
    NET_DVR_PTZPOS Pos_judge;
    DWORD tmp = 0;
    NET_DVR_GetDVRConfig(0, NET_DVR_GET_PTZPOS, 0, &Pos_judge, sizeof(NET_DVR_PTZPOS), &tmp);

    ptz_pos.wZoomPos = Pos_judge.wZoomPos;
    if(!NET_DVR_SetDVRConfig(user_id,NET_DVR_SET_PTZPOS,channel,(void*)&ptz_pos,sizeof(NET_DVR_PTZPOS))){
        errCode = NET_DVR_GetLastError();
        ROS_ERROR("******************************************\n");
        ROS_ERROR("errCode: %d\n",errCode);
        ROS_ERROR("******************************************\n");
    }
    
}


bool HikCamera::initHikSDK()
{
    NET_DVR_Init();
    NET_DVR_USER_LOGIN_INFO struLoginInfo = {0};
    NET_DVR_DEVICEINFO_V40  struDeviceInfoV40 = {0};
    struLoginInfo.bUseAsynLogin = false;

    struLoginInfo.wPort = port;
    memcpy(struLoginInfo.sDeviceAddress,ip_addr.c_str(), NET_DVR_DEV_ADDRESS_MAX_LEN);
    memcpy(struLoginInfo.sUserName, usr_name.c_str(), NAME_LEN);
    memcpy(struLoginInfo.sPassword, password.c_str(), NAME_LEN);
    user_id = NET_DVR_Login_V40(&struLoginInfo, &struDeviceInfoV40);

    if (user_id < 0)
    {
        ROS_INFO("[%s] Login fail, get: %u",camera_name.c_str(), NET_DVR_GetLastError());
        NET_DVR_Cleanup();
        return false;
    }
    
    //自己分配内存，需要大于实际数据长度
    if (struJpegWithAppendAata.pJpegPicBuff == NULL)
    {
        struJpegWithAppendAata.pJpegPicBuff = new char[2 * 1024 * 1024];
        memset(struJpegWithAppendAata.pJpegPicBuff, 0, 2 * 1024 * 1024);
    }
    if (struJpegWithAppendAata.pP2PDataBuff == NULL)
    {
        struJpegWithAppendAata.pP2PDataBuff = new char[2 * 1024 * 1024];
        memset(struJpegWithAppendAata.pP2PDataBuff, 0, 2 * 1024 * 1024);
    }
    if (struJpegWithAppendAata.pVisiblePicBuff == NULL)//可见光图至少为4M
    {
        struJpegWithAppendAata.pVisiblePicBuff = new char[10 * 1024 * 1024];
        memset(struJpegWithAppendAata.pVisiblePicBuff, 0, 10 * 1024 * 1024);
    }

    if(!initThemCam()){return false;}

    return true;
}


void HikCamera::initROSIO(ros::NodeHandle& priv_node)
{
    /// camera parameter
    priv_node.param("camera_frame_id", frame_id, std::string("hik_camera"));
    priv_node.param("camera_name", camera_name,  std::string("hik_camera"));
    priv_node.param("camera_info_url", camera_info_url, std::string(""));

    priv_node.param<std::string>("ip_addr", ip_addr,"192.168.5.100");
    ROS_INFO("[%s] ip address:\t%s", camera_name.c_str(), ip_addr.c_str());

    priv_node.param<std::string>("usr_name",usr_name,"admin");
    ROS_INFO("[%s] user name: \t%s", camera_name.c_str(), usr_name.c_str());

    priv_node.param<std::string>("password",password,"ht123456");
    ROS_INFO("[%s] password:  \t%s", camera_name.c_str(), password.c_str());

    priv_node.param<int>("port",port, 8000);
    ROS_INFO("[%s] port:      \t%d", camera_name.c_str(), port);

    priv_node.param<int>("channel",channel,1);
    ROS_INFO("[%s] channel:   \t%d", camera_name.c_str(), channel);

    priv_node.param<int>("link_mode",link_mode, 0);
    if(link_mode < 0 || link_mode >5)
    {
        ROS_WARN("[%s] value %d for link_mode is illegal, set to default value 0 (tcp)",camera_name.c_str(), link_mode);
    }

    std::string _mode []  = {"tcp", "udp", "multicast","rtp","rtp/rtsp", "rstp/http"};
    ROS_INFO("[%s] link mode: \t%s", camera_name.c_str(), _mode[link_mode].c_str());

    priv_node.param<int>("image_width",image_width,1280);
    ROS_INFO("[%s] image width:  \t%d", camera_name.c_str(), image_width);

    priv_node.param<int>("image_height",image_height, 720);
    ROS_INFO("[%s] image height: \t%d", camera_name.c_str(), image_height);

    // 云台控制
    ptz_ctrl = priv_node.subscribe<hik_tem::PtzCtrl>("/ptz_ctrl", 10, boost::bind(&HikCamera::PtzCtrlCallback, this, _1));
    // 全图温度采集
    timer = priv_node.createTimer(ros::Duration(0.1), boost::bind(&HikCamera::P2PDataCollet, this, _1));
    p2p_data_pub = priv_node.advertise<hik_tem::P2PData>("/p2p_data", 10);
}

void HikCamera::run()
{
    ros::NodeHandle priv_node("~");

    initROSIO(priv_node);

    if(initHikSDK())
    {
        ros::spin();
    }
}

HikCamera::~HikCamera()
{

    if(user_id)
    {
        NET_DVR_Logout_V30(user_id);
    }

    NET_DVR_Cleanup();

    ROS_INFO("[%s] END",camera_name.c_str());
}
