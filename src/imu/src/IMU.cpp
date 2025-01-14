#include "IMU.h"


Imu::Imu() {}

bool Imu::initROSIO(ros::NodeHandle& priv_node)
{
    /// imu parameter
    priv_node.param<std::string>("ip_addr", ip_addr,"192.168.5.100");
    ROS_INFO("ip address:\t%s", ip_addr.c_str());

    priv_node.param<int>("port",port, 8000);
    ROS_INFO("port:      \t%d", port);
    
    // turn it to network byte order
    in_addr_t ip_addr_n = inet_addr(ip_addr.c_str());
    if (ip_addr_n == INADDR_NONE) {
        std::cerr << "Invalid IP address: " << ip_addr_n << std::endl;
        return false;
    }
    in_port_t port_n = htons(port);
    
    // bind socket and init publisher
    if(initIMU(ip_addr_n, port_n))
    {
        ROS_INFO("initIMU success");
        pub_ship = priv_node.advertise<message_interface::Ownship>("/ownship",10);
        pub_env = priv_node.advertise<message_interface::EnvData>("/envdata",10);
        timer = priv_node.createTimer(ros::Duration(0.01), &Imu::readUdpPendingDatagrams, this, false, true);
        return true;
    }
    else
    {
        ROS_INFO("initIMU fail");
        return false;
    }
}

bool Imu::initIMU(in_addr_t ip_addr_n, in_port_t port_n)
{
    // Create UDP socket
    if (_socket == -1) {
        _socket = socket(AF_INET, SOCK_DGRAM, 0);
        if (_socket == -1) {
            ROS_INFO("Failed to create socket");
            return false;
        }
    }

    int reuse = 1;
    if (setsockopt(_socket, SOL_SOCKET, SO_REUSEADDR,
                   (char *)&reuse, sizeof(reuse)) < 0) {
      ROS_INFO("setting SO_REUSEADDR");
      close(_socket);
      return false;
    }

    // Bind to the server address
    // on this port, receives ALL multicast groups
    memset(&addr, 0, sizeof(addr));

    addr.sin_family = AF_INET;       // Use IPV4
    addr.sin_port   = port_n;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if(bind(_socket, (struct sockaddr*)&addr, sizeof(addr)) == -1){
        ROS_INFO("Failed to bind socket on port");
        close(_socket);
        return false;
    }
    
    // Tell the operating system to add the socket to the multicast group
    // on all interfaces. 
    struct ip_mreq mreq;
    mreq.imr_multiaddr.s_addr = ip_addr_n;
    mreq.imr_interface.s_addr = htonl(INADDR_ANY);
    if(setsockopt(_socket, IPPROTO_IP, IP_ADD_MEMBERSHIP, 
                    (char *)&mreq, sizeof(mreq)) < 0){
        ROS_INFO("setsockopt failed !");
        close(_socket);
        return false;
    }else
    {
        ROS_INFO("setsockopt success!");
    }


    // loop until IMU is found
    char buf[BUFF_LEN];
    memset(buf, 0, BUFF_LEN);

    socklen_t len = sizeof(addr);
    while(true)
    {
        int ret = recvfrom(_socket, buf, BUFF_LEN, 0, 
                            (struct sockaddr*)&addr, &len);
        if(ret > 300){
            ROS_INFO("IMU found");
            break;
        }
    }

    // Set the socket to non-blocking mode
    int flags = fcntl(_socket, F_GETFL, 0);
    fcntl(_socket, F_SETFL, flags | O_NONBLOCK);
    
    return true;   
}


void Imu::readUdpPendingDatagrams(const ros::TimerEvent& event)
{
    
    memset(buffer, 0, BUFF_LEN);

    socklen_t len = sizeof(addr);
    int ret = recvfrom(_socket, buffer, BUFF_LEN, 0, 
                            (struct sockaddr*)&addr, &len);

    if (ret > 0){
        parse(buffer);
    }

}

void Imu::parse(char *buffer)
{
    // Parse data to JSON and Publish data
    std::string str_b = buffer;
    rapidjson::Document doc;
    doc.Parse(str_b.c_str());

    if (doc.HasParseError()){
        ROS_INFO("GetJsonData ParseError");
        return;
    }
    std::string dataTypeStr = doc["DataType"].GetString();
    if (dataTypeStr == "OwnShip"){
        parseOwnShip(doc);
    }
    else if (dataTypeStr == "EnvirData"){
        parseEnvData(doc);
    }
    ROS_INFO("Stream CallBack, IMU DATA IS RECEIVED.");
}

void Imu::parseOwnShip(rapidjson::Document& doc)
{
    message_interface::Ownship ownship;
    // Parse data to JSON
    ownship.DataType = doc["DataType"].GetString();
    ownship.DateTime = doc["DateTime"].GetString();
    ownship.MMSI = doc["MMSI"].GetInt();

    const rapidjson::Value& content = doc["content"];

    ownship.cog = content["cog"].GetFloat();
    ownship.draft = content["draft"].GetFloat();
    ownship.groundMileAll = content["groundMileAll"].GetFloat();
    ownship.groundMileClean = content["groundMileClean"].GetFloat();
    ownship.head = content["head"].GetFloat();
    ownship.headRatio = content["headRatio"].GetFloat();
    ownship.lat = std::stod(content["lat"].GetString());
    ownship.lon = std::stod(content["lon"].GetString());
    ownship.pitch = content["pitch"].GetFloat();
    ownship.pitchRatio = content["pitchRatio"].GetFloat();
    ownship.roll = content["roll"].GetFloat();
    ownship.rollRatio = content["rollRatio"].GetFloat();
    ownship.seaDepth = content["seaDepth"].GetFloat();
    ownship.sog = content["sog"].GetFloat();
    ownship.sogX = content["sogX"].GetFloat();
    ownship.sogY = content["sogY"].GetFloat();
    ownship.stw = content["stw"].GetFloat();
    ownship.stwX = content["stwX"].GetFloat();
    ownship.stwY = content["stwY"].GetFloat();
    ownship.waterMileAll = content["waterMileAll"].GetFloat();
    ownship.waterMileClean = content["waterMileClean"].GetFloat();

    pub_ship.publish(ownship);

}

void Imu::parseEnvData(rapidjson::Document& doc)
{
    message_interface::EnvData envdata;

    envdata.DataType = doc["DataType"].GetString();
    envdata.DateTime = doc["DateTime"].GetString();
    envdata.MMSI = doc["MMSI"].GetInt();

    const rapidjson::Value& content = doc["content"];

    envdata.CurrentAng = content["CurrentAng"].GetFloat();
    envdata.CurrentSpd = content["CurrentSpd"].GetFloat();
    envdata.FogMod = content["FogMod"].GetInt();
    envdata.RainMod = content["RainMod"].GetInt();
    envdata.SnowMod = content["SnowMod"].GetInt();
    envdata.SurgeAng = content["SurgeAng"].GetFloat();
    envdata.SurgeHeight = content["SurgeHeight"].GetFloat();
    envdata.SurgePeirod = content["SurgePeirod"].GetFloat();
    envdata.WaterDeep = content["WaterDeep"].GetFloat();
    envdata.WaveHeightSea = content["WaveHeightSea"].GetFloat();
    envdata.WaveHeightWind = content["WaveHeightWind"].GetFloat();
    envdata.WavePeirod = content["WavePeirod"].GetFloat();
    envdata.WindAngA = content["WindAngA"].GetFloat();
    envdata.WindAngR = content["WindAngR"].GetFloat();
    envdata.WindSpdA = content["WindSpdA"].GetFloat();
    envdata.WindSpdR = content["WindSpdR"].GetFloat();

    pub_env.publish(envdata);

}


void Imu::run()
{
    ros::NodeHandle priv_node("~");

    if(initROSIO(priv_node))ros::spin();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu");

    Imu imu;
    imu.run();

    return 0;
}

