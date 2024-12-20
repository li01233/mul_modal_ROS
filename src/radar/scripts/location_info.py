from dataclasses import dataclass, field
import struct
import socket

@dataclass(frozen=True, order=True)
class LocationInfo:
    field1: int
    field2: int
    model_id: int
    field3: int
    field4: int
    field5: int
    field6: int
    data_ip: str
    data_port: int
    radar_ip: str
    radar_port: int
    
    @staticmethod
    def parse(data: bytes):
        fields = list(struct.unpack('<IIBBHIII4sI4s', data))
        
        fields[7] = socket.inet_ntoa(struct.pack('>I', fields[7]))
        fields[8] = struct.unpack('<2H', fields[8])[0]
        fields[9] = socket.inet_ntoa(struct.pack('>I', fields[9]))
        fields[10] = struct.unpack('<2H', fields[10])[0]
        
        return fields

def ntohs(x):
    # 将输入转换为整数
    x = int(x)
    # 确保结果是16位的无符号整数
    return x & 0xFFFF

def ntohl(x):
    # 将输入转换为整数
    x = int(x)
    # 确保结果是32位的无符号整数
    return x & 0xFFFFFFFF


class LocationInfoBlock:
    field1: int
    field2: int
    model_id: int
    field3: int
    field4: int
    field5: int
    field6: int
    data_ip: str
    data_port: int
    radar_ip: str
    radar_port: int

    def __init__(self, report):
        self.field1, self.field2, self.model_id, self.field3, 
        self.field4, self.field5, self.field6, self.data_ip, 
        self.data_port, self.radar_ip, self.radar_port = struct.unpack('<IIBBI IIIIIII', report)

class RadarLocationInfo:
    def __init__(self, report):
        self.report_addr = socket.inet_ntoa(struct.pack('<I', report.data_ip))
        self.report_addr_port = report.data_port
        self.send_command_addr = socket.inet_ntoa(struct.pack('<I', report.radar_ip))
        self.send_command_addr_port = report.radar_port
        self.spoke_data_addr = socket.inet_ntoa(struct.pack('<I', report.data_ip))
        self.spoke_data_addr_port = report.data_port
        self.serialNr = " "



class RaymarineLocate:
    def ProcessReport(self, report):
        rRec = LocationInfoBlock(report)
        raymarine_radar_code = 0x28

        if rRec.model_id == raymarine_radar_code:
            infoA = RadarLocationInfo(rRec)
            if rRec.data_ip == 0:
                if raymarine_radar_code != 0x28:
                    return False
                else:
                    radar_ipA.port = ntohs(rRec.radar_port)
                    infoA.report_addr.addr = ntohl(rRec.radar_ip)
                    infoA.report_addr.port = ntohs(rRec.radar_port)
            else:
                radar_ipA.port = ntohs(1)
                infoA.report_addr.addr = ntohl(rRec.data_ip)
                infoA.report_addr.port = ntohs(rRec.data_port)
            infoA.send_command_addr.addr = ntohl(rRec.radar_ip)
            infoA.send_command_addr.port = ntohs(rRec.radar_port)
            infoA.spoke_data_addr.addr = ntohl(rRec.data_ip)
            infoA.spoke_data_addr.port = ntohs(rRec.data_port)
            infoA.serialNr = " "
            return True
        return False
    

    

