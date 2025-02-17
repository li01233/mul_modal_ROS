import socket
import struct
import numpy as np
import cv2
from cv_bridge import CvBridge

import rospy

from std_msgs.msg import String
from sensor_msgs.msg import Image
from message_interface.msg import Spoke

from location_info import LocationInfo
import control_message
from rmr_report import RMReport
from quantum_scan import QuantumScan
from quantum_report import QuantumReport
import filter as RadarFilter

MAX_SPOKE_LENGTH = 256
DEFAULT_NUM_SPOKES = 250
MAX_INTENSITY = 128
MIN_RANGE_INDEX = 0
MAX_RANGE_INDEX = 200

class FrameId:
    RM_REPORT = 0x00010001
    QUANTUM_REPORT = 0x00280002
    QUANTUM_SPOKE = 0x00280003

class Qauntum():

    def __init__(self):
        self.host = "127.0.0.1"
        
        # uncomment below to simulate locator
        # data = b'\x00\x00\x00\x00\x92\x8b\x80\xcb(\x00\x00\x00\x03\x00d\x00\x06\x08\x10\x00\x01\xb3\x01\xe8\x0e\n\x11\x002\x00\xe0\n\x0f\n6\x00'
        # bl = LocationInfo.parse(data[:36])
        # self.quantum_location = LocationInfo(*bl)
        self.quantum_location = self.locate_quantum() # this might take a while to return
        
        self.command_socket = self.create_command_socket() # socket for controlling radar
        self.report_socket = self.create_report_socket() # socket for receiving radar data

        self.standby_timer = rospy.Timer(rospy.Duration(1),self.standby_timer_callback)
        self.polar_image_timer  = rospy.Timer(rospy.Duration(5), self.imager_callback)
        self.data_timer = rospy.Timer(rospy.Duration(0.01), self.radar_data_callback)
        self.diagnostic_timer = rospy.Timer(rospy.Duration(3.5),self.publish_radar_heartbeat)

        self.image_publisher_ = rospy.Publisher("/radar_image", Image, queue_size=10)
        self.polar_image_publisher = rospy.Publisher('/USV/Polar', Image, queue_size=10)
        self.diagnostic_pub = rospy.Publisher('/diagnostic_status', String, queue_size=10)
        self.data_pub = rospy.Publisher('/quantum_spoke', Spoke, queue_size=10)

        # subscription
        self.radar_control_subscription = rospy.Subscriber('/radar_control', String, self.radar_control_callback, queue_size=10)
    
        self.alive_counter = 0
        self.num_spokes = DEFAULT_NUM_SPOKES # DEFAULT_NUM_SPOKES
        self.spokes = np.zeros((DEFAULT_NUM_SPOKES, MAX_SPOKE_LENGTH), np.uint8) # MAX_SPOKE_LENGTH = 1024
        self.set_zoom(0)
        self.bridge = CvBridge()
        self.scanning = False


    def locate_quantum(self, multicast_group='224.0.0.1', multicast_port=5800, quantum_model_id=40) -> LocationInfo:
        # Create UDP socket
        locator_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM, proto=socket.IPPROTO_UDP)
        locator_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        # Bind to the server address
        # on this port, receives ALL multicast groups
        locator_socket.bind(('', multicast_port))
        
        # Tell the operating system to add the socket to the multicast group
        # on all interfaces.
        mreq = struct.pack('4sL', socket.inet_aton(multicast_group), socket.INADDR_ANY)
        locator_socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        rospy.loginfo(f'Initialized locator socket')
        
        # loop until Raymarine Quantum is found
        rospy.loginfo(f'Locating radar...')
        while True:
            data, senderaddr = locator_socket.recvfrom(1024)
            rospy.loginfo(f'received {len(data)} bytes from {senderaddr[0]}:{senderaddr[1]}')
            if len(data) != 36: continue # ignore any packets not 36 bytes
            
            quantum_location = LocationInfo(*LocationInfo.parse(data))
            rospy.loginfo(f'{quantum_location=}')
            if quantum_location.model_id != quantum_model_id: continue
            
            rospy.loginfo(f'Found radar at {quantum_location.radar_ip}')
            break
        
        rospy.loginfo(f'Closing locator socket')
        locator_socket.close()
        return quantum_location


    def create_command_socket(self) -> socket:
        command_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM, proto=socket.IPPROTO_UDP)
        command_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        command_socket.setblocking(0) # non-blocking socket
        rospy.loginfo(f'Initialized command socket')
        
        return command_socket


    def standby_timer_callback(self, event):
        self.radar_stay_alive()
        self.alive_counter = (self.alive_counter + 1)%5
        

    def radar_stay_alive(self) -> None:
        self.transmit_command(control_message.STAY_ALIVE_1SEC)
        if (self.alive_counter%5 == 0):
            self.transmit_command(control_message.STAY_ALIVE_5SEC)
        rospy.loginfo(f'Sent stay alive command')


    def transmit_command(self, command) -> None:
        self.command_socket.sendto(command, (self.quantum_location.radar_ip, self.quantum_location.radar_port))
        rospy.loginfo(f'Sent {len(command)} bytes to {self.quantum_location.radar_ip}:{self.quantum_location.radar_port}')

    def start_scan(self) -> None:
        self.transmit_command(control_message.TX_ON)
        rospy.loginfo(f'Sent tx on command')
        self.scanning = True


    def stop_scan(self) -> None:
        self.transmit_command(control_message.TX_OFF)
        rospy.loginfo(f'Sent tx off command')
        self.scanning = False


    def set_zoom(self, i) -> None:
        self.range_index = i
        self.transmit_command(control_message.get_range_command(i))
        rospy.loginfo(f'Sent zoom level {i} command')


    def zoom_in(self) -> None:
        self.set_zoom(max(self.range_index - 1, MIN_RANGE_INDEX))


    def zoom_out(self) -> None:
        self.set_zoom(min(self.range_index + 1, MAX_RANGE_INDEX))


    def create_report_socket(self) -> socket:
        report_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM, proto=socket.IPPROTO_UDP)
        report_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # Bind to the server address
        # on this port, receives ALL multicast groups
        report_socket.bind(('', self.quantum_location.data_port))
        # Tell the operating system to add the socket to the multicast group
        # on all interfaces.
        # mreq = struct.pack('4sL', socket.inet_aton(self.quantum_location.data_ip), socket.INADDR_ANY) # no needed
        rospy.loginfo(f'receiver host: {self.host}')
        mreq = struct.pack('4s4s', socket.inet_aton(self.quantum_location.data_ip), socket.inet_aton(self.host)) # set to the ip of the receiver
        report_socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        rospy.loginfo(f'Initialized report socket')
        
        return report_socket


    def radar_data_callback(self, event):
        try:
            data, senderaddr = self.report_socket.recvfrom(1024)
        except OSError:
            return None
        
        self.process_frame(data)


    def process_frame(self, data: bytes):
        if len(data) < 4: return # data must be longer than 4 bytes
        
        self.last_frame_id = struct.unpack('<I', data[:4])[0] # read first 4 bytes
        rospy.loginfo(f'Received {len(data)} bytes (frame_id=0x{self.last_frame_id:X})')
        rospy.logdebug(f'{data}')
        
        if self.last_frame_id == FrameId.RM_REPORT:
            self.process_rm_report(data)
        elif self.last_frame_id == 0x00010002:
                # ProcessFixedReport(data, len)
            pass
        elif self.last_frame_id == 0x00010003:
            # ProcessScanData(data, len)
            pass
        elif self.last_frame_id == FrameId.QUANTUM_SPOKE:
            self.process_quantum_scan_data(data)
        elif self.last_frame_id == FrameId.QUANTUM_REPORT:
            self.process_quantum_report(data)
        elif self.last_frame_id == 0x00280001:  # type and serial for Quantum radar
            pass
            # IF_serial = wxString::FromAscii(data + 10, 7)
            # MOD_serial = wxString::FromAscii(data + 4, 6)
            # if (MOD_serial == _('E70498')) {
            # m_ri->m_quantum2type = true
            # }
            # m_ri->m_radar_location_info.serialNr = IF_serial
            # status = m_ri->m_state.GetValue()

            # match status:
            #     case RADAR_OFF:
            #         LOG_VERBOSE(wxT('%s reports status RADAR_OFF'), m_ri->m_name.c_str())
            #         stat = _('Off')
            #     case RADAR_STANDBY:
            #         LOG_VERBOSE(wxT('%s reports status STANDBY'), m_ri->m_name.c_str())
            #         stat = _('Standby')
            #     case RADAR_WARMING_UP:
            #         LOG_VERBOSE(wxT('%s reports status RADAR_WARMING_UP'), m_ri->m_name.c_str())
            #         stat = _('Warming up')
            #     case RADAR_TRANSMIT:
            #         LOG_VERBOSE(wxT('%s reports status RADAR_TRANSMIT'), m_ri->m_name.c_str())
            #         stat = _('Transmit')
            #     case _:
            #         # LOG_BINARY_RECEIVE(wxT('received unknown radar status'), report, len)
            #         stat = _('Unknown status')

            # s = wxString::Format(wxT('IP %s %s'), m_ri->m_radar_address.FormatNetworkAddress(), stat.c_str())
            # info = m_ri->GetRadarLocationInfo()
            # s << wxT('\n') << _('SKU ') << MOD_serial << _(' Serial #') << info.serialNr
            # SetInfoStatus(s)
        elif self.last_frame_id == 0x00010006:
            pass
            # IF_serial = wxString::FromAscii(data + 4, 7)
            # MOD_serial = wxString::FromAscii(data + 20, 7)
            # m_info = m_ri->GetRadarLocationInfo()
            # m_ri->m_radar_location_info.serialNr = IF_serial
        elif self.last_frame_id == 0x00018801:  # HD radar
            pass
            # ProcessRMReport(data, len)
        elif self.last_frame_id == 0x00010007:
            pass
        elif self.last_frame_id == 0x00010008:
            pass
        elif self.last_frame_id == 0x00010009:
            pass
        elif self.last_frame_id == 0x00018942:
            # rospy.INFO('other frame')
            pass
        else:
            # rospy.INFO('default frame')
            pass


    def process_rm_report(self, data: bytes):
        if len(data) < 186: return # ensure packet is longer than 186 bytes
        rospy.loginfo(f'{rmr}')
        
        bl = RMReport.parse_report(data[:260])
        rmr = RMReport(*bl)


    def process_quantum_scan_data(self, data: bytes):
        if len(data) < 20: return # ensure packet is longer than 20 bytes
        
        qheader = QuantumScan.parse_header(data[:20])
        qdata = QuantumScan.parse_data(data[20:])
        qs = QuantumScan(*qheader, qdata)
        rospy.logdebug(f'{qs}')
        
        if self.spokes is None: return

        self.num_spokes = qs.num_spokes
        # Quantum Q24C spokes are 180 degrees out of phase
        # i.e. 0th azimuth points towards the **back** of the radar
        # and 125th azimuth points towards the **front** of the radar
        self.spokes[(qs.azimuth + DEFAULT_NUM_SPOKES//2) %  DEFAULT_NUM_SPOKES, :len(qs.data)] = qs.data
        
        #########record############################################
        msg = Spoke()
        msg.azimuth = qs.azimuth
        msg.data = qs.data
        self.data_pub.publish(msg)


    def process_quantum_report(self, data: bytes):
        if len(data) < 260: return # ensure packet is longer than 260 bytes
        
        bl = QuantumReport.parse_report(data[:260])
        qr = QuantumReport(*bl)
        self.range_index = qr.range_index
        
        self.last_quantum_report = qr


    def imager_callback(self, event):
        polar_image = np.copy(self.spokes/MAX_INTENSITY * 255).astype(np.uint8)
        cv2.imwrite('test/polar_image.jpg', polar_image)
        
        self.polar_image_publisher.publish(self.bridge.cv2_to_imgmsg(polar_image, encoding="passthrough"))
        
        I, D, P = RadarFilter.generate_map(r=polar_image, k=None, p=None, K=None, area_threshold=50, gamma=None)
        
        ctrl_station_img = I
        self.image_publisher_.publish(self.bridge.cv2_to_imgmsg(ctrl_station_img, encoding="passthrough"))
        
        self.spokes = np.zeros((DEFAULT_NUM_SPOKES, MAX_SPOKE_LENGTH), np.uint8)


    def radar_control_callback(self, msg):
        if msg.data == 'start_scan':
            self.start_scan()
        elif msg.data == 'stop_scan':
            self.stop_scan()
        elif msg.data == 'toggle_scan':
            self.scanning = not self.scanning
            if self.scanning: self.start_scan()
            else: self.stop_scan()
        elif msg.data == 'zoom_in':
            self.zoom_in()
        elif msg.data == 'zoom_out':
            self.zoom_out()
        else:
            rospy.loginfo(f'unknown radar control: {msg.data}')


    def publish_radar_heartbeat(self, event):
        if self.scanning:
            self.diagnostic_pub.publish(String(data="Radar: Scanning"))
        else:
            self.diagnostic_pub.publish(String(data="Radar: Standby"))


def main(args=None):
    rospy.init_node("quantum_radar")
    quantum = Qauntum()
    rospy.spin()


if __name__ == '__main__':
    main()
