import serial
import struct
import time

# Constants for MSP message IDs
MSP_API_VERSION = 1
MSP_FC_VARIANT = 2
MSP_FC_VERSION = 3
MSP_BOARD_INFO = 4
MSP_BUILD_INFO = 5
# ...existing constants...
MSP_STATUS = 101
MSP_RAW_IMU = 102
# ...existing constants...

# MSP mode bits
MSP_MODE_ARM = 0
MSP_MODE_ANGLE = 1
# ...existing mode bits...

# Data structures for MSP messages
class MSPApiVersion:
    def __init__(self, protocol_version=0, api_major=0, api_minor=0):
        self.protocol_version = protocol_version
        self.api_major = api_major
        self.api_minor = api_minor

class MSPStatus:
    def __init__(self, cycle_time=0, i2c_error_counter=0, sensor=0, flight_mode_flags=0, config_profile_index=0):
        self.cycle_time = cycle_time
        self.i2c_error_counter = i2c_error_counter
        self.sensor = sensor
        self.flight_mode_flags = flight_mode_flags
        self.config_profile_index = config_profile_index

# ...other data structures...

class MSP:
    def __init__(self, ser, timeout=0.5):
        self.ser = ser
        self.timeout = timeout

    def reset(self):
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def send(self, message_id, payload=b''):
        size = len(payload)
        header = b'$M<'
        checksum = size ^ message_id
        for byte in payload:
            checksum ^= byte
        frame = header + bytes([size]) + bytes([message_id]) + payload + bytes([checksum])
        self.ser.write(frame)

    def recv(self, message_id=None, max_size=256):
        start_time = time.time()
        while True:
            if self.ser.in_waiting >= 3:
                header = self.ser.read(3)
                if header == b'$M>' or header == b'$M<':
                    size = self.ser.read(1)[0]
                    recv_id = self.ser.read(1)[0]
                    payload = self.ser.read(size)
                    checksum = self.ser.read(1)[0]
                    checksum_calc = size ^ recv_id
                    for byte in payload:
                        checksum_calc ^= byte
                    if checksum == checksum_calc:
                        if message_id is None or recv_id == message_id:
                            return recv_id, payload
                else:
                    continue
            if time.time() - start_time >= self.timeout:
                return None, None

    def activity_detected(self):
        start_time = time.time()
        while True:
            if self.ser.in_waiting > 0:
                return True
            if time.time() - start_time >= self.timeout:
                return False

    def wait_for(self, message_id):
        start_time = time.time()
        while time.time() - start_time < self.timeout:
            recv_id, payload = self.recv(message_id)
            if recv_id == message_id:
                return payload
        return None

    def request(self, message_id):
        self.send(message_id)
        return self.wait_for(message_id)

    def command(self, message_id, payload=b'', wait_ack=True):
        self.send(message_id, payload)
        if wait_ack:
            ack = self.wait_for(message_id)
            return ack is not None
        return True

    def get_active_modes(self):
        status_payload = self.request(MSP_STATUS)
        if status_payload:
            status_data = struct.unpack('<HHHIB', status_payload)
            status = MSPStatus(*status_data)
            boxids_payload = self.request(MSP_BOXIDS)
            if boxids_payload:
                boxids = list(boxids_payload)
                active_modes = 0
                for i, boxid in enumerate(boxids):
                    if status.flight_mode_flags & (1 << i):
                        if boxid in self.BOXIDS:
                            mode_bit = self.BOXIDS.index(boxid)
                            active_modes |= 1 << mode_bit
                return active_modes
        return None

    # Define BOXIDS as per C++ implementation
    BOXIDS = [
        0,  # 0: MSP_MODE_ARM
        1,  # 1: MSP_MODE_ANGLE
        2,  # 2: MSP_MODE_HORIZON
        3,  # 3: MSP_MODE_NAVALTHOLD
        5,  # 4: MSP_MODE_MAG
        6,  # 5: MSP_MODE_HEADFREE
        7,  # 6: MSP_MODE_HEADADJ
        # ...other mode mappings...
    ]