import serial
import struct
import time

# Constants for MSP message IDs
MSP_API_VERSION = 1
MSP_FC_VARIANT = 2
MSP_FC_VERSION = 3
MSP_BOARD_INFO = 4
MSP_BUILD_INFO = 5
MSP_CALIBRATION_DATA = 14
MSP_FEATURE = 36
MSP_BOARD_ALIGNMENT = 38
MSP_CURRENT_METER_CONFIG = 40
MSP_RX_CONFIG = 44
MSP_SONAR_ALTITUDE = 58
MSP_ARMING_CONFIG = 61
MSP_RX_MAP = 64
MSP_LOOP_TIME = 73
MSP_STATUS = 101
MSP_RAW_IMU = 102
MSP_SERVO = 103
MSP_MOTOR = 104
MSP_RC = 105
MSP_RAW_GPS = 106
MSP_COMP_GPS = 107
MSP_ATTITUDE = 108
MSP_ALTITUDE = 109
MSP_ANALOG = 110
MSP_RC_TUNING = 111
MSP_PID = 112
MSP_MISC = 114
MSP_SERVO_CONFIGURATIONS = 120
MSP_NAV_STATUS = 121
MSP_SENSOR_ALIGNMENT = 126
MSP_STATUS_EX = 150
MSP_SENSOR_STATUS = 151
MSP_BOXIDS = 119
MSP_UID = 160
MSP_GPSSVINFO = 164
MSP_GPSSTATISTICS = 166
MSP_SET_PID = 202

# MSP commands
MSP_SET_HEAD = 211
MSP_SET_RAW_RC = 200
MSP_SET_RAW_GPS = 201
MSP_SET_WP = 209

# MSP mode bits
MSP_MODE_ARM = 0
MSP_MODE_ANGLE = 1
MSP_MODE_HORIZON = 2
MSP_MODE_NAVALTHOLD = 3
MSP_MODE_MAG = 4
MSP_MODE_HEADFREE = 5
MSP_MODE_HEADADJ = 6
MSP_MODE_CAMSTAB = 7
MSP_MODE_NAVRTH = 8
MSP_MODE_NAVPOSHOLD = 9
MSP_MODE_PASSTHRU = 10
MSP_MODE_BEEPERON = 11
MSP_MODE_LEDLOW = 12
MSP_MODE_LLIGHTS = 13
MSP_MODE_OSD = 14
MSP_MODE_TELEMETRY = 15
MSP_MODE_GTUNE = 16
MSP_MODE_SONAR = 17
MSP_MODE_BLACKBOX = 18
MSP_MODE_FAILSAFE = 19
MSP_MODE_NAVWP = 20
MSP_MODE_AIRMODE = 21
MSP_MODE_HOMERESET = 22
MSP_MODE_GCSNAV = 23
MSP_MODE_HEADINGLOCK = 24
MSP_MODE_SURFACE = 25
MSP_MODE_FLAPERON = 26
MSP_MODE_TURNASSIST = 27
MSP_MODE_NAVLAUNCH = 28
MSP_MODE_AUTOTRIM = 29

# Data structures for MSP messages
from MSP_structs import (
    MSPApiVersion,
    MSPStatus,
    MSPFCVariant,
    MSPFCVersion,
    MSPBoardInfo,
    MSPBuildInfo,
    MSPRawIMU,
    MSPSensorStatus,
    MSPServo,
    MSPServoConfigurations,
    MSPMotor,
    MSPRC,
    MSPAttitude,
    MSPAltitude,
    MSPSonarAltitude,
    MSPAnalog,
    MSPArmingConfig,
    MSPLoopTime,
    MSPRCTuning,
    MSPPID,
    MSPMisc,
    MSPRawGPS,
    MSPCompGPS,
    MSPNavStatus,
    MSPGPSSVInfo,
    MSPGPSStatistics,
    MSPUID,
    MSPFeature,
    MSPBoardAlignment,
    MSPCurrentMeterConfig,
    MSPRXConfig,
    MSPRXMap,
    MSPSensorAlignment,
    MSPCalibrationData,
    MSPSetHead,
    MSPSetRawRC,
    MSPSetPID,
    MSPSetRawGPS,
    MSPSetWP
)

class MSP:
    def __init__(self, ser, timeout=0.5):
        self.ser = ser
        self.timeout = timeout

    def reset(self):
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def send(self, message_id, payload=b''):
        if hasattr(payload, 'serialize') and callable(payload.serialize):
            # Serialize the payload to bytes
            payload = payload.serialize()
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
        0,  #  0: MSP_MODE_ARM
        1,  #  1: MSP_MODE_ANGLE
        2,  #  2: MSP_MODE_HORIZON
        3,  #  3: MSP_MODE_NAVALTHOLD (cleanflight BARO)
        5,  #  4: MSP_MODE_MAG
        6,  #  5: MSP_MODE_HEADFREE
        7,  #  6: MSP_MODE_HEADADJ
        8,  #  7: MSP_MODE_CAMSTAB
        10, #  8: MSP_MODE_NAVRTH (cleanflight GPSHOME)
        11, #  9: MSP_MODE_NAVPOSHOLD (cleanflight GPSHOLD)
        12, # 10: MSP_MODE_PASSTHRU
        13, # 11: MSP_MODE_BEEPERON
        15, # 12: MSP_MODE_LEDLOW
        16, # 13: MSP_MODE_LLIGHTS
        19, # 14: MSP_MODE_OSD
        20, # 15: MSP_MODE_TELEMETRY
        21, # 16: MSP_MODE_GTUNE
        22, # 17: MSP_MODE_SONAR
        26, # 18: MSP_MODE_BLACKBOX
        27, # 19: MSP_MODE_FAILSAFE
        28, # 20: MSP_MODE_NAVWP (cleanflight AIRMODE)
        29, # 21: MSP_MODE_AIRMODE (cleanflight DISABLE3DSWITCH)
        30, # 22: MSP_MODE_HOMERESET (cleanflight FPVANGLEMIX)
        31, # 23: MSP_MODE_GCSNAV (cleanflight BLACKBOXERASE)
        32, # 24: MSP_MODE_HEADINGLOCK
        33, # 25: MSP_MODE_SURFACE
        34, # 26: MSP_MODE_FLAPERON
        35, # 27: MSP_MODE_TURNASSIST
        36, # 28: MSP_MODE_NAVLAUNCH
        37, # 29: MSP_MODE_AUTOTRIM
    ]