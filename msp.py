import serial
import time
import logging
import struct

# Configure logging
logging.basicConfig(filename='msp_service.log', level=logging.DEBUG,
                    format='%(asctime)s - %(levelname)s - %(message)s')


class MSP:
    MSP_HEADER = "$M<"
    MSP_IDENT = 100
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
    MSP_BOX = 113
    MSP_MISC = 114
    MSP_MOTOR_PINS = 115
    MSP_BOXNAMES = 116
    MSP_PIDNAMES = 117
    MSP_WP = 118
    MSP_BOXIDS = 119
    MSP_SERVO_CONF = 120
    MSP_NAV_STATUS = 121
    MSP_NAV_CONFIG = 122
    MSP_MOTOR_MIXER = 128
    MSP_RESET_CONF = 131
    MSP_SET_RAW_RC = 200
    MSP_SET_RAW_GPS = 201
    MSP_SET_PID = 202
    MSP_SET_BOX = 203
    MSP_SET_RC_TUNING = 204
    MSP_ACC_CALIBRATION = 205
    MSP_MAG_CALIBRATION = 206
    MSP_SET_MISC = 207
    MSP_RESET_CONF = 208
    MSP_SET_WP = 209
    MSP_SWITCH_RC_SERIAL = 210
    MSP_IS_SERIAL = 211
    MSP_DEBUG = 254

    def __init__(self, ser_port="/dev/ttyAMA0"):
        self.ser = serial.Serial()
        self.ser.port = ser_port
        self.logger = logging.getLogger("MSP")
        self.is_initialized = False

    def initialize_serial(self, baudrate=115200):
        """Initializes the serial port."""
        if not self.is_initialized:
            try:
                self.ser.baudrate = baudrate
                self.ser.bytesize = serial.EIGHTBITS
                self.ser.parity = serial.PARITY_NONE
                self.ser.stopbits = serial.STOPBITS_ONE
                self.ser.timeout = 0.05  # 50ms timeout
                self.ser.xonxoff = False
                self.ser.rtscts = False
                self.ser.dsrdtr = False
                self.ser.open()
                self.is_initialized = True
                self.logger.info(f"Serial port {self.ser.port} initialized at {baudrate}")
            except Exception as e:
                self.logger.error(f"Error opening serial port {self.ser.port}: {e}")
                raise

    def close_serial(self):
        """Closes the serial port."""
        if self.is_initialized:
            try:
                self.ser.close()
                self.is_initialized = False
                self.logger.info(f"Serial port {self.ser.port} closed")
            except Exception as e:
                self.logger.error(f"Error closing serial port {self.ser.port}: {e}")

    def send_command(self, msp_code, data=[]):
        """Sends an MSP command with optional data payload."""
        self.initialize_serial()

        data_length = len(data)
        checksum = 0
        buffer = bytearray()
        buffer.extend(map(ord, self.MSP_HEADER))
        buffer.append(data_length)
        buffer.append(msp_code)
        checksum ^= data_length ^ msp_code

        for d in data:
            buffer.append(d)
            checksum ^= d
        buffer.append(checksum)

        try:
            self.ser.write(buffer)
            self.logger.debug(f"Sent MSP command: {msp_code}, Data: {[x for x in data]}")
        except Exception as e:
            self.logger.error(f"Error sending MSP command {msp_code}: {e}")

    def receive_data(self, msp_code, num_of_data_expected=0):
        """Receives data from the MSP device."""
        start_time = time.time()
        timeout = 1  # seconds
        
        
        while (time.time() - start_time) < timeout:
          if self.ser.in_waiting > 0:
            header = self.ser.read(3).decode('utf-8', errors='replace')
            if header == self.MSP_HEADER:
                data_length = self.ser.read(1)[0]
                received_msp_code = self.ser.read(1)[0]
                checksum = data_length ^ received_msp_code

                if received_msp_code == msp_code:
                    data = bytearray()
                    for _ in range(data_length):
                        byte = self.ser.read(1)[0]
                        data.append(byte)
                        checksum ^= byte

                    received_checksum = self.ser.read(1)[0]
                    if received_checksum == checksum:
                        self.logger.debug(f"Received MSP data for code {msp_code}: {data.hex()}")
                        return data
                    else:
                        self.logger.error(f"Checksum mismatch for MSP code {msp_code}")
                        return None
            else:
                self.logger.warning(f"Received unexpected header: {header}")
        self.logger.warning(f"Timeout waiting for MSP data for code {msp_code}")
        return None

    def decode_data(self, msp_code, data):
        """Decodes received MSP data based on msp_code."""
        try:
            if msp_code == self.MSP_RC:
                # Assuming 8 channels, 2 bytes each
                channels = struct.unpack('<8H', data)  
                self.logger.info(f"Decoded RC data: Channels: {channels}")
                return channels
            elif msp_code == self.MSP_STATUS:
                # Decoding logic for MSP_STATUS
                # Assuming it returns a tuple of (cycleTime, i2cError, sensors, mode)
                cycleTime, i2cError, sensors, mode = struct.unpack('<HHHI', data)
                self.logger.info(f"Decoded Status data: CycleTime={cycleTime}, i2cError={i2cError}, Sensors={sensors}, Mode={mode}")
                return cycleTime, i2cError, sensors, mode
            elif msp_code == self.MSP_IDENT:
                # Decoding logic for MSP_IDENT
                # Assuming it returns a tuple of (version, multitype, msp_version, capability)
                version, multitype, msp_version, capability = struct.unpack('<BBBI', data)
                self.logger.info(f"Decoded Ident data: Version={version}, Multitype={multitype}, MSPVersion={msp_version}, Capability={capability}")
                return version, multitype, msp_version, capability
            elif msp_code == self.MSP_MOTOR:
                # Decoding logic for MSP_MOTOR
                # Assuming it returns a tuple of motor values (e.g., 8 motors, 2 bytes each)
                motors = struct.unpack('<8H', data)
                self.logger.info(f"Decoded Motor data: Motors={motors}")
                return motors
            elif msp_code == self.MSP_ALTITUDE:
                # Decoding logic for MSP_ALTITUDE
                # Assuming it returns a tuple of (altitude, vario) in centimeters and decimeters/second
                altitude, vario = struct.unpack('<iH', data)
                self.logger.info(f"Decoded Altitude data: Altitude={altitude} cm, Vario={vario} dm/s")
                return altitude, vario
            else:
                self.logger.warning(f"Decoding not implemented for MSP code: {msp_code}")
                return data  # Return raw data if not decoded
        except struct.error as e:
            self.logger.error(f"Error decoding data for MSP code {msp_code}: {e}")
            return None

# Example usage (in a separate file, e.g., main.py):
if __name__ == "__main__":
    msp = MSP()
    msp.initialize_serial()
    
    # Example of sending a command and receiving/decoding data
    msp.send_command(msp.MSP_RC)
    rc_data = msp.receive_data(msp.MSP_RC, 16)  # Assuming 16 bytes for RC data
    if rc_data:
        decoded_rc_data = msp.decode_data(msp.MSP_RC, rc_data)
        print("Decoded RC data:", decoded_rc_data)
    
    msp.send_command(msp.MSP_MOTOR)
    motor_data = msp.receive_data(msp.MSP_MOTOR, 16)  # Assuming 16 bytes for RC data
    if motor_data:
        decoded_motor_data = msp.decode_data(msp.MSP_MOTOR, motor_data)
        print("Decoded RC data:", decoded_motor_data)
        
    msp.send_command(msp.MSP_ALTITUDE)
    altitude_data = msp.receive_data(msp.MSP_ALTITUDE, 6)  # Assuming 16 bytes for RC data
    if altitude_data:
        decoded_altitude_data = msp.decode_data(msp.MSP_ALTITUDE, altitude_data)
        print("Decoded RC data:", decoded_altitude_data)

    msp.close_serial()