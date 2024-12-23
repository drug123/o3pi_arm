import time
import logging
from msp import MSP  # Import the MSP library

# Configure logging
logging.basicConfig(filename='msp_service.log', level=logging.DEBUG,
                    format='%(asctime)s - %(levelname)s - %(message)s')

if __name__ == "__main__":
    logger = logging.getLogger("Main")
    msp = MSP()

    try:
        msp.initialize_serial()

        while True:
            # Send RC data (example)
            rc_data = [1500, 1500, 1500, 1500, 1000, 1000, 1000, 1000] # Example 8 channel data
            msp.send_command(msp.MSP_SET_RAW_RC, rc_data)

            # Request and decode various data
            # Status
            msp.send_command(msp.MSP_STATUS)
            status_data = msp.receive_data(msp.MSP_STATUS, 11) # 11 bytes of data in status
            if status_data:
                decoded_status_data = msp.decode_data(msp.MSP_STATUS, status_data)
                logger.info(f"Decoded Status: {decoded_status_data}")
            
            # Ident
            msp.send_command(msp.MSP_IDENT)
            ident_data = msp.receive_data(msp.MSP_IDENT, 7) # 7 bytes of data in ident
            if ident_data:
                decoded_ident_data = msp.decode_data(msp.MSP_IDENT, ident_data)
                logger.info(f"Decoded Ident: {decoded_ident_data}")

            # Request Motor data
            msp.send_command(msp.MSP_MOTOR)
            motor_data = msp.receive_data(msp.MSP_MOTOR, 16)  # 8 motors, 2 bytes each
            if motor_data:
                decoded_motor_data = msp.decode_data(msp.MSP_MOTOR, motor_data)
                logger.info(f"Decoded Motor data: {decoded_motor_data}")

            # Request Altitude data
            msp.send_command(msp.MSP_ALTITUDE)
            altitude_data = msp.receive_data(msp.MSP_ALTITUDE, 6)  # Altitude (4 bytes) + Vario (2 bytes)
            if altitude_data:
                decoded_altitude_data = msp.decode_data(msp.MSP_ALTITUDE, altitude_data)
                logger.info(f"Decoded Altitude data: {decoded_altitude_data}")

            # Request RC data
            msp.send_command(msp.MSP_RC)
            rc_data = msp.receive_data(msp.MSP_RC, 16)  # 8 channels, 2 bytes each
            if rc_data:
                decoded_rc_data = msp.decode_data(msp.MSP_RC, rc_data)
                logger.info(f"Decoded RC data: {decoded_rc_data}")

            time.sleep(0.5)  # Adjust sleep time as needed

    except KeyboardInterrupt:
        logger.info("Keyboard interrupt received, closing serial port.")
        msp.close_serial()
    except Exception as e:
        logger.error(f"An error occurred: {e}")
        msp.close_serial()