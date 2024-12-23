# o3pi_arm
Raspberry Pi 4 Configuration Steps

## Enable UART:
Run `sudo raspi-config`

Go to Interface Options -> Serial Port

Select No for "Would you like a login shell to be accessible over serial?"

Select Yes for "Would you like the serial port hardware to be enabled?"

Reboot the Raspberry Pi.

Identify Serial Port: The primary UART on the Raspberry Pi 4 is usually `/dev/ttyAMA0`. You can verify this by running `ls /dev/tty*`.

## Service Setup (using systemd)
### Create a Service File:
Create a file named `msp_service.service` in `/etc/systemd/system/`:
```
[Unit]
Description=MSP Service
After=network.target

[Service]
ExecStart=/usr/bin/python3 /home/pi/msp_project/main.py
WorkingDirectory=/home/pi/msp_project
StandardOutput=inherit
StandardError=inherit
Restart=always
User=pi

[Install]
WantedBy=multi-user.target
```
Explanation:
* Description: A brief description of your service.
* After: Specifies that the service should start after the network is up.
* ExecStart: The command to execute to start your service. Make sure to replace /home/pi/msp_project/main.py with the actual path to your main Python script.
* WorkingDirectory: The working directory for your service.
* StandardOutput and StandardError: These are set to inherit so you can see the output of your script in the system logs.
* Restart: Set to always so the service automatically restarts if it crashes.
* User: The user to run the service as. It's generally recommended not to run services as root.
* WantedBy: Specifies when the service should be started during boot. multi-user.target is a common target for services that should be running when the system is ready for users to log in.
## Enable and Start the Service:
* Reload systemd: ```sudo systemctl daemon-reload```
* Enable the service: ```sudo systemctl enable msp_service.service```
* Start the service: ```sudo systemctl start msp_service.service```
* Check Service Status: ```sudo systemctl status msp_service.service```
### View Logs:
`sudo journalctl -u msp_service.service` or check the log file directly: `tail -f /home/pi/msp_project/msp_log.txt` (or wherever you configured the log file to be).

## Directory Structure
```
msp_project/
├── main.py       # Main script
└── msp.py        # MSP library
```
## Installation of Python Packages
Run in your terminal following command:
```
pip install pyserial
```
## Important Notes:
Error Handling: The Python code includes basic error handling (try-except blocks) for serial communication and data decoding. You should expand this to handle
