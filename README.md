# MATE_2022
This is the vision code used for the 2022 competition. It detects various objects and goals while also serving as a live feed for the drivers. 
The current implementation of this code is meant to be ran on a Raspberry Pi 4 running WPILib's Pi-Gen image. This doesn't do vision for us, it simply provides a platform for development. It includes OpenCV, cscore (Camera Server), ntcore (NetworkTables), and a very nice web dashboard for managing the RPI's status.

### Steps to get things running:
1) Flash the image at the link below onto the rpi. To talk to the rpi, you will need to connect it to a network via ethernet. (The rpi's video output is disabled by default to save resources.)
2) Upload the precompiled VISION file (or compile this repositories code yourself, and then upload the executable) via the web dashboard. Use hostname wpilibpi.local to connect to the interface.
3) Setup any attached USB cameras via the web dashboard's 'Vision Settings' tab.
4) Restart the program via the web dashboard's 'Vision Status' tab.
5) Hopefully yeet the robot into the water and watch everything work perfectly. 

NOTE: The camera feeds are HTTP streams that start at port 1181 and go up.

RPI Image Download: https://github.com/wpilibsuite/WPILibPi/releases
