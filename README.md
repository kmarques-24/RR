# RESCUE Roller operating system built atop the ESP-IDF and microROS.
## Overview
This code allows a RESCUE Roller to be teleoperated, estimate its odometry, and act as a micro-ROS node. 

| File | Function |
| -------- | -------- |
| main.cpp | Initializes sensors and services |
| imu_service.cpp (/.h) | Reads and stores latest IMU reports |
| tof_service.cpp (/.h) | Polls and stores latest ToF reading |
| hardware_encoders.cpp (/.h) | Interrupts to track motor encoder position & speed |
| hardware_motors.cpp (/.h) | Allows motor duty cycle set |
| controller.cpp (/.h) | Drives commanded twist via PID speed control |
| estimator.cpp (/.h) | Augments encoders with IMU to complete odometry |
| uros_service.cpp (/.h) | Publishes odometry and ToF point cloud, subscribes to twist commands from keyboard |

To add new sensor, write service script with initialization protections, update main.cpp, edit uros_service.cpp to publish data (if applicable), and edit estimator.cpp to use readings to refine odometry (if applicable). 

## Requirements
* ESP-IDF v5.1.2 for VScode
* micro_ros_esp_idf_component (humble) fork at kmarques-24
* esp32_BNO08x fork at kmarques-24
* VL53L5CX-Library fork at kmarques-24
* Router

The forked repos have necessary compatibility edits.

### Partner Repository
Meant for use with: https://github.com/kmarques-24/rr_py  

Designed to take twist commands from ros2 teleop_twist_keyboard.

## Getting Started
Clone the RR repo by running the following command in a directory of choice:
```
git clone https://github.com/kmarques-24/RR
```

In RR, run the following to start working in VScode:
```
source $HOME/esp/v5.1.2/esp-idf/export.sh
code .
```

In the top level of the repository run the following to pull in the component repositories:
```
git submodule init
git submodule update
```

IMPORTANT FOR COMPATIBILITY to run the following to downgrade empy:
(These lines assume that 'esp' was installed in your home directory.) 
```
cd ~/esp/v5.1.2/esp-idf
source $HOME/esp/v5.1.2/esp-idf/export.sh
pip3 install catkin_pkg lark-parser colcon-common-extensions 
pip3 uninstall empy
pip3 install empy==3.3.4
```

### Router
Plug in router and wait a minute for network RR to become visible. 

Connect and open http://192.168.8.1/ to see connected clients (laptop, esp32) and IPs.

## Building and Developing
Always run before using idf.py commands:
```
source $HOME/esp/v5.1.2/esp-idf/export.sh
```

Build target needs to be set to esp32s3. Doing so resets configuration, so first delete sdkconfig. 
Then set target and load default config:
```
idf.py set-target esp32s3
idf.py reconfigure
```

To change configuration, run the following, make edits, hit S to save sdk.config:
```
idf.py menuconfig
```
Optional - save new sdk.config as default:
```
idf.py save-defconfig
```

In idf.py menuconfig, check the following: 
- Serial flasher config -> Flash size -> 4 MB (or higher)
    - Can be up to 8 MB for ESP32S3-Sense or 16 MB for ESP32S3-Plus
- Partition Table -> Partition Table -> Custom partition table CSV
- Partition Table -> Custom partition CSV file -> partitions.csv
- Custom: Rescue Roller Configuration -> 
    - Check ID, motor type, wheel radius, and wheel base
- Custom: ToF Sensor Configuration -> 
    - Check correct GPIO
- Custom: Brushed (or Brushless) Motor Configuration -> 
    - Check correct CPR and GPIO
- Custom: WiFi Configuration -> 
    - SSID and Password should match those of router
- esp32_BNO08x -> GPIO Configuration ->
    - Check correct GPIO for SPI
- micro-ROS Settings -> WiFi Configuration -> 
    - SSID and Password should match those of router
- micro-ROS Settings -> micro-ROS Agent IP ->
    - Should match host computer IP. Check this by looking at its address on the router
- micro-ROS Settings -> micro-ROS Agent Port -> 8888
- Component config -> Log output -> Default log verbosity -> Debug
    - Most useful for debugging with idf.py monitor

Build code: 
```
idf.py build
```

Build and flash code:
```
idf.py build
idf.py flash
```

Monitor and debug:
```
idf.py monitor
```
Final INFO output should show all modules initialized.  
If adding new modules, add proper initializion protections and logging.

## Additional Resources:

https://www.freertos.org/Documentation/00-Overview

https://docs.espressif.com/projects/esp-idf/en/v5.4.1/esp32s3/get-started/index.html
