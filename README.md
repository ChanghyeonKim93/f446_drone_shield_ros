# F446 DRONE SHIELD
<p align = "left">
<img src= "https://github.com/ChanghyeonKim93/f446_drone_shield_ros/blob/master/imgs/example_images.png" width="960">
</p> 

This repository is for **F446_Drone_Shield** designed by CHKim. 

Main role of this package is to make a serial communication between 'Nucleo-F446RE' and 'PC(USB)'.

F446_Drone_Shield can provide belows:

* Serial communication between Nucleo-F446RE and PC via USB 2.0 (3.0 OK). Default baudrate is 460800.

* 8 x PWM outputs with configurable frequencies (500 Hz default, 1 ~ 3,000 Hz tested, HIGH level: 3.3 volt.)

* 2 x analog reads (0 ~ 3.3 Volts). ADC1 has an optional voltage divider (one over 11) which can be selected by selection pin.
** Note that the ADC read values jitter somewhat. (up to 100 mV)

* 2 x 3.3 V , 2 x 5.0 V pinouts. (only for peripheral MEMS sensors with low power consumption)
    
* 1 x Sonar distance sensor. 5 cm ~ 330 cm from HC-SR04, at 100 Hz  **(Sonar sensor is not included on the shield.)**

* 1 x IMU data. 3-D acc. 3-D gyro from Invensense ICM-42605, up to 1 kHz **(IMU is not included on the shield.)**

Tested in:

* Ubuntu 18.04LTS with ROS melodic
* Ubuntu 20.04LTS with ROS noetic

1.Installations
------
* Prerequisites
    * ROS1
    * Mbed studio

* Linux (PC)
```
cd ~/{$YOUR_WORKSPACE}/src
git clone https://github.com/ChanghyeonKim93/f446_drone_shield_ros.git
cd .. && catkin build f446_drone_shield_ros
``` 

* Nucleo-F446RE (requires Mbed Studio. Both Linux or Windows OK)
    1. Install Mbed Studio. You can download from the STMicro homepage. Any version is OK.
    2. Make a project in the Mbed Studio. (empty project)
    3. Copy the **all files in 'mbed_code'** folder of this repository to the project folder.
    4. In Mbed Studio, go to the directory: **mbed-os -> targets -> TARGET_STM -> TARGET_STM32F446xE -> TARGET_NUCLEO_F446RE**
    5. Overwrite the original 'PeripheralPins.c' file by **the provided 'PeripheralPins.c'**.
    
        This procedure is required to additionally use the PWM 4,5,6,7 (TIM8, PA6,PA7,PA8,PA9 pins). In default, only PWM 0,1,2,3 are provided. Note that PA6,PA7,PA8,PA9 pins are originally mapped to TIM3.
        
    6. Build the program in Mbed Studio **in Release Mode** and upload it to the Nucleo board.


2.Usage
------
* Before usage, please check and set a **serial portname** (refer the below documentation to set the permanent udevrules name.), topicnames, baudrate and so on.



* Topics you can subscribe

  `/icm42605/imu`(sensor_msgs::Imu): ICM42605 IMU streaming

  `/battery_state/0`(sensor_msgs::BatteryState): Voltage read (VIN 0)

  `/battery_state/1`(sensor_msgs::BatteryState): Voltage read (VIN 1)

  `/hcsr04/range`(sensor_msgs::Range): Sonar distance sensor data

* Topic you can publish to control PWM

  `/serial/pc/to_fmu`(std_msgs::UInt16MultiArray): pwm signal data
  
  * You can send the **pwm command** by publishing the **"/serial/pc/from_fmu"** topic from your own rosnode.
    * Message type: `UInt16MultiArray`
    * Message format: PWM1,PWM2, ...., PWM8 (total 32 bytes)
      * Each byte contains PWM signal (unsigned short, 0~65535).
    
* Then, execute the launch.
```
roslaunch f446_drone_shield_ros run.launch 
``` 


3.PCB Schematics
------
* You can see the KiCAD PCB schematic of the 'F446_DRONE_SHIELD' as belows:

<p align = "left">
<img src= "https://github.com/ChanghyeonKim93/f446_drone_shield_ros/blob/master/imgs/schematics.PNG" width="960">
</p> 
<p align = "left">
<img src= "https://github.com/ChanghyeonKim93/f446_drone_shield_ros/blob/master/imgs/pcb.PNG" width="360">
</p> 

4.Trouble Shooting
------

* **Update udev_rules**

    First, find a vendor ID and product ID of your device by the below command:

        lsusb

    Then, you might get like this:

        Bus 001 Device 011: ID 0483:374b STMicroelectronics ST-LINK/V2.1

    '0483' is the vecdor ID and '374b' is the product ID. After, find a distinct serial number of your device (I assume your device is autoallocated on /dev/ttyACM0) by:

        udevadm info -a -n /dev/ttyACM* | grep '{serial}' | head -n1

    Then, you could see like:

        ATTRS{serial}=="0672FF494851871222051346" // It differs depending on the board.

    0672FF494851871222051346 is the very your device's distinct serial number. (Not always distinct. When using multiple usb devices, please check whether devices have same serial number.)

    Make udevrules file at:

        cd /etc/udev/rules.d
        sudo gedit 99-nucleo-serial.rules

    In this file, type as below and save the file:

        SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374b", ATTRS{serial}=="0672FF494851871222051346", SYMLINK+="nucleo_f446re"

    As you can figure out, you can replace 'nucleo_f446re' with any name you want (but should be distinguished).
