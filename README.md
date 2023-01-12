# F446 DRONE SHIELD
This repository is for a Nucleo-F446RE shield designed by CHKim. 

The 'Nucleo-F446RE' and 'PC(USB)'.

Tested in:

* Ubuntu 20.04LTS with ROS Noetic

1.Installation
------
    cd ~/{$YOUR_WORKSPACE}/src

    git clone https://github.com/ChanghyeonKim93/f446_drone_shield_ros.git

    cd .. && catkin_make (or catkin build f446_drone_shield_ros)
    

2.Usage
------
    roslaunch f446_drone_shield_ros run.launch 
    
   
Befure usage, please set a serial portname (refer the below documentation to set the permanent udevrules name.), topicnames, baudrate and so on.

3.Update USB udevrules
------
First, find a vendor ID and product ID of your device by the below command:

    lsusb
    
Then, you might get like this:

    Bus 001 Device 011: ID 0483:374b STMicroelectronics ST-LINK/V2.1

'0483' is the vecdor ID and '374b' is the product ID. After, find a distinct serial number of your device (I assume your device is autoallocated on /dev/ttyACM0) by:

    udevadm info -a -n /dev/ttyACM* | grep '{serial}' | head -n1

Then, you could see like:

    ATTRS{serial}=="0672FF494851871222051346"

0672FF494851871222051346 is the very your device's distinct serial number. (Not always distinct. When using multiple usb devices, please check whether devices have same serial number.)

Make udevrules file at:

    cd /etc/udev/rules.d

    sudo gedit 99-nucleo-serial.rules
    
In this file, type as below and save the file:

    SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374b", ATTRS{serial}=="0672FF494851871222051346", SYMLINK+="nucleo_l432kc"

As you can figure out, you can replace 'nucleo_l432kc' with any name you want (but should be distinct).