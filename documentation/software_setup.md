# Software Setup

Install the following software on your Odroid XU-4.

## OS 
Start by installing a Linux image (MATE) on the computer memory. Hardkernel provides the images and a tutorial in their [Odroid Wiki](https://wiki.odroid.com/getting_started/os_installation_guide).
You will need a Micro SD card reader to connect the memory and its adapter to your PC.

Alternatively, you can buy a memory module with the OS already installed.

## WiFi Drivers
Many USB WiFi dongles are not directly supported on all Linux kernels. If your WiFi module is not working, you likely need to install your drivers manually. For the WiFi module in the [Bill of Materials](/documentation/bill_of_materials.md), use these [drivers](https://github.com/lwfinger/rtl8723bu).

## Robot Operating System (ROS)
Install the correct ROS distribution for your Linux version. [ROS](https://www.ros.org/) is used to communicate between your Arduinos and your computer. If you are unfamiliar with ROS, follow their excellent [Beginner Level turtorials](http://wiki.ros.org/ROS/Tutorials)

You can install the [package 'car'](/ros) and modify from there. 
The package is very minimal; it contains:
- Launch file for USB serial communnication
- custom messages for the vehicle state, radar state, and control actions
- a controller script that runs a Tensorflow session


## TensorFlow / TensorFlow Lite
Install [TensorFlow](https://www.tensorflow.org/) or [TensorFlow Lite](https://www.tensorflow.org/lite) if you want to execute your trained Neural Networks on the car.
Installing the full version of TensorFlow on an ARM system such as the Odroid can be difficult. Compiling the Tensorflow Wheel yourself is slow and requires some experience.
[This guide](https://www.jianshu.com/p/375cacb4c0f2) will help you to install a [precompiled ARM version](https://github.com/lhelontra/tensorflow-on-arm/releases) of TensorFlow on your Odroid XU-4.

## Remote Access
You can access your Odroid remotely over the internet. If you want to cast the entire screen of the Odroid, you can use [NoMachine](https://www.nomachine.com/). Keep in mind that remote access is slower than working on the computer directly; connect to the Odroid over USB/HDMI directly for longer sessions, or use SSH.

## Arduino

### IDE
Install the [Arduino IDE](https://www.arduino.cc/en/software/) to write code to the Arduinos.

### Libraries
Through Arduinos Library Managers, install the following libraries:
- [Rosserial library](https://www.arduino.cc/reference/en/libraries/rosserial-arduino-library/)
- [Sparkfun ICM-20948 library](https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary)

The vehicle has 2 of the new Arduino Nano boards, called the Arduino Nano Every. This is a board with the new Mega-AVR architecture, instead of the standard AVR architecture of the common Uno boards. (Note: the official terminology of Mega-AVR and AVR is different between Arduino and the chip maker.) Unfortunately, many libraries are not compatible without a little care of modification.
	
For the Rosserial Library:
    Use Arduino AVR board manager 1.8.4 or older

For the Sparkfun ICM library (for the sparkfun IMU):
1.	Open ICM_20948.cpp 
2.	Go to line 750, column 35
3. 	change
> digitalWrite(_ad0, _ad0val);

to

> digitalWrite(_ad0, (PinStatus)_ad0val);

### ROS Message files
The ROS communication relies on custom messages. The library for them can be generated as follows:
Creating message.h files for Arduino:

1. cd to your Arduino library folder
2. Delete ros_lib folder if you want to re-generate
> rosrun rosserial_arduino make_libraries.py 

### Code
The code for the Arduinos can be found [here](/Arduino)

