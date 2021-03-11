# Software Setup

Install the following software on your Odroid XU-4.

## OS 
Start by installing a Linux image (MATE) on the computer memory. Hardkernel provides the images and a tutorial in their [Odroid Wiki](https://wiki.odroid.com/getting_started/os_installation_guide).
You will need a Micro SD card reader to connect the memory and its adapter to your PC.

Alternatively, you can buy a memory module with the OS already installed.

## WiFi Drivers
Many USB WiFi dongles are not directly supported on all Linux kernels. If your WiFi module is not working, you likely need to install your drivers manually. For the WiFi module in the [Bill of Materials](https://github.com/robertcornet/TUD_AI_driving/blob/main/documentation/bill_of_materials.md), use these [drivers](https://github.com/lwfinger/rtl8723bu).

## Robot Operating System (ROS)
Install the correct ROS distribution for your Linux version. [ROS](https://www.ros.org/) is used to communicate between your Arduinos and your computer. If you are unfamiliar with ROS, follow their excellent [Beginner Level turtorials](http://wiki.ros.org/ROS/Tutorials)

## TensorFlow / TensorFlow Lite
Install [TensorFlow](https://www.tensorflow.org/) or [TensorFlow Lite](https://www.tensorflow.org/lite) if you want to execute your trained Neural Networks on the car.
Installing the full version of TensorFlow on an ARM system such as the Odroid can be difficult. Compiling the Tensorflow Wheel yourself is slow and requires some experience.
[This guide](https://www.jianshu.com/p/375cacb4c0f2) will help you to install a [precompiled ARM version](https://github.com/lhelontra/tensorflow-on-arm/releases) of TensorFlow on your Odroid XU-4.


## Remote Access
You can access your Odroid remotely over the internet. If you want to cast the entire screen of the Odroid, you can use [NoMachine](https://www.nomachine.com/). Remote access is a lot slower than working on the computer directly. Connect to the Odroid over USB/HDMI directly for longer sessions.
