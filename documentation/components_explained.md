# Componenents Explained

This page explains the choice of individual components, as well as how they are used, and what can be improved.

## The wheel speed sensors 
The rotational velocity of the wheels is important information when vehicle dynamics are considered. It is often the only measurement we have of the interface between the vehicle and the road.
It can be used to determine the velocity of the vehicle (from the non-driven wheels) or to estimate the longitudinal slip of the driven wheels.

Conventionally, wheel speeds are measured with hall sensors and a magnetic ring. This is also the method used on a large research platform such as the [AutoRally project](https://autorally.github.io/).

In our case, the rotational velocity is measured with IR-reflectance sensors. A small IR transmitter and receiver board is mounted on each wheel hub. A disk is mounted inside the wheels with aluminium tape to reflect the IR light. The gaps in the disk obviously do not reflect light, generating a square wave signal when the wheel is rotating.

IR-reflectance sensors can typically be bought in a **'Digital'** or **'Analog'** version. 
A digital sensor determines the reflectivity of a surface by pulling a voltage to 'HIGH', and counting how long it takes to discharge a small capacitor due to the amount of light reflected. This is a very slow process, and this gives no information about when the surface switches from 'light' to 'dark'. 

A much better solution is to use an **Analog** version of the board. An analog board puts out a voltage as a function of the reflected light. Since we are only interested in 'light' or 'dark', we can read the sensor with a **DIGITAL** pin on the arduino. Whenever the surface switches from **'light' to 'dark',** the voltage switches from **HIGH to LOW**, and an Interrupt routine is called to note the time of the switch.

Other projects use encoder disks with a black and white pattern to create a 'light' and 'dark' surface. This is not sufficient; a dark colored surface can still reflect IR light. A much more robust 'dark' surface is a surface that is sufficiently far away, regardless of color. Therefor a robust solution was found in using 'slotted' encoder disks.

An analog board with a **47k** resistor works best. Boards with a lower resistance will not reliable pull the voltage low enough to trigger Arduino's Digital LOW threshold.

## IMU

The vehicle is equiped with a low-cost 9-dof IMU:[SparkFun 9DoF IMU Breakout - ICM-20948](https://www.sparkfun.com/products/15335). 

This is used to measure the longitudinal and lateral accelerations.
The accelerations are measured in a plane perpendicular to gravitation vector. The orientation of the board is determined by a switching filter. This compensates the drift of the gyros and the noise of the accelerometers.

### Switching Filter	
If no motion is detected for a certain amount of time, the board is assumed to be stationary.
If the board is stationary; the orientation is determined by a weighted sum between the Accelerometers and the Gyrometers:

alpha = 0.95*(Gyro)+0.05*(Accel.)

When the board is in motion, the accelerometer data can no longer be used to accuratelyseperate the direction of the gravitational vector and the specific motion of the board. The angle is then solely determined by the Gyrometers:

alpha = 1.0*(Gyro)

### Inertial Navigation
The longitudinal and lateral velocity can be obtained by integrating the measured accelerations. In practice the usability of the calculated velocities is extremely limited. The calculated values are transmitted on the ROS network, but it is recommended that they should not be trusted.

## POWER
Power is supplied to the odroid with a 6 amp BEC(battery Eliminating Circuit). This is a common component in the RC hobby. Alternatively any DC/DC converter buck can be used that outputs 5V at 4+ A.

## Batteries
Power is provided to the car with Traxxas Lipo Batteries. They are very common, and easy to charge and maintain with plug and play chargers.

Do not let the voltage get too low. The vehicle will drain the battery completely if it is not disconnected. The battery monitor on the vehicle provides a reference for the voltage. If the voltage gets too low, the alarm will sound. 
The individual cell voltage can not be measured, only the sum (total voltage) is valid.

## Ultrasonic sensors
Ultrasonic sensors are used as a cheap way to observe the environment as an alternative to using camera vision. 

The vehicle can be equipped with a number of ultrasonic sensors in different configurations. The intention was to use ultrasonic sensors as a low cost alternative to lidar sensors. However, in testing it turns out that the usability of ultrasonic sensors is limited:

1. If the sensor is not pointing perpendicular to the surface it is supposed to measure, the sound waves will not bounce back to the receiver and the surface will not be observed correctly.

2. The relative slow speed of sound means that the sensors can not be polled at rates over 30hz. Normally, each sensor is polled individually, and the next sensor in the array is polled only after the first has received its echo (or has timed out). This would slow the update rate down to unusable levels for multiple sensors. 

3. The sensors have a fairly wide, and not always consistent FOV. This means that a sensor will pick up any object in its FOV, not only the distance straigth ahead.
