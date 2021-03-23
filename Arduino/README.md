# Arduino
The sensors on the vehicle are connected to 2 Arduino Nano's. The Arduinos then publish the sensor information over USB on the ROS network.


## Arduino 1 - Vehicle sensors
The first Arduino is wired to all the vehicle specific sensors:

- Inertial Measurement Unit (IMU)
- Wheel speed sensors (4x)
- 3 Channel RC receiver 
- 
Every 40 milliseconds the sensor information is published on the ROS network.

This Arduino also controls the actuators:
- Steering out (servo)
- Throttle out (esc)

For the Throttle and Steering actions you can switch between two sources:
- Actions from the RC receiver (manual control)
- Actions from the ROS network (automated mode)


## Arduino 2 - Ultrasonic sensors
The second Arduino is for the ultrasonic sensors. There are 2 files included that use the sensors in a different way.

### radar_sequential.ino
In this file, a sensor is triggered after the previous sensor is finished:

> trigger sensor n  
>   
> *if* (response received or timeout expired (no response received)){  
>      *trigger sensor n+1*  
> }

> every 40 ms:  
> *publish latest sensor values on ROS network*
  
### radar_parallel.ino
In this file, all sensors are triggered at the same time, every 40 ms:

> every 40 ms:  
> *publish latest sensor values on ROS network*    
> *trigger all sensors simultaneously*
