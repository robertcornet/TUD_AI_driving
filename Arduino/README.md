# Arduino

The sensors on the vehicle are connected to 2 Arduino Nano's. The Arduinos then publish the sensor information over USB on the ROS network.


## Arduino 1 - Vehicle sensors
The first Arduino int

## Arduino 2 - Ultrasonic sensors
The second Arduino is for the ultrasonic sensors. There are 2 files included that use the sensors in a different way.

### radar_sequential.ino
In this file the 8 sensors are read one after the other:

> trigger sensor n  
> if (response received or timeout expired (no response received)){  
> trigger sensor n+1  
>}

> every 40 ms:
> publish latest sensor values on ROS network
  
### radar_parallel.ino

> every 40 ms:
> publish latest sensor values on ROS network
  
> trigger all sensors simultaneously
> wait for responses
