# Vehicle motion simulator

The simulator can be used to develop and test your controllers. From the two inputs (Steering and Throttle), it will approximate how the car would behave. The simulator is built in the [OpenAI GYM](https://gym.openai.com/) structure.

The simulator is used in combination with the Reinforcement Learning programs from the [RLControlTheoreticGuarantee Github](https://github.com/RLControlTheoreticGuarantee/Guarantee_Learning_Control). 

Inluded on this page is a folder with an older copy of that github, including the vehicle simulator. 
If you follow their installation guide, you can download the folder above and start training immediately.


## Using the simulator
- Define your objective and what states you can observe in your experiment. Make sure your controller does not rely on states that you can not observe in the real world!
- Choose the time step for the agent (eg, 10 hz, 25hz, etc). The vehicle simulation updates at 1000hz for stable integration. 
- Define how you want an episode to start (*def reset()*) and when to end (*done*)
- Construct your reward so it promotes the desired behaviour


## States
In the simulator you have access to the following vehicle states:

- longitudinal velocity *u*
- lateral velocity *v*
- yaw rate *r*
- longitudinal acceleration *a<sub>x</sub>*
- lateral acceleration *a<sub>y</sub>*
- wheel rotation velocities *Ω* 

Additionally, you can simulate what ultrasonic sensors would detect in different scenarios:
- detecting room/racetrack borders (lidar)
- single object detection (eg. determining distance to leading car)


## Primary Simulator Functions
The most important functions are:

*def* \_\_init\_\_()\
Here all the parameters are set for the car and the environment. For example, here you would change the mass, the dimensions, or the motor characteristics of the car. This is also were you set if your car is 4wd or rwd, and how large the time steps of the simulation will be.

*def* reset()\
This function is called at the beginning of every training episode. It (re)sets the parameters for the episode. Here you can set how and where your vehicle starts in the environment. It can also be used to build a different enviroment for every episode.

*def* step()\
This is the main simulation function. It takes two inputs (steering and throttle), and runs the simulation for one step in the environment.  
It returns the new state of the car and environment, and a reward based on that state. 

## Secondary Simulator functions

### Motor and ESC
*def* ESC()\
This function simulates the behaviour of the electronic speed controller (ESC) included in the Tamiya kit (TBLE-02S).\
The ESC puts out a voltage to the motor depending on the input signal. The input is a PWM signal with pulses between 1000us (full throttle) and 2000us (full braking). This behaviour is modelled in the simulator as a voltage multiplier; the ESC acts as a multiplier in range [0-1] between the battery voltage and motor.\
The folling values were measured by experiment:

|Throttle %|Voltage Mult.|
|---|---|
|0|0|
|7.5|0|
|7.6|0.44|
|10|0.53|
|20|0.75|
|30|0.84|
|40|0.89|
|50|0.94|
|60|0.96|
|70|1|
|80|1|
|90|1|
|100|1|

*def* motor()\
This function calculates the torque that the motor will produce, given the voltage from the ESC and its rotational speed.

Tamiya specifies a motor RPM per volt (KV) of 2222, and a stall torque per volt (TV) of 0.0042 Nm. Our measurements revealed a KV closer to 2775, and a TV of 0.0065 Nm.

|Voltage|Motor RPM|KV|
|---|---|---|
|3.45|9322|2702|
|4.09|11376|2781|
|5.82|16116|2769|
|6.57|18179|2765|
|6.95|19276|2774|
|7.30|20540|2813|
|7.48|20935|2798|
|7.79|21725|2789|

### Tires
TIRE_FLUCTUATION is an array of 4 values. It can be used to add a little noise on the tire behaviour of the individual wheels, that changes from episode to episode.

### LIDAR
*def* lidar_get_state()\
Returns the distance to the walls at the angles set in SENSOR_ARRAY (*def* \_\_init\_\_()).\
*def* lidar_get_state() calls *def* lidar_rayDist(). There the distances can be checked in two different ways:
1. To the walls of a room (self.track__center)
2. To the 'walls' of a race track (self.track_left, self.track_right)\
Both options are quite different, make sure to use the right one!

### Room generation
*def* gen_room()\
Very simple function that generates a square room given a length and width in meters. It returns the x,y-coordinates of the corners.

### Track generation
*def* track_generator()\
Generates a random track. It is adapted from the methods discussed [here](http://blog.meltinglogic.com/2013/12/how-to-generate-procedural-racetracks/)

*def* track_check_valid()\
Checks if the generated random track is 'valid'. If a track has 'kinks' in the corners, the track will be 'invalid'. 


## Expirimental

### Car following, leading car
These functions are for running a leading car. You can use it to train your AI to follow a car using lidar or ultrasonic sensing.

*def* initialize_leading_car()\
Initializes the leading car in a random position in front of your own car.

*def* run_leading_car()\
Chooses the actions for the leading car. Limits for the speed, acceleration and yaw rate of the leading car are set in this function.

*def* get_radar_state_zones()\
This function tries to simulate how an ultrasonic sensor would see the leading car in practice. If the leading car is within the FOV of a sensor, it returns the distance between the two cars.  

## See also
For more details of the vehicle motion simulation, refer to Chapter 2 of [this Thesis](https://repository.tudelft.nl/islandora/object/uuid%3A7bedb60a-ced8-4fcf-97ca-80208861a413)
