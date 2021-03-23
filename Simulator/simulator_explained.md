# The Simulator Explained

The simulator can be used to develop and test your controllers. From the two inputs (Steering and Throttle), it will approximate how the car would behave.

## overview of functions
The simulator is built in the [OpenAI GYM](https://gym.openai.com/) structure.
The following are the main functions:

def _ _init_ _()


## Using the simulator
- Define your objective and what states you have observable in your experiment
- Choose the time step for the agent (eg, 10 hz, 25hz, etc). The simulator updates at 1000hz for stable integration. 
- Make sure only the observable states are returned to your learning algorithm!
- Define how you want an episode to start (*def reset()*) and when to end (*done*)
- Construct your reward so it promotes the desired behaviour


## SIMULATING LIDAR

The simulator supports simulating lidar behaviour in a few different ways.

1,2: Ray Tracing
tracing each ray and determining the distance to the nearest wall-line segment.
A wall-line can be:
1. left/right wall of the track.
2. the walls of a room

This methond only measures the distance in the center of the FOV of the sensor.

3, FOV:
determining the distance between the car and an object, and triggering all sensors for which that object is in its FOV. 
This is used to detect the leading car, in case of car following



## Learning Algorithms
The simulator is used in combination with the Reinforcement Learning programs from the [RLControlTheoreticGuarantee Github](https://github.com/RLControlTheoreticGuarantee/Guarantee_Learning_Control). 

Inluded on this page is a folder with an older copy of that github, including the vehicle simulator. 
If you follow their installation guide, you can download the folder above and start training immediately.

## Electronic Speed Control

The ESC puts out a voltage to the motor depending on the input signal. 
The input is a PWM signal with pulses between 1000us (full throttle) and 2000us (full braking).
This behaviour is modelled in the simulator as a voltage multiplier; the ESC acts as a multiplier in range [0-1] between the battery voltage and motor.
For the TBLE-02S ESC included in the Tamiya kit, the following values were measured:


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

## The Motor
Tamiya specifies a motor KV (RPM per volt) of 2222. Our measurements revealed a KV closer to 2775:
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





## The vehicle simulator is a OpenAI environment written in Python. 

# inputs/outputs
The simulator has 2 inputs:
- Steering
- Throttle

These are values in range -1,1 which represents 100% in either direction. 


Based on these inputs, it will run 1 simulation step, lets say 0.1 seconds. It then returns the following:
- new state
- reward
- done (true/false)

## states
In the simulator you have access to the following vehicle states:

- longitudinal velocity u
- lateral velocity v
- yaw rate r
- longitudinal acceleration ax
- lateral acceleration ay
- wheel rotation velocities (4x)

Additionally, you can simulate what ultrasonic sensors would detect in different scenarios:
- detecting room/racetrack borders
- single object detection (eg. determining distance to leading car)



## See also
For more specific details on the specifics of the vehicle motion simulation, refer to Chapter 2 of [this Thesis](https://repository.tudelft.nl/islandora/object/uuid%3A7bedb60a-ced8-4fcf-97ca-80208861a413)
