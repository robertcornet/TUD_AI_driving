# The Simulator Explained

## Learning Algorithms
The simulator is used in combination with the Reinforcement Learning programs from the [RLControlTheoreticGuarantee Github](https://github.com/RLControlTheoreticGuarantee/Guarantee_Learning_Control). 

Inluded on this page is a folder with an older copy of that github, including the vehicle simulator. 
If you follow their installation guide, you can download the folder above and start training immediately.


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

## Using the simulator
- Define your objective and what states you have observable in your experiment
- Choose the time step for the agent (eg, 10 hz, 25hz, etc). The simulator updates at 1000hz for stable integration. 
- Make sure only the observable states are returned to your learning algorithm!
- Define how you want an episode to start (*def reset()*) and when to end (*done*)
- Construct your reward so it promotes the desired behaviour


## See also
For more specific details on the specifics of the vehicle motion simulation, refer to Chapter 2 of [this Thesis](https://repository.tudelft.nl/islandora/object/uuid%3A7bedb60a-ced8-4fcf-97ca-80208861a413)
