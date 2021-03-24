# How it works
The provided ROS package has very few components. Our network has 3 'nodes', and we can send messages between these nodes.  
- Node 1: This node is on Arduino 1, and sends messages over USB with the topic 'vehicle_state'. It also listens to the messages from Node 3 with the topic 'control_actions'.  
- Node 2: This node is on Arduino 2, and sends messages over USB with the topic 'radar_state'.  
- Node 3: This is the controller that runs on the computer. It listens to Nodes 1 and 2 about the 'vehicle_state' and 'radar_state'. Based on this information, it sends the 'control_actions' to Node 1.

The Arduinos are set to publish a new message with a frequency of 25 hz (40ms interval).


# Setting up your ROS environment
After installing ROS, start by [setting up your own workspace.](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) The name of the workspace attached is 'tu_ws'.

There are only a few components needed to get the car running. These are:
- custom messages. The vehicle_state_msg and radar_state_msg messages are simple custom messages that tell us what kind of data we can expect on the vehicle_state and radar_state topics.
- launch file: A launch file makes it easy to start multiple processes in one go. We use it to start the two USB connections to the Arduinos.
- script: A script is a function or program we can run on a ROS node. In our case, *controller.py* is the script were we keep our controller.

The above is all captured in a 'package', called *car*.




# TOPICS
vehicle_state
radar_state
control_actions
# MESSAGES

## vehice_state_msg
- float32 u (m/s)
- float32 v (m/s)
- float32 r (rad/s)
- float32 a_x (m/s^2)
- float32 a_y (m/s^2)		
- float32 omega_fl (rad/s)
- float32 omega_fr (rad/s)
- float32 omega_rl (rad/s)
- float32 omega_rr (rad/s)
		
## radar_state_msg
- float32 radar1 (left)(cm)
- float32 radar2
- float32 radar3
- float32 radar4
- float32 radar5
- float32 radar6
- float32 radar7
- float32 radar8 (right)


## control_actions_msg
- float32 steering_action ([-1,1])
- float32 throttle_action ([-1,1])
