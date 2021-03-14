
# ROS 
Arduino publish frequency 30 hz (33.333 ms interval)

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
