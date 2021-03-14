
# ROS 
Arduino publish frequency 30 hz (33.333 ms interval)

# TOPICS
vehicle_state
radar_state
control_actions
# MESSAGES

## vehice_state_msg
float32 u
float32 v
float32 r
float32 a_x
float32 a_y			
float32 omega_fl
float32 omega_fr
float32 omega_rl
float32 omega_rr
		
## radar_state_msg
float32 radar1 (left) (time or distance?)
float32 radar2
float32 radar3
float32 radar4
float32 radar5
float32 radar6
float32 radar7
float32 radar8 (right)


## control_actions_msg
float32 steering_action 
float32 throttle_action 
