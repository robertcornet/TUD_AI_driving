#!/usr/bin/env python
import rospy
import message_filters
import math
from std_msgs.msg import Float32
from car.msg import vehicle_state_msg 
from car.msg import radar_state_msg 
from car.msg import control_actions_msg
import tensorflow as tf 
import numpy as np 

import time
import csv

#create logfile and header
log_dir = '/home/odroid/tu_ws/logs/controller_log_'+time.asctime()+'.csv'

with open(log_dir, "a") as fp:
    writer = csv.writer(fp, delimiter=",")
    writer.writerow(["time", "u", "v","r","ax","ay","fl","fr","rl","rr","r1","r2","r3","r4","r5","r6","r7","r8","omega_avg"]) 

time_offset = time.time()



###tf.reset_default_graph()
imported_graph = tf.train.import_meta_graph('/home/odroid/tu_ws/src/car/scripts/models/car_v1/SACv82/0/model.ckpt.meta') #import tensorflow graph
s_dim = 9 #initialize state vector
state_vector_init = np.zeros(s_dim)
state_vector = state_vector_init[np.newaxis, :]
S = tf.placeholder(tf.float32, [None, s_dim], 's')  #initialize tensorflow placeholder
sess=tf.Session() #restore tensorflow session
imported_graph.restore(sess, '/home/odroid/tu_ws/src/car/scripts/models/car_v1/SACv82/0/model.ckpt')



def pwm(value, leftMin, leftMax, rightMin, rightMax): #interpolating function to map actions to pwm values    
    leftSpan = leftMax - leftMin # Figure out how 'wide' each range is
    rightSpan = rightMax - rightMin    
    valueScaled = float(value - leftMin) / float(leftSpan)# Convert the left range into a 0-1 range (float)    
    return rightMin + (valueScaled * rightSpan) # Convert the 0-1 range into a value in the right range.



def choose_action(vehicle, radar):

    ################# WRITING TO LOG FILE #####################        
    global time_offset    
    global log_dir
    omega_avg = 0.25*(vehicle.omega_fl+vehicle.omega_fr+vehicle.omega_rl+vehicle.omega_rr)

    data_to_write=[(time.time()-time_offset,vehicle.u, vehicle.v, vehicle.r, vehicle.a_x, vehicle.a_y, vehicle.omega_fl, vehicle.omega_fr, vehicle.omega_rl, vehicle.omega_rr, radar.radar1, radar.radar2, radar.radar3, radar.radar4, radar.radar5, radar.radar6, radar.radar7, radar.radar8, omega_avg)]

    with open(log_dir, "a") as fp:
        writer = csv.writer(fp, delimiter=",")
        writer.writerows(data_to_write)
    ################# WRITING TO LOG FILE #####################

    state_vector[0,0] = 0
    state_vector[0,1] = 0  
    state_vector[0,2] = vehicle.r
    state_vector[0,3] = vehicle.a_x
    state_vector[0,4] = vehicle.a_y
    state_vector[0,5] = vehicle.omega_fl
    state_vector[0,6] = vehicle.omega_fr
    state_vector[0,7] = vehicle.omega_rl
    state_vector[0,8] = vehicle.omega_rr

  
 
    actor = sess.run(['Actor/tanh_1/forward/Tanh:0'], feed_dict={'s:0' :state_vector})[0] #inference
    pwm_steering = pwm(actor[0,0],-1,1,1345,1855) 
    pwm_throttle = pwm((actor[0,1]),-1,1,2000,1000)            
    pwm_throttle = max(pwm_throttle,1350) #limit throttle 
    
    
    #publish control actions message
    control_actions_to_publish = control_actions_msg()
    control_actions_to_publish.steering_action = pwm_steering
    control_actions_to_publish.throttle_action = pwm_throttle    
    pub = rospy.Publisher('control_actions', control_actions_msg, queue_size = 10)
    pub.publish(control_actions_to_publish)  #publish 
   
    
def controller_main():
    #initialize node 
    rospy.init_node('controller', anonymous=True) #create node called 'controller'
    sub1 = message_filters.Subscriber("vehicle_state", vehicle_state_msg) #subscribe to first topic    
    sub2 = message_filters.Subscriber("radar_state", radar_state_msg) #subscribe to second topic
        
    #merge, synchronize, and call choose_action
    ts = message_filters.ApproximateTimeSynchronizer([sub1, sub2], queue_size=10, slop=0.1, allow_headerless=True)
    ts.registerCallback(choose_action)
    rospy.spin()

if __name__ == '__main__':
    try:
        controller_main()
    except rospy.ROSInterruptException:
        pass




