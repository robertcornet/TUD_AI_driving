import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import math
from math import sqrt, tan, sin, cos 
import gym
from gym import spaces, logger
from gym.utils import seeding
import numpy as np
import csv
import time

import random
from scipy.spatial import ConvexHull, convex_hull_plot_2d
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from scipy import interpolate
from scipy.interpolate import splprep, splev

import pyglet
from gym.envs.classic_control import rendering

class Car_Env(gym.Env):
    
    """
    Description    
    """

    def __init__(self):

        #Driveline Setting
        #self.POWER_SPLIT = np.array([0,0,0.5,0.5]) #fl,fr,rl,rr #rwd        
        self.POWER_SPLIT = np.array([0.25,0.25,0.25,0.25]) #fl,fr,rl,rr #4wd

        #Timing     
        self.AGENTdt = 0.05     #timestep for the agent/simulator   [s]
        self.BODYdt = 0.001     #timestep for updating body motion  [s]
        self.TIREdt = 0.001     #timestep for updating tire motion  [s]
        self.FRAMESKIP_BODY = int(self.AGENTdt/self.BODYdt)
        self.FRAMESKIP_TIRE = int(self.BODYdt/self.TIREdt)
                       
        #Unit conversions
        self.RAD2DEG = 180/np.pi 
        self.DEG2RAD = np.pi/180 
        self.RADS2RPM = 9.5492965964254
        self.RPM2RADS = 0.104719755

        #Vehicle parameters
        self.g = 9.81                       # [m / s ^ 2] Gravity
        self.l_f = 0.1285                   # [m] length CoG to front axle
        self.l_r = 0.1285                   # [m] length CoG to front axle
        self.B = 0.162                      # [m]Wheelbase (width)
        self.m = 1.875                      # [kg] mass
        self.h_s = 0.040                    # [m] height cog
        self.h_f = 0.022                    # [m] height front roll center
        self.h_r = 0.023                    # [m] height rear roll center
        self.I_z = 0.0149                   # [kg * m ^ 2] Rotational inertia 2100
        self.C_kappa = 105                  # [N / -] Longitudinal tire stiffness 105000 
        self.C_a_f = self.m*9.81*0.5*0.5*0.165*57.29578    # 37.09672571880001 [N / rad] 47000
        self.C_a_r = self.m*9.81*0.5*0.5*0.165*57.29578    # 37.09672571880001 [N / rad]               
        self.e_r = 0.35                     # [-] friction reduction coefficient (tire modelling)
        self.r_w = 0.032                    # [m] Wheel radius
        self.J_w = 3*(0.032*self.r_w**2)    # [kg * m ^ 2] wheel inertia #wheel weight = 32 gr, I = mr^2 1
        self.L = self.l_f + self.l_r        # [m]
        
        self.Fz_long = self.m * (self.h_s / (2 * self.L))                     #  [N * s ^ 2 / m]
        self.Fz_lat_f = self.m * (self.l_r / self.L) * (self.h_f / self.B)    #  [N * s ^ 2 / m]
        self.Fz_lat_r = self.m * (self.l_f / self.L) * (self.h_r / self.B)    #  [N * s ^ 2 / m]  

        self.m_f = (self.m*self.l_r)/self.L     #front and rear mass for marginal velocities 
        self.m_r = (self.m*self.l_f)/self.L     #front and rear mass for marginal velocities    
        
        #Tire Modelling
        self.eta = 1.1
        self.tau_m = 0.5*self.TIREdt
        self.ux_m = np.array([0,0,0,0],dtype=np.float64) #marginal velocities 
        self.ux_m[0] = self.tau_m*self.C_kappa*((self.r_w**2/self.J_w)+(1/(0.5*self.m_f)))
        self.ux_m[1] = self.tau_m*self.C_kappa*((self.r_w**2/self.J_w)+(1/(0.5*self.m_f)))
        self.ux_m[2] = self.tau_m*self.C_kappa*((self.r_w**2/self.J_w)+(1/(0.5*self.m_r)))
        self.ux_m[3] = self.tau_m*self.C_kappa*((self.r_w**2/self.J_w)+(1/(0.5*self.m_r)))
        self.uy_m = np.array([0,0,0,0],dtype=np.float64) #marginal velocities 
        self.uy_m[0] = self.tau_m*(self.C_a_f/(0.5*self.m_f))
        self.uy_m[1] = self.tau_m*(self.C_a_f/(0.5*self.m_f))
        self.uy_m[2] = self.tau_m*(self.C_a_r/(0.5*self.m_r))
        self.uy_m[3] = self.tau_m*(self.C_a_r/(0.5*self.m_r))      

        #Track Settings
        self.ENV_X_SIZE = 20#Max length of the env [m]
        self.ENV_Y_SIZE = 20 #Max width of the env [m] # lower left corner of environment is (0,0)
        self.MIN_DIST_BETWEEN_COORDINATES = 1 #[m] min spacing between any two grid_coordinates in the environment
        self.NUMBER_RANDOM_COORDINATES = 30 #how many main point to generate track
        self.TRACK_DIFFICULTY = 0.3 #[0-1] Determines the displacement of the inbetween points
        self.TRACK_RESOLUTION = 50 #number of elements of interplolated track #number of track elements is crucial for how much neighbouring elements need to be checked.
        self.NUMBER_X_COORDINATES = int(self.ENV_X_SIZE/self.MIN_DIST_BETWEEN_COORDINATES) # print('number of gridslots in x direction',NUMBER_X_COORDINATES)
        self.NUMBER_Y_COORDINATES = int(self.ENV_Y_SIZE/self.MIN_DIST_BETWEEN_COORDINATES) # print('number of gridslots in y direction',NUMBER_Y_COORDINATES)
        self.MAX_RANDOM_COORDINATES = int(self.NUMBER_X_COORDINATES*self.NUMBER_Y_COORDINATES) #max available grid slots
        self.NUMBER_RANDOM_COORDINATES = min(self.NUMBER_RANDOM_COORDINATES,self.MAX_RANDOM_COORDINATES) #how many points/coordinates we pick. Catch error in case rndm > max        
        self.MAX_ANGLE = 100 #minimum angle between following points
        
        #LIDAR SETTINGS
        self.SENSOR_RANGE = 4 #SENSOR_RANGE IN METERS
        self.SENSOR_ARRAY = self.DEG2RAD*np.array([0,45,90,135,180,225,270,315]) #Directions of the sensors, counterclockwise
        
        self.SENSOR_FOV = self.DEG2RAD*2*22.5  #estimate of the sensors FOV, used for car following
        # self.SENSOR_ARRAY = self.DEG2RAD*np.array([-24,-18,-12,-6,6,12,18,24])
        # self.SENSOR_ARRAY = self.DEG2RAD*np.array([-10.5,-7.5,-4.5,-1.5,1.5,4.5,7.5,10.5])
        
        # self.RADAR_BUFFER = 5  #amount of track indices checked for radar intersections      
        # self.ind_fw = np.zeros(self.RADAR_BUFFER+1)
        # self.ind_bw = np.zeros(self.RADAR_BUFFER+1)
        
        #Actuators
        self.BATTERY_VOLTAGE = 8.4
        self.MOTOR_KV = 2775 #KV, RPM per volt
        self.MOTOR_TV = 0.0065 #Nm torque per volt         
        
        self.FINAL_DRIVE_RATIO = 8.2727  #final drive ratio
        self.SERVO_SPEED = 0.08 #sec/60deg
        self.SERVO_RATE_LIMIT = (self.AGENTdt/self.SERVO_SPEED)*60*self.DEG2RAD #max change in rad between agent steps (stepdt)
        self.MAX_STEERING_ANGLE = 16.5*self.DEG2RAD  #16.5*self.DEG2RAD #29
        self.MAX_SPEED = (self.BATTERY_VOLTAGE*self.MOTOR_KV/self.FINAL_DRIVE_RATIO)*2*self.r_w*np.pi/60 #top speed in m/s
        

        self.episode = 0
        self.difficulty = 0.2
        self.viewer = None
      
        
        ###############################################################        
        #action and observation spaces
        high_action = np.array([1, 1])

        u_range = 20 #planar vehicle motion range
        v_range = 20
        r_range = 20        
        a_range = 20 #acceleration range
        omega_range = 500 #wheel speed 
        l_range = 500 #radar array observation range

        
        high_state = np.array([
            u_range,
            v_range,
            r_range,
            a_range,
            a_range,
            omega_range,
            omega_range,
            omega_range,
            omega_range])

            # l_range,
            # l_range,
            # l_range,
            # l_range,
            # l_range,
            # l_range,
            # l_range,
            # l_range])      

        self.action_space = spaces.Box(low=-high_action, high=high_action, dtype=np.float32)
        self.observation_space = spaces.Box(-high_state, high_state, dtype=np.float32)

        self.seed()

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]



    def step(self, action):
        
        self.time += self.AGENTdt  
       
        #Input
        self.servo_angle = float(action[0])*self.MAX_STEERING_ANGLE
        self.throttle = float(action[1])        
             
        # run_leading_car(self) #run leading car

        motor_voltage = self.BATTERY_VOLTAGE * ESC(self)  #Simulate Electronic Speed Control  
        run_body_EOM(self,self.servo_angle,motor_voltage,self.omega_fl,self.omega_fr,self.omega_rl,self.omega_rr) #run vehicle simulation
        
        self.radar = 4*np.ones(8)
        # self.radar = get_radar_state_zones(self)  #radar distances in cm
        # self.radar = lidar_get_state(self)
        
        # save_radar_history(self)
        # self.radar_prev = read_radar_history(self)

        
        self.state=np.array([
        0,
        0,
        self.r,  
        self.a_x,
        self.a_y,
        self.omega_fl,
        self.omega_fr,
        self.omega_rl,
        self.omega_rr]) 
        # ,
        # self.radar[0],
        # self.radar[1],
        # self.radar[2],
        # self.radar[3],
        # self.radar[4],
        # self.radar[5],
        # self.radar[6],
        # self.radar[7]])

    
        
   
        #VEHICLE STABILITY LIMITS
        # rmax = r_max(self,self.u)
        # vmax = v_max(self,self.u,self.r)




        V = np.sqrt(self.u**2+self.v**2) #[0-inf]
        v_reward = (V/self.MAX_SPEED) #[0-1]

        if self.u > 0:
            v_reward = v_reward**0.5 #[0-1]
        else:
            v_reward = -10*(v_reward**0.5)
       


        # u_reward = (self.u/self.MAX_SPEED)*self.u/abs(self.u)
        radius = V/self.r
        radius_target = 1

        radius_error = abs(radius-radius_target)
        r_cost = -1/(radius_error+1)+1

        # #r_reward: push vehicle outside
        r_reward = radius/abs(radius)*(-1/(abs(radius)+1)+1)
       

        # cost = v_reward+r_reward+0.1*self.r 
        cost = v_reward-r_cost
        
        # print("Radius: ",radius,"V: ", V,"r: ", self.r)
        

        # End episode when wall reached
        done = False
        if self.CAR_INSIDE == False:
            done = True
        done = bool(done)     





        l_rewards = 0.

        
        rmax = abs((self.g * self.mu)/self.u)        

        asat = 0.2579054
        vmax = abs(self.u*asat + self.l_r*self.r)
        
        v_violation = 0
        r_violation = 0

        if abs(self.v) > vmax:
            v_violation = abs(self.v)-vmax

        if abs(self.r) > rmax:
            r_violation = abs(self.r)-rmax

        l_rewards = v_violation**2 + r_violation**2

        violation = 1 if l_rewards > 0 else 0

        return self.state, cost, done, dict(l_rewards=l_rewards, violation_of_constraint=violation)
 




    def reset(self):

        self.time = 0
        self.episode += 1
        
        # if self.difficulty <1: #increase difficulty every episode for incremental learning
        #     self.difficulty+=1/8000        
        # if (self.episode%100 == 0):
        #     print("Episode ", self.episode, ", Difficulty ", self.difficulty)
               
        self.servo_angle = 0 
        self.throttle = 0
        self.BRAKING = 0

        #create randomness in the vehicle and tires
        self.TIRE_FLUCTUATION = np.random.normal(1, 0, 4)
        self.mu = 0.20#np.random.uniform(low=0.98, high=1) 

        self.BATTERY_VOLTAGE =8.2#np.random.uniform(low=8.1, high=8.35) 
        self.MAX_SPEED = (self.BATTERY_VOLTAGE*self.MOTOR_KV/self.FINAL_DRIVE_RATIO)*2*np.pi*self.r_w/60 #top speed in m/s

        ##########################################################################################
        # TRACK
        ##########################################################################################

        # TRACK_VALID = False
        # self.TRACK_WIDTH = np.random.uniform(low=1.2, high=2.5) #half-width of track
        # while TRACK_VALID == False:
        #     self.track_center,self.track_left,self.track_right = track_generator(self)  #generate a track            
        #     print("left",track_check_valid(self.track_left), "right", track_check_valid(self.track_right))
        #     if track_check_valid(self.track_left) and track_check_valid(self.track_right):
        #         break
        # # self.dist_between_points = np.zeros((len(self.track_center),1)) #find track length
        # # for i in range((len(self.track_center)-1)):
        # #     self.dist_between_points[i+1] = np.sqrt((self.x_center[i+1]-self.x_center[i])**2+(self.y_center[i+1]-self.y_center[i])**2)        
        # # self.TRACK_LENGTH = np.cumsum(self.dist_between_points) # print('Total track length is', self.TRACK_LENGTH[-1])
        
        # # #initialize vehicle at beginning of track
        # # self.X = self.track_center[0,0]      
        # # self.Y = self.track_center[0,1]#self.np_random.uniform(low=-0.1, high=0.1)
        # # del_y = self.y_center[1]-self.y_center[0]
        # # del_x = self.x_center[1]-self.x_center[0]
        # # self.YAW = np.arctan2(del_y,del_x) 

        ##########################################################################################
        # ROOM
        ##########################################################################################

        self.room_length = 8#np.random.uniform(low=2.5, high=8)
        self.room_width = 6#np.random.uniform(low=2.5, high=8)
        self.track_center,self.track_left,self.track_right = gen_room(self)  #generate a room 
        
        self.EDGE_BUFFER = 0.2 #how far to stay from the walls before episode is terminated [m]
        self.START_BUFFER = 1 #how far away from the walls to start [m]
        # make sure to not initialize the vehicle too close facing the walls
        # self.X = np.random.uniform(low=0+self.START_BUFFER, high=self.room_length-self.START_BUFFER)
        # self.Y = np.random.uniform(low=0+self.START_BUFFER, high=self.room_width-self.START_BUFFER)
        # self.YAW = np.random.uniform(low=0, high=np.pi)

        self.CAR_INSIDE = True
        self.X = np.random.uniform(low=1, high=3) 
        self.Y = np.random.uniform(low=1, high=5) 
        self.YAW = 0
       
        # self.x_target = self.xmax/2 #center x
        # self.y_target = self.ymax/2 #center y

        #initialize vehicle 
        # self.X = 0
        # self.Y = 0
        # self.YAW = 0

        self.u = 0
        self.v = 0
        self.r = 0
        self.a_x = 0
        self.a_y = 0

        self.omega_fl = self.u/self.r_w #initial rotational velocity in rad/s
        self.omega_fr = self.u/self.r_w
        self.omega_rl = self.u/self.r_w
        self.omega_rr = self.u/self.r_w      

       
        # initialize_leading_car(self)
        # run_leading_car(self)

        self.radar = 4*np.ones(8)
        # self.radar = get_radar_state_zones(self)  #radar distances in cm
        # self.radar = lidar_get_state(self)
        # self.t1=self.t2=self.t3=self.t4=self.t5=self.radar #initialize radar history
        # self.radar_prev = read_radar_history(self)

        self.state=np.array([
        0,
        0,
        self.r,  
        self.a_x,
        self.a_y,
        self.omega_fl,
        self.omega_fr,
        self.omega_rl,
        self.omega_rr]) 
        # ,
        # self.radar[0],
        # self.radar[1],
        # self.radar[2],
        # self.radar[3],
        # self.radar[4],
        # self.radar[5],
        # self.radar[6],
        # self.radar[7]])

                
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None


        return self.state

    



    def render(self, mode='human'):
        screen_width = 800
        screen_height = 800

        if self.viewer is None:
            self.viewer = rendering.Viewer(screen_width, screen_height)
            self.viewer.set_bounds(-10, 10,-10,10)

            ## DRAW THE TRACK
            for i in range(len(self.track_left)-1):
                self.draw_track_left = rendering.Line((self.track_left[i,0],self.track_left[i,1]),(self.track_left[i+1,0],self.track_left[i+1,1]))
                self.draw_track_left.set_color(0, 0, 1)
                self.viewer.add_geom(self.draw_track_left)
                self.draw_track_right = rendering.Line((self.track_right[i,0],self.track_right[i,1]),(self.track_right[i+1,0],self.track_right[i+1,1]))
                self.draw_track_right.set_color(0, 0, 1)
                self.viewer.add_geom(self.draw_track_right)
                self.draw_track_center = rendering.Line((self.track_center[i,0],self.track_center[i,1]),(self.track_center[i+1,0],self.track_center[i+1,1]))
                self.draw_track_center.set_color(1, 0, 0)
                self.viewer.add_geom(self.draw_track_center)
                        
            #background throttle
            l,r,t,b = -5,-4,5,-5
            throttle_bar_background = rendering.FilledPolygon([(l, b), (l, t), (r, t), (r, b)])
            throttle_bar_background.set_color(0.8, 0.8, 0.8)
            self.viewer.add_geom(throttle_bar_background)
            #throttle bar
            l,r,t,b = -5,-4,5,0
            throttle_bar = rendering.FilledPolygon([(l, b), (l, t), (r, t), (r, b)])
            throttle_bar.set_color(0, 0.4470, 0.7410)
            self.throttle_bar_trans = rendering.Transform()
            throttle_bar.add_attr(self.throttle_bar_trans)
            self.viewer.add_geom(throttle_bar)                        
            #background steering
            l,r,t,b = -4,-3,5,-5
            steering_bar_background = rendering.FilledPolygon([(l, b), (l, t), (r, t), (r, b)])
            steering_bar_background.set_color(0.8, 0.8, 0.8)
            self.viewer.add_geom(steering_bar_background)
            #steering bar
            l,r,t,b = -4,-3,5,0
            view_steering = rendering.FilledPolygon([(l, b), (l, t), (r, t), (r, b)])
            view_steering.set_color(0.85, 0.325, 0.0980)
            self.steering_trans = rendering.Transform()
            view_steering.add_attr(self.steering_trans)
            self.viewer.add_geom(view_steering)
            #speed bar
            l,r,t,b = -3,-2,5,0
            view_speed_bar = rendering.FilledPolygon([(l, b), (l, t), (r, t), (r, b)])
            view_speed_bar.set_color(0.9290, 0.6940, 0.1250)
            self.speed_trans = rendering.Transform()
            view_speed_bar.add_attr(self.speed_trans)
            self.viewer.add_geom(view_speed_bar)

            #add car
            l,r,t,b = -0.15,+0.15,+0.1,-0.1
            car = rendering.FilledPolygon([(l, b), (l, t), (r, t), (r, b)])
            self.cartrans = rendering.Transform()
            car.add_attr(self.cartrans)
            self.viewer.add_geom(car)        

            #add leading car
            # l,r,t,b = -0.2,+0.2,+0.1,-0.1
            # lead_car = rendering.FilledPolygon([(l, b), (l, t), (r, t), (r, b)])
            # self.lead_cartrans = rendering.Transform()
            # lead_car.add_attr(self.lead_cartrans)
            # self.viewer.add_geom(lead_car)       
            
            #add radar line elements
            r1 = rendering.Line((0,0),(1,0))
            r1.set_color(0,0,1)
            self.r1trans = rendering.Transform()
            r1.add_attr(self.r1trans)
            self.viewer.add_geom(r1)
            r2 = rendering.Line((0,0),(1,0))
            self.r2trans = rendering.Transform()
            r2.add_attr(self.r2trans)
            self.viewer.add_geom(r2)
            r3 = rendering.Line((0,0),(1,0))
            self.r3trans = rendering.Transform()
            r3.add_attr(self.r3trans)
            self.viewer.add_geom(r3)
            r4 = rendering.Line((0,0),(1,0))
            self.r4trans = rendering.Transform()
            r4.add_attr(self.r4trans)
            self.viewer.add_geom(r4)
            r5 = rendering.Line((0,0),(1,0))
            self.r5trans = rendering.Transform()
            r5.add_attr(self.r5trans)
            self.viewer.add_geom(r5)
            r6 = rendering.Line((0,0),(1,0))
            self.r6trans = rendering.Transform()
            r6.add_attr(self.r6trans)
            self.viewer.add_geom(r6)
            r7 = rendering.Line((0,0),(1,0))
            self.r7trans = rendering.Transform()
            r7.add_attr(self.r7trans)
            self.viewer.add_geom(r7)
            r8 = rendering.Line((0,0),(1,0))
            self.r8trans = rendering.Transform()
            r8.add_attr(self.r8trans)
            self.viewer.add_geom(r8)
        

        #following cam:
        self.viewer.set_bounds(self.X-6, self.X+6, self.Y-6,self.Y+6)        
        #inputs
        self.throttle_bar_trans.set_scale(1,self.throttle)
        self.steering_trans.set_scale(1,self.servo_angle/self.MAX_STEERING_ANGLE)  
        #speed
        self.speed_trans.set_scale(1,sqrt(self.u**2+self.v**2)/self.MAX_SPEED)          
        
        #car
        self.cartrans.set_translation(self.X,self.Y)
        self.cartrans.set_rotation(self.YAW)
        #leading car
        # self.lead_cartrans.set_translation(self.X_lead,self.Y_lead)
        # self.lead_cartrans.set_rotation(self.YAW_lead)

        # radar array
        self.r1trans.set_scale(self.radar[0]/100,0)
        self.r1trans.set_translation(self.X,self.Y)
        self.r1trans.set_rotation(self.YAW+self.SENSOR_ARRAY[0])        
        self.r2trans.set_scale(self.radar[1]/100,0)
        self.r2trans.set_translation(self.X,self.Y)
        self.r2trans.set_rotation(self.YAW+self.SENSOR_ARRAY[1])          
        self.r3trans.set_scale(self.radar[2]/100,0)
        self.r3trans.set_translation(self.X,self.Y)
        self.r3trans.set_rotation(self.YAW+self.SENSOR_ARRAY[2])    
        self.r4trans.set_scale(self.radar[3]/100,0)
        self.r4trans.set_translation(self.X,self.Y)
        self.r4trans.set_rotation(self.YAW+self.SENSOR_ARRAY[3])    
        self.r5trans.set_scale(self.radar[4]/100,0)
        self.r5trans.set_translation(self.X,self.Y)
        self.r5trans.set_rotation(self.YAW+self.SENSOR_ARRAY[4])    
        self.r6trans.set_scale(self.radar[5]/100,0)
        self.r6trans.set_translation(self.X,self.Y)
        self.r6trans.set_rotation(self.YAW+self.SENSOR_ARRAY[5])    
        self.r7trans.set_scale(self.radar[6]/100,0)
        self.r7trans.set_translation(self.X,self.Y)
        self.r7trans.set_rotation(self.YAW+self.SENSOR_ARRAY[6])    
        self.r8trans.set_scale(self.radar[7]/100,0)
        self.r8trans.set_translation(self.X,self.Y)
        self.r8trans.set_rotation(self.YAW+self.SENSOR_ARRAY[7])    

        return self.viewer.render(return_rgb_array=mode == 'rgb_array')
        

    def close(self):
        if self.viewer:
            self.viewer.close()
            self.viewer = None


def get_radar_state_zones(self):    
    radar = np.zeros(8)

    yaw_follow = self.YAW % (2 * np.pi)        
    angle_follow_to_lead = math.atan2((self.Y_lead-self.Y),(self.X_lead-self.X))
    #convert -pi > 0 to pi > 2pi
    if (angle_follow_to_lead < 0): #if tan2 in bottom half
        angle_follow_to_lead += 2*math.pi
    angle_follow_centric = (angle_follow_to_lead-yaw_follow) % (2 * np.pi)

    d = np.sqrt((self.X-self.X_lead)**2+(self.Y-self.Y_lead)**2) #distance to leading car

    fov = abs(math.atan((0.125)/d)) #
    fov_high = (angle_follow_centric+fov)% (2 * np.pi)
    fov_low = (angle_follow_centric-fov)% (2 * np.pi)

    #rotate everything by pi for easier comparison
    fov_high = (fov_high+np.pi)% (2 * np.pi)    
    fov_low = (fov_low+np.pi)% (2 * np.pi)

    for i in range(len(self.SENSOR_ARRAY)):
        sensor_high = (self.SENSOR_ARRAY[i] + 0.5*self.SENSOR_FOV+np.pi) % (2 * np.pi)
        sensor_low = (self.SENSOR_ARRAY[i] - 0.5*self.SENSOR_FOV+np.pi) % (2 * np.pi)

        if fov_high<sensor_low or fov_low > sensor_high:
            radar[i] = 100*self.SENSOR_RANGE
        else:
            radar[i] = min(d*100,400)

    return radar

def save_radar_history(self):
    self.t5 = self.t4
    self.t4 = self.t3
    self.t3 = self.t2
    self.t2 = self.t1
    self.t1 = self.radar

    return 

def read_radar_history(self):
    radar_prev = self.t5

    return radar_prev

def initialize_leading_car(self):
    self.X_lead = self.X + np.random.uniform(low=0.2, high=1.5)
    self.Y_lead = self.Y + np.random.uniform(low=-0.02, high=0.02)
    self.YAW_lead = self.YAW + np.random.uniform(low=-0.01, high=0.01)

    self.n_action_steps = np.random.randint(10,100)
    self.n_action_steps_executed = 0 
    self.u_target_lead = np.random.uniform(low=0, high=self.difficulty*self.MAX_SPEED)
    self.u_lead = 0
    self.r_lead = 0
       
    return

def run_leading_car(self):
    if self.n_action_steps_executed < self.n_action_steps: #execute planned action
        self.n_action_steps_executed +=1
    else: #pick new action and timeline
        self.n_action_steps = np.random.randint(10,100)
        self.n_action_steps_executed = 1

        self.u_target_lead = np.random.uniform(low=0, high=self.difficulty*0.60*self.MAX_SPEED)
        self.r_lead = self.difficulty*np.random.uniform(low=-1, high=1)

    u_dot = self.u_target_lead-self.u_lead
    u_dot = self.difficulty*np.clip(u_dot,-1,1-1*(self.u_lead/self.MAX_SPEED)) #limit the acceleration of the leading car
    self.u_lead += u_dot*self.AGENTdt
    # print("leading car speed: ", 3.6*self.u_lead)
    self.YAW_lead += self.r_lead*(self.u_lead/5)*self.AGENTdt #limits turning circle to be > 2m
    X_dot_lead = math.cos(self.YAW_lead)*self.u_lead
    Y_dot_lead = math.sin(self.YAW_lead)*self.u_lead
    self.X_lead += X_dot_lead * self.AGENTdt
    self.Y_lead += Y_dot_lead * self.AGENTdt
   
    return
     
def run_body_EOM(self,servo_angle,motor_voltage,omega_fl,omega_fr,omega_rl,omega_rr):

    delta_i = np.array([servo_angle,servo_angle,0,0])     
    alpha_i = np.zeros(4)
    Fz_i = np.zeros(4)
    F_x_i = np.zeros(4)
    F_y_i = np.zeros(4)      
    
    for i in range(self.FRAMESKIP_BODY): #runs 1000/10 = 100 times
        # Normal Loads, Load Transfer   
        Fz_i[0] = ((self.m*self.g)/(2*self.L))*self.l_r - self.a_x * self.Fz_long - self.a_y * self.Fz_lat_f
        Fz_i[1] = ((self.m*self.g)/(2*self.L))*self.l_r - self.a_x * self.Fz_long + self.a_y * self.Fz_lat_f
        Fz_i[2] = ((self.m*self.g)/(2*self.L))*self.l_f + self.a_x * self.Fz_long - self.a_y * self.Fz_lat_r
        Fz_i[3] = ((self.m*self.g)/(2*self.L))*self.l_f + self.a_x * self.Fz_long + self.a_y * self.Fz_lat_r   
        #Corner Velocities
        u_fl = u_rl = self.u - self.r*(0.5*self.B)    
        u_fr = u_rr = self.u + self.r*(0.5*self.B)
        v_fl = v_fr = self.v+self.l_f*self.r
        v_rl = v_rr = self.v-self.l_r*self.r  
        #slip angles
        alpha_i[0] = delta_i[0]-math.atan((v_fl)/max(abs(u_fl),self.eta*self.uy_m[0]))
        alpha_i[1] = delta_i[1]-math.atan((v_fr)/max(abs(u_fr),self.eta*self.uy_m[1]))
        alpha_i[2] = -math.atan((v_rl)/max(abs(u_rl),self.eta*self.uy_m[2]))
        alpha_i[3] = -math.atan((v_rr)/max(abs(u_rr),self.eta*self.uy_m[3]))
        alpha_i = np.clip(alpha_i,-0.25*math.pi,0.25*math.pi)    

        F_xw_i, F_yw_i,omega_fl,omega_fr,omega_rl,omega_rr = tire_and_wheel_dynamics(self, alpha_i,u_fl,u_fr,u_rl,u_rr,Fz_i,motor_voltage,omega_fl,omega_fr,omega_rl,omega_rr)
        #update omegas
        self.omega_fl = omega_fl
        self.omega_fr = omega_fr
        self.omega_rl = omega_rl
        self.omega_rr = omega_rr

        # Force Conversion from wheels to body 
        F_x_i=F_xw_i*np.cos(delta_i) - F_yw_i * np.sin(delta_i)* self.TIRE_FLUCTUATION
        F_y_i=F_xw_i*np.sin(delta_i) + F_yw_i * np.cos(delta_i)* self.TIRE_FLUCTUATION
        T = (F_y_i[0]+F_y_i[1])*self.l_f-(F_y_i[2]+F_y_i[3])*self.l_r+(-F_x_i[0]+F_x_i[1]-F_x_i[2]+F_x_i[3])*0.5*self.B;

        # Derivatives
        a_x_dot = (sum(F_x_i)/self.m - self.a_x)/self.BODYdt
        a_y_dot = (sum(F_y_i)/self.m - self.a_y)/self.BODYdt
        u_dot = sum(F_x_i)/self.m + (self.v*self.r)
        v_dot = sum(F_y_i)/self.m - (self.u*self.r)
        r_dot = T/self.I_z
        #Vehicle motion
        self.u += u_dot*self.BODYdt 
        self.v += v_dot*self.BODYdt     
        self.r += r_dot*self.BODYdt

        self.a_x += a_x_dot*self.BODYdt
        self.a_y += a_y_dot*self.BODYdt  
        #Vehicle location
        self.YAW += self.r*self.BODYdt
        X_dot = math.cos(self.YAW)*self.u - math.sin(self.YAW)*self.v
        Y_dot = math.sin(self.YAW)*self.u + math.cos(self.YAW)*self.v
        self.X += X_dot * self.BODYdt
        self.Y += Y_dot * self.BODYdt


     
        #################### ROOM, BREAK IF TOO CLOSE TO THE WALLS        
        # if self.X < self.EDGE_BUFFER:
        #     self.CAR_INSIDE = False
        #     # print("hit left")
        #     break
        # elif self.X > (self.room_length-self.EDGE_BUFFER):
        #     self.CAR_INSIDE = False
        #     # print("hit right")
        #     break
        # elif self.Y < self.EDGE_BUFFER:
        #     self.CAR_INSIDE = False
        #     # print("hit bottom")
        #     break
        # elif self.Y > (self.room_width-self.EDGE_BUFFER):
        #     self.CAR_INSIDE = False
        #     # print("hit top")
        #     break
        

def tire_and_wheel_dynamics(self,alpha_i,u_fl,u_fr,u_rl,u_rr,Fz_i,motor_voltage,omega_fl,omega_fr,omega_rl,omega_rr):
    
    f = np.zeros(4)
    Cx = self.C_kappa*np.ones(4)
    Cy = self.C_a_f*np.ones(4) 
    kappa_i = np.zeros(4)

    for i in range(self.FRAMESKIP_TIRE):          
        
        motor_rpm = np.dot(self.POWER_SPLIT,[self.omega_fl,self.omega_fr,self.omega_rl,self.omega_rr])*self.FINAL_DRIVE_RATIO*self.RADS2RPM #determine motor rpm from average driven wheels
        motor_torque = motor(self,motor_voltage,motor_rpm)
        driveline_torque = self.FINAL_DRIVE_RATIO*motor_torque   
        
        if self.BRAKING == 0: 
            T_net_i = self.POWER_SPLIT*driveline_torque
        else: 
            T_net_i = self.POWER_SPLIT*driveline_torque*np.array([0,0,math.copysign(1,omega_rl),math.copysign(1,omega_rr)])
      
        kappa_i[0] = (omega_fl*self.r_w - u_fl)/max(abs(u_fl),self.eta*self.ux_m[0])
        kappa_i[1] = (omega_fr*self.r_w - u_fr)/max(abs(u_fr),self.eta*self.ux_m[1])
        kappa_i[2] = (omega_rl*self.r_w - u_rl)/max(abs(u_rl),self.eta*self.ux_m[2])
        kappa_i[3] = (omega_rr*self.r_w - u_rr)/max(abs(u_rr),self.eta*self.ux_m[3])   
        kappa_i = np.clip(kappa_i,-0.99, 0.99)  
                
        zeta =  self.mu*Fz_i*(1-kappa_i) *(1-self.e_r*np.sqrt(kappa_i**2 + np.tan(alpha_i)**2))/(2*np.sqrt((Cx*kappa_i)**2 + (Cy*np.tan(alpha_i))**2))   
        
        for i  in range (4):
            if zeta[i] < 1:
                f[i] = zeta[i] *(2-zeta[i])
            else:
                f[i] = 1
              
        F_xw_i = Cx * (kappa_i / (1-kappa_i)) * f 
        F_yw_i = Cy * (np.tan(alpha_i) / (1-kappa_i)) * f 

        omega_fl_dot = (T_net_i[0]-F_xw_i[0]*self.r_w)/self.J_w
        omega_fr_dot = (T_net_i[1]-F_xw_i[1]*self.r_w)/self.J_w
        omega_rl_dot = (T_net_i[2]-F_xw_i[2]*self.r_w)/self.J_w
        omega_rr_dot = (T_net_i[3]-F_xw_i[3]*self.r_w)/self.J_w

        omega_fl += omega_fl_dot*self.TIREdt
        omega_fr += omega_fr_dot*self.TIREdt
        omega_rl += omega_rl_dot*self.TIREdt
        omega_rr += omega_rr_dot*self.TIREdt
    
    return F_xw_i, F_yw_i, omega_fl,omega_fr,omega_rl,omega_rr

def servo(self, servo_angle_desired):
    servo_angle_delta = servo_angle_desired - self.servo_angle
    self.servo_angle += np.clip(servo_angle_delta, -self.SERVO_RATE_LIMIT,self.SERVO_RATE_LIMIT)         
    return self.servo_angle

def ESC(self):
    throttle_percentage = self.throttle*100
    if (throttle_percentage >=0) and (throttle_percentage<= 7.6):
        voltage_multiplier = 0
    elif (throttle_percentage >7.6) and (throttle_percentage <= 30):
        voltage_multiplier = -0.0007*throttle_percentage**2+0.0429*throttle_percentage+0.1586
    elif (throttle_percentage > 30) and (throttle_percentage<=60):
        voltage_multiplier = (throttle_percentage-30)*0.006146666+0.8156
    elif throttle_percentage > 60:
        voltage_multiplier = 1
    else: #active braking
        voltage_multiplier = throttle_percentage/100       
    return voltage_multiplier



def motor(self, motor_voltage,motor_rpm):
    if motor_voltage >= 0:
        omega_max = motor_voltage*self.MOTOR_KV
        if motor_rpm <= omega_max:#accelerating  
            torque = motor_voltage*self.MOTOR_TV-(self.MOTOR_TV/self.MOTOR_KV)*motor_rpm
            self.BRAKING = 0
        else: #brake if turning faster than current voltage:                       
            back_emf_voltage = abs(motor_rpm)/self.MOTOR_KV
            torque = -(back_emf_voltage*self.MOTOR_TV-(self.MOTOR_TV/self.MOTOR_KV)*motor_voltage*self.MOTOR_KV)
            self.BRAKING = 1
    else:
        # torque = 1.5*np.clip(motor_voltage*(self.MOTOR_TV/self.MOTOR_KV)*motor_rpm,motor_voltage*self.MOTOR_TV,-motor_voltage*self.MOTOR_TV)
        torque = 1.2*motor_voltage*(self.MOTOR_TV/self.MOTOR_KV)*abs(motor_rpm)
        self.BRAKING = 1

    return torque





#GENERATE ROOM
def gen_room(self):

    left_border = 0
    lower_border = 0
    right_border = self.room_length
    upper_border = self.room_width
    
    bl = [left_border,lower_border]
    br = [right_border,lower_border]
    tl = [left_border,upper_border]
    tr = [right_border,upper_border]

    loop = np.zeros((5,2))
    loop[0,:] = bl
    loop[1,:] = br
    loop[2,:] = tr
    loop[3,:] = tl
    loop[4,:] = bl

    track_center=track_left=track_right = loop
    
    return track_center, track_left, track_right



## GENERATE TRACK
def track_fix_angles(max_angle, points):
    for i in range(len(points)):
        if i > 0:
            prev_point = i - 1
        else:
            prev_point = len(points)-1
        next_point = (i+1) % len(points)
        px = points[i][0] - points[prev_point][0]      
        py = points[i][1] - points[prev_point][1]
        pl = math.sqrt(px*px + py*py)
        px /= pl 
        py /= pl     
        nx = -(points[i][0] - points[next_point][0])
        ny = -(points[i][1] - points[next_point][1])
        nl = math.sqrt(nx*nx + ny*ny)
        nx /= nl
        ny /= nl  
        a = math.atan2(px * ny - py * nx, px * nx + py * ny)
        if (abs(math.degrees(a)) <= max_angle):
            continue
        diff = math.radians(max_angle * math.copysign(1,a)) - a
        c = math.cos(diff)
        s = math.sin(diff)
        new_x = (nx * c - ny * s) * nl
        new_y = (nx * s + ny * c) * nl
        points[next_point][0] = (points[i][0] + new_x) #int(points[i][0] + new_x)
        points[next_point][1] = (points[i][1] + new_y) #int(points[i][1] + new_y)
    return points

def track_offset(coordinates, distance):
    coordinates = iter(coordinates)
    x1, y1 = coordinates.__next__()
    z = distance
    points = []
    for x2, y2 in coordinates:
        # tangential slope approximation
        try:
            slope = (y2 - y1) / (x2 - x1)
            # perpendicular slope
            pslope = -1/slope  # (might be 1/slope depending on direction of travel)
        except ZeroDivisionError:
            continue
        mid_x = (x1 + x2) / 2
        mid_y = (y1 + y2) / 2
        sign = ((pslope > 0) == (x1 > x2)) * 2 - 1
        delta_x = sign * z / ((1 + pslope**2)**0.5)
        delta_y = pslope * delta_x
        points.append((mid_x + delta_x, mid_y + delta_y))
        x1, y1 = x2, y2
    return points

def track_generator(self):

    #picking random coordinates
    grid_numbers = random.sample(range(1,self.MAX_RANDOM_COORDINATES+1),self.NUMBER_RANDOM_COORDINATES) #Each grid slot gets a number, set amount are picked from the list: #range(start,stop), stop is not counted # print('grid numbers',sorted(grid_numbers)) 
    grid_coordinates = np.zeros((self.NUMBER_RANDOM_COORDINATES,2)) #convert the numbers to coordinates:
    for i in range(self.NUMBER_RANDOM_COORDINATES):  
        grid_coordinates[i,0] = int((grid_numbers[i])%self.NUMBER_X_COORDINATES)*self.MIN_DIST_BETWEEN_COORDINATES # print('x_coordinate:',grid_coordinates[i,0])
        grid_coordinates[i,1] = (math.ceil(grid_numbers[i]/self.NUMBER_X_COORDINATES)-1)*self.MIN_DIST_BETWEEN_COORDINATES # print('y_coordinate:',grid_coordinates[i,1],math.ceil(grid_coordinates[i,1]))

    #Get convex hull of outer points
    hull = ConvexHull(grid_coordinates) #find the convex hull around the random coordinates
    hull_loop = np.zeros((len(hull.simplices),2)) #store the loop coordinates in a new vector without the other grid_coordinates #print('Outer loop has ',len(hull.simplices),' grid_coordinates')
    for i in range(len(hull.vertices)):
        hull_loop[i,:] = grid_coordinates[hull.vertices[i],:]
    
    #Put hull coordinates in loop vector
    loop = np.zeros((2*(len(hull.simplices)),2))#New vector that includes midgrid_coordinates:
    for i in range(len(hull.vertices)):
        loop[(i*2),:]=hull_loop[i,:] #put original coordinates in correct place in new vector
    
    #displacing midpoints
    max_displacement = np.zeros((len(loop),1))  #max displacement for each midpoint
    for i in range(len(hull_loop)-1):
        max_displacement[(i*2)+1] = np.random.uniform(low=-1, high=1)*self.TRACK_DIFFICULTY*0.5*np.sqrt((loop[i*2,0]-loop[(i*2)+2,0])**2+(loop[i*2,1]-loop[(i*2)+2,1])**2)
    max_displacement[-1] = np.random.uniform(low=-1, high=1)*self.TRACK_DIFFICULTY*0.5*np.sqrt((loop[0,0]-loop[-1,0])**2+(loop[0,1]-loop[-1,1])**2)
    
    #fill in the inbetween pieces (1) and add a random displacement (2)
    for i in range(len(hull_loop)-1):
        loop[(i*2)+1,:]=(loop[(i*2)+2,:]+loop[(i*2),:])/2 + max_displacement[(i*2)+1]
    #add last one manually
    loop[-1,:]=(loop[0,:]+loop[-2,:])/2 + max_displacement[-1]

    #push points apart
    for i in range(3):
        loop = track_fix_angles(self.MAX_ANGLE, loop)

    # # add first element of loop at the end
    loop = np.vstack((loop, loop[0,:]))

    #making the loop smooth by interpolating a spline
    x = loop[:,0]
    y = loop[:,1]
    tck, u = interpolate.splprep([x, y], s=0, per=True)
    xi, yi = interpolate.splev(np.linspace(0, 1, self.TRACK_RESOLUTION), tck)

    #CREATING LEFT AND RIGHT BOUNDARIES
    loop_i = np.zeros((self.TRACK_RESOLUTION,2))
    loop_i[:,0] = xi
    loop_i[:,1] = yi

    parallel = track_offset(loop_i, 0.5*self.TRACK_WIDTH)
    t2 = [x[0] for x in parallel]
    s2 = [x[1] for x in parallel]

    parallel = track_offset(loop_i, -0.5*self.TRACK_WIDTH)
    t3 = [x[0] for x in parallel]
    s3 = [x[1] for x in parallel]

    # add first element to end of left and right road boundaries
    t2 = np.hstack((t2,t2[0]))
    s2 = np.hstack((s2,s2[0]))
    t3 = np.hstack((t3,t3[0]))
    s3 = np.hstack((s3,s3[0]))

    loop_left = np.zeros((len(t2),2))
    loop_left[:,0] = t2
    loop_left[:,1] = s2

    loop_right = np.zeros((len(t3),2))
    loop_right[:,0] = t3
    loop_right[:,1] = s3

    track_center = loop_i
    track_left = loop_left
    track_right = loop_right

    return track_center, track_left, track_right

def track_check_intersects(s0,s1): #[(x,y),(x,y)]
    dx0 = s0[1][0]-s0[0][0]
    dx1 = s1[1][0]-s1[0][0]
    dy0 = s0[1][1]-s0[0][1]
    dy1 = s1[1][1]-s1[0][1]
    p0 = dy1*(s1[1][0]-s0[0][0]) - dx1*(s1[1][1]-s0[0][1])
    p1 = dy1*(s1[1][0]-s0[1][0]) - dx1*(s1[1][1]-s0[1][1])
    p2 = dy0*(s0[1][0]-s1[0][0]) - dx0*(s0[1][1]-s1[0][1])
    p3 = dy0*(s0[1][0]-s1[1][0]) - dx0*(s0[1][1]-s1[1][1])
    return (p0*p1<=0) & (p2*p3<=0)

def track_check_valid(points):
    valid = True
    x_temp = points[:-1,0] #remove last item because its identical to the 1st
    y_temp = points[:-1,1]
    for i in range(len(x_temp)-1): 
        s0 = [(x_temp[i],y_temp[i]),(x_temp[i+1],y_temp[i+1])] #pick first line element# print('s0:',i,',',i+1)        
        for n in range(i+1, i+(len(x_temp)-2)): #compare line element against n following elements
            b0=((n+1)%len(x_temp))#new counting index
            b1=(b0+1)%len(x_temp)
            s1 = [(x_temp[b0],y_temp[b0]),(x_temp[b1],y_temp[b1])] #pick following line element
            intersect = track_check_intersects(s0,s1) #do line elements intersect?
            if intersect == True:
                valid = False
    return valid




## LIDAR

def lidar_translate(v, angle, dist):
    x, y = v
    x += cos(angle) * dist
    y += sin(angle) * dist
    return x, y

def lidar_magnitude(vector):
    return np.sqrt(np.dot(np.array(vector),np.array(vector)))


def lidar_norm(vector):
    return np.array(vector)/lidar_magnitude(np.array(vector))

# from https://gist.github.com/danieljfarrell/faf7c4cafd683db13cbc
def lidar_lineRayIntersectionDist(origin, direction, dist, point1, point2):
    rayOrigin = np.array(origin, dtype=np.float)
    rayDirection = np.array(lidar_norm(lidar_translate((0, 0), direction, 1)), dtype=np.float)
    point1 = np.array(point1, dtype=np.float)
    point2 = np.array(point2, dtype=np.float)
    v1 = rayOrigin - point1
    v2 = point2 - point1
    v3 = np.array([-rayDirection[1], rayDirection[0]])
    # t11 = np.cross(v2,v1)
    # print('v2',v2,'v1',v1,'cross',t11)
    # print((v2[0]*v1[1])-(v2[1]*v1[0]))
    # t22 = np.dot(v2,v3)
    # t1 = t11/t22
    t1 = ((v2[0]*v1[1])-(v2[1]*v1[0]))/np.dot(v2,v3)
    # t1 = np.cross(v2, v1) / np.dot(v2, v3)   
    t2 = np.dot(v1, v3) / np.dot(v2, v3)
    if t1 < 0.0 or t1 > dist or t2 < 0.0 or t2 > 1.0:
        return dist
    return t1
    
def lidar_rayDist(self,direction): 
    dist = self.SENSOR_RANGE
    d = dist
    for i in range(len(self.track_center)-1):
        # i = int(self.ind_fw[p])
        v0 = self.track_center[i]
        v1 = self.track_center[(i + 1)%len(self.track_center)]
        d = lidar_lineRayIntersectionDist([self.X, self.Y], direction, dist, v0, v1)
        if d < dist:
            break      
    return d

# def lidar_rayDist(self,direction): 
#     dist = 4
#     d = dist
#     for p in range(len(self.ind_fw)):
#         i = int(self.ind_fw[p])
#         v0 = self.track_left[i]
#         v1 = self.track_left[(i + 1)%len(self.track_left)]
#         d = lidar_lineRayIntersectionDist([self.X, self.Y], direction, dist, v0, v1)
#         if d < dist:
#             break        
#         # v0 = self.track_right[i]
#         # v1 = self.track_right[(i + 1)%len(self.track_left)]
#         # d = lidar_lineRayIntersectionDist([self.X, self.Y], direction, dist, v0, v1)
#         # if d < dist:
#         #     break
#         # i = int(self.ind_bw[p])
#         # v0 = self.track_left[i]
#         # v1 = self.track_left[(i + 1)%len(self.track_left)]
#         # d = lidar_lineRayIntersectionDist([self.X, self.Y], direction, dist, v0, v1)
#         # if d < dist:
#         #     break        
#         # v0 = self.track_right[i]
#         # v1 = self.track_right[(i + 1)%len(self.track_left)]
#         # d = lidar_lineRayIntersectionDist([self.X, self.Y], direction, dist, v0, v1)
#         # if d < dist:
#         #     break
#     return d

# def lidar_get_state(self):
#     s = []
#     for i in range(len(self.SENSOR_ARRAY)):
#         direction = i * (2*np.pi/len(self.SENSOR_ARRAY)) + self.YAW
#         s.append(lidar_rayDist(self, direction))
#     return s

def lidar_get_state(self):
    s = []
    for i in range(len(self.SENSOR_ARRAY)):
        direction = self.SENSOR_ARRAY[i] + self.YAW
        s.append(lidar_rayDist(self, direction)*100)
    return s

## VEHICLE STABILITY
def r_max(self,u):
    rmax = abs((self.g * self.mu)/u)
    return rmax

def v_max(self,u,r):   
    asat = 0.2579054
    vmax = abs(u*asat + self.l_r*r)
    return vmax

