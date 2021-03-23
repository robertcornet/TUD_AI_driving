 // ---------------------------------------------------------------- //
// Arduino Ultrasoninc Sensor array for research vehicle
// Arduino IDE < 1.8.4
// Using HC-SR04 Module
// Created for the Department of Cognitive Robotics (CoR) TuDelft
// ---------------------------------------------------------------- //

// The sensors are triggered sequentially, with the assumption that if
// an object is not present within 4 meters, there will be no echo
// returning from a object >4 meters away. Thus, each sensor can be 
// trigerred after the previous has received a response, or the time for 
// objects within 4 meters has expired.

volatile bool sensor_ready = true;
int sensor = 0;
int trigPin = 0;
int echoPin = 0;

int triggerpins[] = {A0, A1, A2, A3, A4, A5, A6, A7};
int echopins[] = {4,5,6,7,8,9,10,11};
unsigned long distances[] = {11,22,33,44,55,66,77,88};
volatile unsigned long durations[] = {1,2,3,4,5,6,7,8};

volatile unsigned long echostarttime = 1000;

unsigned long trigger_time = 10;  // store trigger time

//ROS///////////////////////////////////////////////////////////////////////////////////
  
#include <ros.h>
#include <car/radar_state_msg.h>
ros::NodeHandle  nh;


car::radar_state_msg  radar_state;


ros::Publisher pub_radar("radar_state", &radar_state); //function, topic, msg
//ros::Subscriber<car::control_actions_msg> sub_control_actions("control_actions", controlCallback);


long publisher_timer; // store ROS publish time
//ROS///////////////////////////////////////////////////////////////////////////////////


void setup() {
    //ROS///////////////////////////////////////////////////////////////////////////////////
    nh.initNode();
    nh.advertise(pub_radar);
    //ROS///////////////////////////////////////////////////////////////////////////////////

    for(int i = 0; i < 8; i++){
      pinMode(triggerpins[i], OUTPUT);
    }
    
    Serial.begin(115200); 
}



void loop() {
  
  if (sensor_ready){         //if previous sensor has returned a value or timed out 
    sensor_ready = false;           //reset ready condition    
    
    sensor +=1;             //adress next sensor
    sensor = (sensor%8);    //loop around after 8th sensor
  
    trigPin = triggerpins[sensor];    
    echoPin = echopins[sensor];

    // Fire respective trigger    
    digitalWrite(trigPin, LOW);   // Clears the trigPin condition    
    delayMicroseconds(2);    
    digitalWrite(trigPin, HIGH);  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds    
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    trigger_time = micros();    
       
    attachInterrupt(echoPin, echorising, RISING); //listen for response on appropriate echo pin 
    }

  if (micros()>trigger_time+(800/0.034)){ //trigger timeout if no responce is received
    detachInterrupt(echoPin); 
    sensor_ready = true; 
    durations[sensor] = 800/0.034;
  };



  //ROS///////////////////////////////////////////////////////////////////////////////////
  if (micros() > publisher_timer) {
    publisher_timer = micros() + 40000; //publish 25 times per second
    // update distances array before publishing
    for(int i = 0; i < 8; i++){
      distances[i] = durations[i]/2 *0.034;
      }
             
      radar_state.radar1 = distances[0];
      radar_state.radar2 = distances[1];
      radar_state.radar3 = distances[2];
      radar_state.radar4 = distances[3];
      radar_state.radar5 = distances[4];
      radar_state.radar6 = distances[5];
      radar_state.radar7 = distances[6];
      radar_state.radar8 = distances[7];
            
      pub_radar.publish(&radar_state); 
  }
  
  nh.spinOnce();
  //ROS///////////////////////////////////////////////////////////////////////////////////
  
}


void echorising(){
  attachInterrupt(echoPin, echofalling, FALLING); 
  echostarttime = micros(); // echo start time
}
void echofalling(){
  durations[sensor] = micros()-echostarttime; // echo total time  
  sensor_ready = true;
}

//  For debugging
//  Serial.print("Distances: ");
//  Serial.print(distances[0]);
//  Serial.print(", ");
//  Serial.print(distances[1]);
//  Serial.print(", ");
//  Serial.print(distances[2]);
//  Serial.print(", ");
//  Serial.print(distances[3]);
//  Serial.print(", ");
//  Serial.print(distances[4]);
//  Serial.print(", ");
//  Serial.print(distances[5]);
//  Serial.print(", ");
//  Serial.print(distances[6]);
//  Serial.print(", ");
//  Serial.println(distances[7]);
