// ---------------------------------------------------------------- //
// Arduino Ultrasoninc Sensor array for research vehicle
// Arduino IDE < 1.8.4
// Using HC-SR04 Module
// Created for the Department of Cognitive Robotics (CoR) TuDelft
// ---------------------------------------------------------------- //

// Triggering all sensors at the same time on common trigger pin

#define trigPin A0      // common trigger pin

#define echoPin1 4      // echo pin sensor 1
#define echoPin2 5      // echo pin sensor 2
#define echoPin3 6      // echo pin sensor 3
#define echoPin4 7      // echo pin sensor 4
#define echoPin5 8      // echo pin sensor 5
#define echoPin6 9      // echo pin sensor 6
#define echoPin7 10     // echo pin sensor 7
#define echoPin8 11     // echo pin sensor 8

bool waiting = false;   // Time-out condition
unsigned long distances[] = {401,402,403,404,405,406,407,408};
volatile unsigned long durations[] = {1,2,3,4,5,6,7,8};
volatile unsigned long echostarttimes[] = {1,2,3,4,5,6,7,8};
unsigned long trigger_time = 10;  // store trigger time

//ROS///////////////////////////////////////////////////////////////////////////////////
#include <ros.h>
#include <car/radar_state_msg.h>
ros::NodeHandle  nh;
car::radar_state_msg  radar_state;
ros::Publisher pub_radar("radar_state", &radar_state); //function, topic, msg

long publisher_timer;  // store ROS publish time
//ROS///////////////////////////////////////////////////////////////////////////////////

void setup() {
    //ROS///////////////////////////////////////////////////////////////////////////////////
    nh.initNode();
    nh.advertise(pub_radar);
    //ROS///////////////////////////////////////////////////////////////////////////////////  
    pinMode(trigPin, OUTPUT); 
    Serial.begin(115200); // 
}



void loop() {

  if (micros()>trigger_time+40000){ // 25 times a second
    
      //Calculate distances and publish
      for(int i = 0; i < 8; i++){
      distances[i] = durations[i]/2 *0.034; 
      }
      
      //ROS///////////////////////////////////////////////////////////////////////////////////
      radar_state.radar1 = distances[0];
      radar_state.radar2 = distances[1];
      radar_state.radar3 = distances[2];
      radar_state.radar4 = distances[3];
      radar_state.radar5 = distances[4];
      radar_state.radar6 = distances[5];
      radar_state.radar7 = distances[6];
      radar_state.radar8 = distances[7];
      
      pub_radar.publish(&radar_state); 
      //ROS///////////////////////////////////////////////////////////////////////////////////    




    for(int i = 0; i < 8; i++){ 
      durations[i] = 800/0.034;
      }
         
    waiting = true; //reset time out condition    
    digitalWrite(trigPin, LOW);   // Clears the trigPin condition
    delayMicroseconds(2);    
    digitalWrite(trigPin, HIGH);  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    attachInterrupt(echoPin1, echo1rising, RISING);         //listen for responses
    attachInterrupt(echoPin2, echo2rising, RISING);
    attachInterrupt(echoPin3, echo3rising, RISING);
    attachInterrupt(echoPin4, echo4rising, RISING);
    attachInterrupt(echoPin5, echo5rising, RISING);
    attachInterrupt(echoPin6, echo6rising, RISING);     
    attachInterrupt(echoPin7, echo7rising, RISING);
    attachInterrupt(echoPin8, echo8rising, RISING);        
    trigger_time = micros();
    }

  if ((micros()>trigger_time+(800/0.034)) && (waiting)){    //trigger timeout if no responce is received
    detachInterrupt(echoPin1);
    detachInterrupt(echoPin2);
    detachInterrupt(echoPin3);
    detachInterrupt(echoPin4);
    detachInterrupt(echoPin5);
    detachInterrupt(echoPin6);     
    detachInterrupt(echoPin7);
    detachInterrupt(echoPin8);
    waiting = false;                                        // Make sure it only times out once.
  };

  
  //ROS///////////////////////////////////////////////////////////////////////////////////
  nh.spinOnce();
  //ROS///////////////////////////////////////////////////////////////////////////////////

}


void echo1rising(){
  attachInterrupt(echoPin1, echo1falling, FALLING); 
  echostarttimes[0] = micros();
}
void echo1falling(){
  durations[0]= micros()-echostarttimes[0];
}

void echo2rising(){
  attachInterrupt(echoPin2, echo2falling, FALLING); 
  echostarttimes[1] = micros();
}
void echo2falling(){
    durations[1]= micros()-echostarttimes[1];
}

void echo3rising(){
  attachInterrupt(echoPin3, echo3falling, FALLING); 
  echostarttimes[2] = micros();
}
void echo3falling(){
  durations[2]= micros()-echostarttimes[2];
}

void echo4rising(){
  attachInterrupt(echoPin4, echo4falling, FALLING); 
  echostarttimes[3] = micros();
}
void echo4falling(){
  durations[3]= micros()-echostarttimes[3];
}

void echo5rising(){
  attachInterrupt(echoPin5, echo5falling, FALLING); 
  echostarttimes[4] = micros();
}
void echo5falling(){
  durations[4]= micros()-echostarttimes[4];
}

void echo6rising(){
  attachInterrupt(echoPin6, echo6falling, FALLING); 
  echostarttimes[5] = micros();
}
void echo6falling(){
  durations[5]= micros()-echostarttimes[5];
}

void echo7rising(){
  attachInterrupt(echoPin7, echo7falling, FALLING); 
  echostarttimes[6] = micros();
}
void echo7falling(){
  durations[6]= micros()-echostarttimes[6];
}

void echo8rising(){
  attachInterrupt(echoPin8, echo8falling, FALLING); 
  echostarttimes[7] = micros();
}
void echo8falling(){
  durations[7]= micros()-echostarttimes[7];
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
