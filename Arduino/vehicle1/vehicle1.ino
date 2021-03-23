// ---------------------------------------------------------------- //
// Arduino Ultrasoninc Sensor array for research vehicle
// Arduino IDE < 1.8.4
// Created for the Department of Cognitive Robotics (CoR) TuDelft
// 
// ---------------------------------------------------------------- //


#include "ICM_20948.h"  // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include "math.h"
#include <Servo.h>

//ROS///////////////////////////////////////////////////////////////////////////////////
#include <ros.h>
#include <car/vehicle_state_msg.h>
#include <car/control_actions_msg.h>
ros::NodeHandle  nh;

float steering_out = 1500.00;
float throttle_out = 1500.00;
void controlCallback(const car::control_actions_msg& pwm_signals){
steering_out = pwm_signals.steering_action;
throttle_out = pwm_signals.throttle_action;    
}

car::control_actions_msg control_actions;
car::vehicle_state_msg vehicle_state;
ros::Publisher pub_vehicle("vehicle_state", &vehicle_state);
ros::Subscriber<car::control_actions_msg> sub_control_actions("control_actions", controlCallback);

long publisher_timer; 
//ROS///////////////////////////////////////////////////////////////////////////////////



#define USE_SPI       // Uncomment this to use SPI
#define SERIAL_PORT Serial
#define SPI_PORT SPI    // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 10        // Which pin you connect CS to. Used only when "USE_SPI" is defined
#define WIRE_PORT Wire  // Your desired Wire port.      Used when "USE_SPI" is not defined
#define AD0_VAL   1     // The value of the last bit of the I2C address. 
                        // On the SparkFun 9DoF IMU breakout the default is 1, and when 
                        // the ADR jumper is closed the value becomes 0

#ifdef USE_SPI
  ICM_20948_SPI myICM;  // If using SPI create an ICM_20948_SPI object
#else
  ICM_20948_I2C myICM;  // Otherwise create an ICM_20948_I2C object
#endif

#define DEG2RAD 0.0174533
#define RAD2DEG 57.2958
#define MG2MSS 0.00981
#define pi 3.141593  
#define RPM2RADS 0.104719755


  float u = 0;
  float v = 0;
  float r = 0;
  
  float a_x = 0;
  float a_y = 0;
  float a_z = 0;
  float a_x_prev = 0;
  float a_y_prev = 0;
  float a_z_prev = 0;

  
  float g_x = 0;
  float g_y = 0;
  float g_z = 0;
  float g_x_prev = 0;
  float g_y_prev = 0;
  
  float alpha_x = 0;
  float alpha_y = 0;  
  float alpha_x_prev = 0;
  float alpha_y_prev = 0;

  float rx = 0;
  float ry = 0;
  float rz = 0;
  float rx2 = 0;
  float ry2 = 0;
  float rz2 = 0;
  
  float a_x_offset = 0;
  float a_y_offset = 0;
  float a_z_offset = 0;
  float g_x_offset = 0;
  float g_y_offset = 0;
  float g_z_offset = 0;
    
  unsigned long t1;
  unsigned long t2; 
  float dt; // Can probably be long?
 
  int STATIC_COUNT = 0;
  int accel_free = 0;

//RX,TX
Servo steering;
Servo throttle;

volatile int pwm_value_ch1 = 0;
volatile int prev_time_ch1 = 0;
volatile int pwm_value_ch2 = 0;
volatile int prev_time_ch2 = 0;
volatile int pwm_value_ch3 = 0;
volatile int prev_time_ch3 = 0;

int ST_in = 1;
int TH_in = 2;
int ST_out = A6;
int TH_out = A7;

//Wheel Encoders
#define ppr 8       //amount of encoder pulses per rotation

volatile unsigned long curr_time_fl = 0;
volatile unsigned long curr_time_fr = 0;
volatile unsigned long curr_time_rl = 0;
volatile unsigned long curr_time_rr = 0;

volatile unsigned long prev_time_fl = 0;
volatile unsigned long prev_time_fr = 0;
volatile unsigned long prev_time_rl = 0;
volatile unsigned long prev_time_rr = 0;

unsigned long system_time = 0;
#define time_limit 250000 //Timeout in micros before wheel speeds are pulled to 0

float rpm_fl = 0;
float rpm_fr = 0;
float rpm_rl = 0;
float rpm_rr = 0;


void setup() {
  //ROS///////////////////////////////////////////////////////////////////////////////////
  nh.initNode();
  nh.advertise(pub_vehicle);
  nh.subscribe(sub_control_actions);
  //ROS///////////////////////////////////////////////////////////////////////////////////

  
  // TX,RX    
  attachInterrupt(1, risingch1, RISING);
  attachInterrupt(2, risingch2, RISING);
  attachInterrupt(3, risingch3, RISING);
  pinMode(ST_in, INPUT);
  pinMode(TH_in, INPUT);

  steering.attach(ST_out);
  steering.writeMicroseconds(1500);
  throttle.attach(TH_out);
  throttle.writeMicroseconds(1500);

  //Wheel Encoders
  attachInterrupt(4, encoder_fl, RISING);
  attachInterrupt(5, encoder_fr, RISING);
  attachInterrupt(6, encoder_rl, RISING);
  attachInterrupt(7, encoder_rr, RISING);


  //Serial connection  
  SERIAL_PORT.begin(115200);
  while(!SERIAL_PORT){}; //Do nothing if serial not connected?

  #ifdef USE_SPI
    SPI_PORT.begin();
  #else
    WIRE_PORT.begin();
    WIRE_PORT.setClock(400000);
  #endif
  
  bool initialized = false;
  while( !initialized ){

  #ifdef USE_SPI
    myICM.begin( CS_PIN, SPI_PORT ); 
  #else
    myICM.begin( WIRE_PORT, AD0_VAL );
  #endif
    
    SERIAL_PORT.print( F("Initialization of the sensor returned: ") );
    SERIAL_PORT.println( myICM.statusString() );
    if( myICM.status != ICM_20948_Stat_Ok ){
      SERIAL_PORT.println( "Trying again..." );
      delay(500);
    }else{
      initialized = true;
    }
  }

  // In this advanced example we'll cover how to do a more fine-grained setup of your sensor
  SERIAL_PORT.println("Device connected!");

  // Here we are doing a SW reset to make sure the device starts in a known state
  myICM.swReset( );
  if( myICM.status != ICM_20948_Stat_Ok){
    SERIAL_PORT.print(F("Software Reset returned: "));
    SERIAL_PORT.println(myICM.statusString());
  }
  delay(250);
  
  // Now wake the sensor up
  myICM.sleep( false );
  myICM.lowPower( false );

  // The next few configuration functions accept a bit-mask of sensors for which the settings should be applied.

  // Set Gyro and Accelerometer to a particular sample mode
  // options: ICM_20948_Sample_Mode_Continuous
  //          ICM_20948_Sample_Mode_Cycled
  myICM.setSampleMode( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous ); 
  if( myICM.status != ICM_20948_Stat_Ok){
    SERIAL_PORT.print(F("setSampleMode returned: "));
    SERIAL_PORT.println(myICM.statusString());
  }

  // Set full scale ranges for both acc and gyr
  ICM_20948_fss_t myFSS;  // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors
  
  myFSS.a = gpm2;         // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                          // gpm2
                          // gpm4
                          // gpm8
                          // gpm16
                          
  myFSS.g = dps500;       // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                          // dps250
                          // dps500
                          // dps1000
                          // dps2000
                          
  myICM.setFullScale( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS );  
  if( myICM.status != ICM_20948_Stat_Ok){
    SERIAL_PORT.print(F("setFullScale returned: "));
    SERIAL_PORT.println(myICM.statusString());
  }


  // Set up Digital Low-Pass Filter configuration
  ICM_20948_dlpcfg_t myDLPcfg;            // Similar to FSS, this uses a configuration structure for the desired sensors
  myDLPcfg.a = acc_d473bw_n499bw;         // (ICM_20948_ACCEL_CONFIG_DLPCFG_e)
                                          // acc_d246bw_n265bw      - means 3db bandwidth is 246 hz and nyquist bandwidth is 265 hz
                                          // acc_d111bw4_n136bw
                                          // acc_d50bw4_n68bw8
                                          // acc_d23bw9_n34bw4
                                          // acc_d11bw5_n17bw
                                          // acc_d5bw7_n8bw3        - means 3 db bandwidth is 5.7 hz and nyquist bandwidth is 8.3 hz
                                          // acc_d473bw_n499bw

  myDLPcfg.g = gyr_d361bw4_n376bw5;       // (ICM_20948_GYRO_CONFIG_1_DLPCFG_e)
                                          // gyr_d196bw6_n229bw8
                                          // gyr_d151bw8_n187bw6
                                          // gyr_d119bw5_n154bw3
                                          // gyr_d51bw2_n73bw3
                                          // gyr_d23bw9_n35bw9
                                          // gyr_d11bw6_n17bw8
                                          // gyr_d5bw7_n8bw9
                                          // gyr_d361bw4_n376bw5
                                          
  myICM.setDLPFcfg( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg );
  if( myICM.status != ICM_20948_Stat_Ok){
    SERIAL_PORT.print(F("setDLPcfg returned: "));
    SERIAL_PORT.println(myICM.statusString());
  }

  // Choose whether or not to use DLPF
  // Here we're also showing another way to access the status values, and that it is OK to supply individual sensor masks to these functions
  ICM_20948_Status_e accDLPEnableStat = myICM.enableDLPF( ICM_20948_Internal_Acc, true );
  ICM_20948_Status_e gyrDLPEnableStat = myICM.enableDLPF( ICM_20948_Internal_Gyr, true );
  SERIAL_PORT.print(F("Enable DLPF for Accelerometer returned: ")); SERIAL_PORT.println(myICM.statusString(accDLPEnableStat));
  SERIAL_PORT.print(F("Enable DLPF for Gyroscope returned: ")); SERIAL_PORT.println(myICM.statusString(gyrDLPEnableStat));

  SERIAL_PORT.println();
  SERIAL_PORT.println(F("Configuration complete!")); 
 

  // Calibration procedure
 delay(500);
  if( myICM.dataReady() ){        
    for (int i = 0; i <= 9999; i++) {
      myICM.getAGMT();     
      a_x_offset += myICM.accX();
      a_y_offset += myICM.accY();
      a_z_offset += myICM.accZ()-1000;
      g_x_offset += myICM.gyrX();
      g_y_offset += myICM.gyrY();
      g_z_offset += myICM.gyrZ();
    }
  }else{
    Serial.println("Not Calibrated!");
  }  
  a_x_offset/=10000;
  a_y_offset/=10000;
  a_z_offset/=10000;
  g_x_offset/=10000;
  g_y_offset/=10000;
  g_z_offset/=10000;
  
  
  
}


void loop() {
  
  // RX,TX
  if (pwm_value_ch3 > 1500){    // manual steering and throttle control       
       steering.writeMicroseconds(pwm_value_ch1); 
       throttle.writeMicroseconds(pwm_value_ch2);        
  }else{
       steering.writeMicroseconds(steering_out); 
       throttle.writeMicroseconds(throttle_out); 
  }

  //Wheel Encoders
  system_time = micros();
  if (curr_time_fl+time_limit>system_time){ //If last update not too long ago
    rpm_fl = (1/((curr_time_fl-prev_time_fl)*0.000001))*60/ppr;
    }
    else{  // pull RPM to 0
    rpm_fl = 0;
    }    
  if (curr_time_fr+time_limit>system_time){ //If last update not too long ago
    rpm_fr = (1/((curr_time_fr-prev_time_fr)*0.000001))*60/ppr;
    }
    else{
    rpm_fr = 0;
    }      
  if (curr_time_rl+time_limit>system_time){ //If last update not too long ago
    rpm_rl = (1/((curr_time_rl-prev_time_rl)*0.000001))*60/ppr;
    }
    else{
    rpm_rl = 0;
    }
  if (curr_time_rr+time_limit>system_time){ //If last update not too long ago
    rpm_rr = (1/((curr_time_rr-prev_time_rr)*0.000001))*60/ppr;
    }
    else{
    rpm_rr = 0;
    }  

  // Get sensor values
  myICM.getAGMT();                // The values are only updated when you call 'getAGMT'  
  a_x = (myICM.accX());//-a_x_offset;
  a_y = (myICM.accY());//-a_y_offset;
  a_z = (myICM.accZ());//-a_z_offset;
  g_x = myICM.gyrX()-g_x_offset;
  g_y = myICM.gyrY()-g_y_offset;
  g_z = myICM.gyrZ()-g_z_offset;
  
  t2 = micros();
  dt = (t2-t1)/1000000.0;   
  t1 = t2; // Update t1 for next iteration

  if (sqrt(a_x*a_x+a_y*a_y+a_z*a_z)<1020) {
    STATIC_COUNT +=1;
  }else{
    STATIC_COUNT = 0;      
  }
      
  if (STATIC_COUNT > 25){
    u = 0;
    v = 0;
    accel_free = 1;
  }else{
    accel_free=0;
  }

  if (accel_free ==1){
    alpha_x = 0.99*(alpha_x+(g_x_prev+0.5*(g_x-g_x_prev))*dt)+0.01*(atan2(a_y,a_z)*RAD2DEG); //angles in deg
    alpha_y = 0.99*(alpha_y+(g_y_prev+0.5*(g_y-g_y_prev))*dt)+0.01*(-atan2(a_x,a_z)*RAD2DEG);
  }else{
    alpha_x += (g_x_prev+0.5*(g_x-g_x_prev))*dt;
    alpha_y += (g_y_prev+0.5*(g_y-g_y_prev))*dt;
    }

  g_x_prev = g_x;
  g_y_prev = g_y; 


  //rotate over x axis
  rx = a_x;
  ry = cos(alpha_x*DEG2RAD)*a_y - sin(alpha_x*DEG2RAD)*a_z;
  rz = sin(alpha_x*DEG2RAD)*a_y + cos(alpha_x*DEG2RAD)*a_z;

  //rotate over y axis
  rx2 = cos(alpha_y*DEG2RAD)*rx + sin(alpha_y*DEG2RAD)*rz;
  ry2 = ry;
  rz2 = -sin(alpha_y*DEG2RAD)*rx + cos(alpha_y*DEG2RAD)*rz;

  //convert from mg to m/s
  a_x = rx2*MG2MSS;
  a_y = ry2*MG2MSS;

  u += (a_x_prev+0.5*(a_x-a_x_prev))*dt; //First order integration with linear interpolation
  v += (a_y_prev+0.5*(a_y-a_y_prev))*dt;
  r = g_z*DEG2RAD;
  
  a_x_prev = a_x; //save 
  a_y_prev = a_y;

 

    //ROS///////////////////////////////////////////////////////////////////////////////////

      if (micros() > publisher_timer) {
      publisher_timer = micros() + 40000;
      
      vehicle_state.u = u;
      vehicle_state.v = v;
      vehicle_state.r = r;

      vehicle_state.a_x = a_x;
      vehicle_state.a_y = a_y;

      vehicle_state.omega_fl = rpm_fl*RPM2RADS;
      vehicle_state.omega_fr = rpm_fr*RPM2RADS;
      vehicle_state.omega_rl = rpm_rl*RPM2RADS;
      vehicle_state.omega_rr = rpm_rr*RPM2RADS;
      
      pub_vehicle.publish(&vehicle_state);
      }
      nh.spinOnce();
    //ROS///////////////////////////////////////////////////////////////////////////////////

  
    
}

void risingch1() {
  attachInterrupt(1, fallingch1, FALLING);
  prev_time_ch1 = micros();
} 
void fallingch1() {
  attachInterrupt(1, risingch1, RISING);
  pwm_value_ch1 = micros()-prev_time_ch1;
}
void risingch2() {
  attachInterrupt(2, fallingch2, FALLING);
  prev_time_ch2 = micros();
}
void fallingch2() {
  attachInterrupt(2, risingch2, RISING);
  pwm_value_ch2 = micros()-prev_time_ch2;
}
void risingch3() {
  attachInterrupt(3, fallingch3, FALLING);
  prev_time_ch3 = micros();
} 
void fallingch3() {
  attachInterrupt(3, risingch3, RISING);
  pwm_value_ch3 = micros()-prev_time_ch3;
}

void encoder_fl() {
  prev_time_fl = curr_time_fl;
  curr_time_fl = micros();  
}
void encoder_fr() {
  prev_time_fr = curr_time_fr;
  curr_time_fr = micros();
}
void encoder_rl() {
  prev_time_rl = curr_time_rl;
  curr_time_rl = micros();
}
void encoder_rr() {
  prev_time_rr = curr_time_rr;
  curr_time_rr = micros();
}
