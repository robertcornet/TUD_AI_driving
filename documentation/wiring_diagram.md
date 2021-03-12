# Wiring diagram for the Arduinos

## Nano1 : RC IN/OUT, ENCODERS IN, IMU IN (vehicle state)
		
		A0
		A1
		A2
		A3
		A4
		A5
		A6 = To Servo
		A7 = To Electronic Speed Controller

		1 = Steering in 										(receiver channel 1)
		2 = Throttle in 										(receiver channel 2)
		3 = AUX in                          (receiver channel 3)

		4 = Wheel Encoder (FL)
		5 = Wheel Encoder (FR)
		6 = Wheel Encoder (RL) 
		7 = Wheel Encoder (RR)

		8 
		9 
		10 = IMU (CS,1)
		11 = IMU (MOSI,4)

		12 = IMU(MISO,2)
		13 = IMU(SCLK,3)
		GND
		AREF

## SPARKFUN IMU
		CS = NANO PIN 10
		MISO = NANO PIN 12
		SCLK = NANO PIN 13
		MOSI = NANO PIN 11
		IV8-5V5 = 5V
		GND = GND



## Nano 2 : Ultrasonic Sensor Array 

		A0 = Trigger radar 1
		A1 = Trigger radar 2
		A2 = Trigger radar 3
		A3 = Trigger radar 4
		A4 = Trigger radar 5
		A5 = Trigger radar 6
		A6 = Trigger radar 7 
		A7 = Trigger radar 8

		1
		2
		3

		4 = Echo radar 1
		5 = Echo radar 2
		6 = Echo radar 3
		7 = Echo radar 4

		8 = Echo radar 5
		9 = Echo radar 6
		10 = Echo radar 7
		11 = Echo radar 8

		12
		13
		GND
		AREF
