# Starting ROS

1. Open a terminal (Ctrl+Alt+T), and start roscore by executing the following command:
  > roscore


2. Open another terminal, and change directory to your ROS workspace:
  > cd/tu_ws
3. Source the setup.bash file in your workspace:
  > source devel/setup.bash

You can now start any of the ROS programs in your workspace.

5. To start the serial communication over USB, run the serial_launch.launch file. You can use a launch file to execute multiple ROS programs with 1 command. In this case, it launches two nodes that listen to the messages from the Arduino's.
  > roslaunch car serial_launch.launch

6. To launch a controller, -*for example controller.py*-,  repeat steps 2-3 and start your executable:
  >rosrun car controller.py


# Switching between manual and automated driving modes

When the transmitter is turned on, the vehicle is in "manual mode". This means that the control signals from the transmitter are directly sent to the servo and speed controller. You are in control of the car.

When the "CH3" button is pressed on the transmitter (transmitter panel, top left), the Arduinos will relay the commands from the Odroid on the ROS network. The vehicle is in "automatic mode". When the "CH3" button is pressed again, the vehicle returns to "manual mode".

Start the vehicle with the wheels raised off the floor. This prevents damage from the vehicle accidently driving away.

When the Arduino is powered on or reset, the accelerometer is calibrated for 5 seconds. Place the vehicle on a flat surface, reset the Arduino and do not touch the vehicle for a couple seconds. 	
