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

To launch a controller, -*for example controller.py*-,  repeat steps 2-3 and start your executable:
>rosrun car controller.py


