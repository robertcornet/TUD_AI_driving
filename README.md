# TU Delft AI driving platform
Here you can learn everything about the TU Delft AI driving platform. It is designed for the department of Cognitive Robotics (CoR) at the faculty of 3ME at the TU Delft.

<a></a>
<img src="/images/106.jpg" height=300>

<a></a>
<img src="/images/101.jpg" height=200>
<img src="/images/102.jpg" height=200>
<img src="/images/100.jpg" height=200>
<img src="/images/103.jpg" height=200>

## Who is it for?
This car is designed for very dynamic driving tasks, such as drifting. It can show the unstable behaviour of cars that are pushed to the limit of friction, at low speed and in small spaces. 

If you are into vehicle research that goes beyond navigation tasks and focusses on dynamic vehicle behaviour and vehicle stability, this platform can be for you. It offers the dynamics of large research platforms on a small scale that requires little space. The onboard accelerometers, gyros and individual wheel speed sensors provide a good basis for developing impressive vehicle controllers.

## What does it offer?
- Low-cost dynamic driving
- Small scale for indoor use
- Access to the relevant vehicle states. (wheel speeds, accelerations, yaw rate)
- Onboard TensorFlow integration for AI development
- Modular platform for easy addition of your own sensors

## Getting Started
- Order everything you need from the [Bill of Materials](/documentation/bill_of_materials.md) 
- Print the [CAD models](/cad)
- Assemble your car with the help of the [Build Guide](/documentation/build_guide.md)
- Install the [Software](/documentation/software_setup.md)

Your platform should now be ready to begin! 
If you want some extra help, check out the [Operation Manual](/documentation/operation_manual.md). If you want to build a vehicle simulation to train your AI, check out the [Simulator section](/simulator).

## See Also
- [BARC](https://github.com/MPC-Berkeley/barc)
- [NVIDIA JetRacer](https://github.com/NVIDIA-AI-IOT/jetracer)
- [AutoRally](https://autorally.github.io/)
- [DonkeyCar](https://www.donkeycar.com/)


## To Do
- Large steering angle mod. _For rear wheel drive drifting, a large steering angle is needed to balance the vehicle in the drift._
- Camera Vision / Lidar module. _The usability of the Ultrasonic sensors is limited._
- Better Inertial Navigation. _Robust algorithm to determine vehicle orientation in space._
- Standstill vehicle rotating bug. _In the simulator, if the vehicle is stationary, it can rotate slowly by steering. This is due to how the slip angles are defined._

