# TU Delft AI driving platform
This page contains all the information for the TU Delft AI driving platform. It is designed for the department of Cognitive Robotics (CoR) at the faculty of 3ME at the TU Delft.

<img src="/images/TUDAI1.jpg" height=256>

## Who is it for?
This platform is designed for very dynamic driving tasks, such as drifting. It can show the unstable behaviour of cars that are driven on the limit of friction. 

If you are into vehicle research that goes beyond navigation tasks and focusses on dynamic vehicle behaviour and vehicle stability, this platform can be for you. It offers the dynamics of the large research platforms on a small scale that requires little space. The onboard accelerometers, gyros and individual wheel speed sensors provide a good basis for developing impressive vehicle controllers.

## What does it offer?
- Low-cost dynamic driving with access to the relevant vehicle states. (wheel speeds, accelerations, yaw rate)
- Small scale suitable for compact spaces
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
- Better Inertial Navigation
- Camera Vision / Lidar module
- Standstill vehicle rotating bug
- Large steering angle mod
