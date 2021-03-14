# Componenents Explained


This page explains the choice of individual components, as well as how they are used, and what can be improved.

## The wheel speed sensors 
The rotational velocity of the wheels is important information when vehicle dynamics are considered. It is often the only measurement we have of the interface between the vehicle and the road.
It can be used to determine the velocity of the vehicle (from the non-driven wheels) or to estimate the longitudinal slip of the driven wheels.

Conventionally, wheel speeds are measured with hall sensors and a magnetic ring. This is also the method used on a large research platform such as the [AutoRally project](https://autorally.github.io/).

The rotational velocity is measured with IR-reflectance sensors. A small IR transmitter and receiver board is mounted on each wheel hub. A disk is mounted inside the wheels with aluminium tape to reflect the IR light. The gaps in the disk obviously do not reflect light, generating a square wave signal when the wheel is rotating.

Experience:
IR-reflectance sensors can typically be bought in a 'Digital' or 'Analog' version. According to the documentation of the Berkeley Autonomous Race Car (BARC), digital sensors are used. 
A digital sensor determines the reflectivity of a surface by pulling a voltage to 'HIGH', and counting how long it takes to discharge a small capacitor due to the amount of light reflected. This is a very slow process, and this gives no information about when the surface switches from 'light' to 'dark'. From communication with one of the guys still working on the development of the BARC, they are still having problems getting reliable data from the sensors.

A much better solution is to use an **analog** version of the board. An analog board puts out a voltage as a function of the reflected light. Since we are only interested in 'light' or 'dark', we can read the sensor with a **DIGITAL** pin on the arduino. Whenever the surface switches from 'light' to 'dark', the voltage switches from HIGH to LOW, and an Interrupt routine is called to note the time of the switch.

The BARC also uses encoder disks with a black and white pattern to create a 'light' and 'dark' surface. This is not sufficient; a dark colored surface can still reflect IR light. A much more robust 'dark' surface is a surface that is sufficiently far away, regardless of color. Therefor a robust solution was found in using 'slotted' encoder disks.

Note: An analog board with a 47k resistor works best. Boards with a lower resistance will not reliable pull the voltage low enough to trigger Arduino's Digital LOW threshold.
