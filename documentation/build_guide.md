# Build Guide

This document will guide you through the build process of the vehicle. Get your tools and materials ready and build along!
If you have questions about the choice of components, refer to [here](/documentation/components_explained.md)

### Required skills
- Soldering servo wires, header pins and battery connectors
- Basic assembly skills
 
### Required tools and materials
- Set of Phillips-style screwdrivers
- 2mm Hex screwdriver
- Access to a 3D-printer or 3D-print service
- Knife, scissors, wire stripping tools
- Hot glue gun
- M3 thread cutting tool
- Soldering iron
- Adhesive aluminium foil/tape or IR-reflective tape
- Small zip ties

# Part 1: The Chassis
Start by building the TT-02 chassis with the help of the included instructions. Use the included foam bumper (black) in the rear, and the  optional TT-01 bumper (blue) in the front. 

Connect the motor:\
blue - green\
yellow - yellow\
orange - \[-\]\


Your vehicle should now look similar to this:

<a></a>
<img src="/images/01.jpg" height=200/>
<img src="/images/02.jpg" height=200/>


# Part 2: Battery Power Wiring
The power from the battery is split to the Vehicle Electronic Speed Controller (ESC) and the DC-DC step down to power on-board computer. We will also add a third connector so we can monitor the battery voltage. The auxiliary connector is a standard servo wire with the signal wire removed:

<img src="/images/04.jpg" height=200/>

Cut the Tamiya connector off the ESC, and wire in the DC-DC converter, the battery connector and the auxiliary port.

Solder the wires in parallel and pay attention to the polarity of the components. Leave about 15cm of battery wire. Cover the wires with heat shrink tube (remember to put it in place before soldering the wires together). This protects the exposed wires from shorting and provides strain relief for the smaller wires:

<a></a>
<img src="/images/05.jpg" height=200/>
<img src="/images/07.jpg" height=200/>
<img src="/images/09.jpg" height=200/>
 
 
Solder the barrel connector to the output of the DC-DC step down converter. **Make sure the output is set to 5 volts!**


# Part 3: The Mounting Plate
The holes in the 3d printed parts are not threaded yet. It is recommended to run a tap trough the holes first. The holes are sized for M3x0.5 threads.

<a></a>
<img src="/images/12.jpg" height=200/>

The threads in the upper deck are intended to be reused frequently, and therefor made with m3 nuts. The hexagon cutouts are slightly undersized. The nuts can be pressed in the deck, but it is recommended to use a little bit of heat to soften the plastic. This can be done with a soldering iron on low heat. Heat the nut until the plastic starts to soften and the nut sinks into the deck. This should only take 5~10 seconds on 250-300 C.  Quickly remove the heat source, turn the part around and press it on a flat surface for a few seconds. This way, the nuts will sit flat and square:

<a></a>
<img src="/images/20.jpg" height=200/>
<img src="/images/22.jpg" height=200/>
<img src="/images/23.jpg" height=200/>


Remove the screws on the front and rear bulkhead and use them to mount the deck supports:

<a></a>
<img src="/images/15.jpg" height=200/>
<img src="/images/19.jpg" height=200/>

Mount the deck brace to the underside of the front and rear deck plates, and mount the combined deck to the deck supports:

<a></a>
<img src="/images/24.jpg" height=200/>
<img src="/images/25.jpg" height=200/>
<img src="/images/27.jpg" height=200/>

# Part 4, Electronics
## Wheels speed sensors and encoder disks
 
Cut the connector of 1 end of the 3-pin wires and solder them to the IR reflectance sensors (4x). 
Use the following lengths or more:
2x 50cm (rear wheels)
2x 30 cm (front wheels)

<a></a>
<img src="/images/33.jpg" height=200/>
<img src="/images/34.jpg" height=200/>
<img src="/images/35.jpg" height=200/>

Remove the wheels and the standard wheel carriers. Use hot glue to secure the sensor mounts in the orientation as shown, with the mounting plate facing down and away from the center! It may be necessary to trim some of the excess material on the wheel hubs.

<a></a>
<img src="/images/32.jpg" height=200/>
<img src="/images/31.jpg" height=200/>


Bolt the sensors to the wheel hubs with M2x6 bolts and zip tie the wires to the linkages for some strain relief from the solder connections:

<a></a>
<img src="/images/36.jpg" height=200/>

Cover the encoder disks with aluminium foil or similar with good IR reflectivity, and mount them on the axles. They will snap in place on the original locking pin:

<a></a>
<img src="/images/75.jpg" height=200/>
<img src="/images/76.jpg" height=200/>

## IMU, Arduino, Odroid

Screw the IMU down on the IMU support plate, and mount the support plate in the center of the deck. Do the same for the Arduino Nano shield, and the Odroid computer mount:

<a></a>
<img src="/images/42.jpg" height=200/>
<img src="/images/43.jpg" height=200/>
<img src="/images/58.jpg" height=200/>

## Wiring and Cable Management
Hook up the sensors to the Arduino's with the help of the [Wiring Diagram](/documentation/wiring_diagram.md).

<a></a>
<img src="/images/44.jpg" height=200/>
<img src="/images/45.jpg" height=200/>
<img src="/images/48.jpg" height=200/>
<img src="/images/47.jpg" height=200/>

It is important that all wires are clear of the moving parts of the vehicle. This applies mainly to the center driveshaft and the steering linkages.
The DC-DC converter can be zip tied to the motor wires under the top deck:

<a></a>
<img src="/images/50.jpg" height=200/>

The encoder cables and IMU cable can be secured between the IMU mounting plate and the deck brace:

Tie the battery wires to the brace as well, so you are not pulling on the fragile wires when changing batteries!

<a></a>
<img src="/images/57.jpg" height=200/>

RECEIVER
The receiver can be wired to the Arduino with 3x 3 pin connectors:

<a></a>
<img src="/images/46.jpg" height=200/>

Stick the receiver on top of the servo with double sided tape.
ESC/SERVO POWER

The Tamiya ESC provides 5v power over the 3 pin conector. We will use that to power the steering servo. This is to make sure the Arduino is not powered from two different sources (USB and ESC) and the servo doesn’t draw too much power from the Arduino. 
We will remove the positive (+) pin from the 3 pin connectors and wire them together. You can use a knife or small screwdriver to lift the retention on the connector:

<a></a>
<img src="/images/53.jpg" height=200/>
<img src="/images/54.jpg" height=200/>
<img src="/images/55.jpg" height=200/>

2 header pins can be used to make a jumper, or the wires can be soldered together directly. Make sure the wires can not create a short on any of the components!


<a></a>
<img src="/images/51.jpg" height=200/>
<img src="/images/52.jpg" height=200/>

## Ultrasonic Sensors

You can add aditional sensors to the car if you like. Here we will show how you can add an array of ultrasonic sensors to the vehicle.

We will be using 8 HC-SR04 ultrasonic sensors.

Snap the sensors in its mounting bracket. Carefully bend the 4 pins 90 degrees so they point backwards.

Remove the screws from the IMU mounting plate, add the 'tower' for the sensors. Also, add the support for the second Arduino:

<a></a>
<img src="/images/66.jpg" height=200/>

The mounting brackets with the sensors can be screwed on the tower:

<a></a>
<img src="/images/67.jpg" height=200/>
<img src="/images/68.jpg" height=200/>
<img src="/images/69.jpg" height=200/>

The wires can be guided down trough the holes in the bottom, and to the Arduino

<a></a>
<img src="/images/70.jpg" height=200/>
<img src="/images/71.jpg" height=200/>
<img src="/images/72.jpg" height=200/>

## Finished!

Congratulations, you can now start experimenting! Install the required [Software](/documentation/software_setup.md) to get the vehicle running.

<a></a>
<img src="/images/106.jpg" height=300>

<a></a>
<img src="/images/101.jpg" height=200>
<img src="/images/102.jpg" height=200>
<img src="/images/100.jpg" height=200>
<img src="/images/103.jpg" height=200>




