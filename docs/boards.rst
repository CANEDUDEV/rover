.. _boards:

Boards
======

The Rover comes equipped with three distinct boards, the IO board, the power board, and the servo board.

.. _io-board:

IO board
--------

The IO board is a general purpose board for applications utilizing IO devices. It has ports for connecting switches, GPIO devices and
analog devices, as well as an I2C port and an SPI port.

There are two reference applications using the IO module, an SBUS receiver
application and an application utilizing joysticks.

The IO board can be used in the car in order to control it through radio waves for example. The cuircuit schematic can be seen below. Together 
with a picture of the component.


.. image:: https://drive.google.com/uc?id=17DWDsLP0Xeu4Ggn_4yOkuYAWnSaQ67ZW&export=view


.. image:: https://drive.google.com/uc?id=16qAcI3Skac4TXF5cuPQdf_xhGgIl0u74&export=view


To connect this cuircuit to power, lets take a closer look at the power supply shown below. As can be seen, there are six pins in total where 
pins 1 and 4, 2 and 5, and 3 and 6 connect to the same internal buss. Since this is the power supply, pins 1 and 4 are useless and are not needed
except in order to connect standardized connectors that use three pins to the port. 


.. image::  https://drive.google.com/uc?id=1CAkphXae9Z4Jiqd-OZ0QH1jIyn8IE5Vs&export=view

On the actuall circuit board an input is labeled AN4 which is used in order to connect the radio receiver to the IO board and enable communication
through radio waves. 

(TODO FIND A GOOD EXPLAINATION WHY THE AN4 PORT IS USED FOR THIS) 

.. _power-board:

Power board
-----------

The power board's purpose is to monitor the Rover's battery and the power
distribution in the system. It has one power input to connect a battery,
two power outputs, one with unregulated voltage and one with regulated
voltage, configurable through software. It's capable of measuring the
current going out of the power outputs, as well as the battery cell
voltage. It's possible to connect up to two 50 ampere fuses in series to
the module, and it has over current protection. The current measurement
range can be adjusted using jumpers. A picture of the board is shown below.

(TODO READ UP ON THE POWER BOARD AND LOOK AT THE CHIP MORE)

.. image:: https://drive.google.com/uc?id=1aMMiNOg-zXjTbssufNN4PVn4c55QyVX2&export=view

.. image:: https://drive.google.com/uc?id=1ppscLL-h5qN1HY1KANv7ItaTqR2T6n8E&export=view

.. image:: https://drive.google.com/uc?id=1a0scsM_BJK-uydBBcU3gfE2dhSi2TM16&export=view

.. _servo-board:

Servo board
-----------

The servo board is a general purpose control module that can be used to
control servos or electronic speed controllers (ESCs). It has a PWM
output with voltage and current measurement, an H-bridge motor driver for
controlling brushed motors, and a sensor input. Additionally, it has an
I2C port and an SPI port. A picture is shown below of the two identical cards.


.. image:: https://drive.google.com/uc?id=1-pZD7aUfIZKTenKfPUc8oO0w2pOSG0Xh&export=view

(TODO: THE 2 SERVO NODES USED LOOK IDENTICAL BUT ARE CONNECTED TO EITHER THE ESC OR THE SERVO. THERE NEEDS TO BE A WAY TO DIFFERENTIATE THE TWO 
SO THAT THE CUSTOMER KNOWS WHICH MODULE IS USED FOR WHICH COMPONENT. ONE WAY COULD BE TO PROGRAM THE MODULE SO THAT IT SENSES WHAT INPUT/OUTPUT 
IS REQUIRED AND THEN USES WHATEVER SETTINGS THAT DEVICE NEEDS)

The reason the servo board is a necessity is that many components use PWM signals as their power input in order to function.
The car however, uses the CAN protocol to send messages to the ECU in the system. This means that there needs to be a way to 
convert CAN messages to PWM which is the main function of the servo board. The board consist of two separate boards mounted on top of
each other. In order to understand its required inputs and outputs lets start by looking at the bottom cuircuit schematic which is shown below.

.. image:: https://drive.google.com/uc?id=1xJVz0zyg0_9SwY-mj2XtGtCLsHdnCGBd&export=view

There is a lot of details here but the only things we need to worry about are certain ports that serve as either an input or an output.
The schematic has labels associated with a corresponding port on the board. These lables are named with an X followed by a number for instance
port X4 is locateed at the upper left corner of the schematic and can also be located on the actuall circuit board at location X4. A zoomed in
figure is shown below. This gives an idea of how to connect the ports by looking at the schematic, for instance we can see that port 1 and 4, 2 and 5
and 3 and 6 lead to the same connection suggesting a parallel cuircuit that can be used to connect other components over the same voltage or that require
the same signal. This port is used to supply power to the servo board and since there are 6 pins, other components can be connected through this node that 
require power from the battery. Note also that pins 1 and 4 have dead ends meaning that they do not perform any particullar function other than ensuring that
standard 3 pin connectors can be connected to the port. The cuircuit is designed to hanlde an input voltage between 4.5-28 V and up to 3A, there exists
circuitry to prevent overcurrent.


.. image:: https://drive.google.com/uc?id=1ZqAU5RI88aOhI8UW7ejvbJ8Ne1IDvn4W&export=view

Similarly, a zoomed in version of the servo module output at port X3 can be seen in the figure below. The way to see this port as an output and not an input 
is that the servo modules function is to convert CAN messages to PWM and since the labels of the ports are ground, SERVO VOUT and SERVO PWM suggesting this port 
needs to be connected to an externall compoment requiring PWM like the ESC or the servo. There are some other ports as well but they are not required for a basic 
start up of the car.


.. image:: https://drive.google.com/uc?id=1rhju29fAI_bFpUasBdFTEFbKNXEiv8FF&export=view

It is now time to look at the upper board of the servo module, the cuircuit schematic is shown below. 


.. image:: https://drive.google.com/uc?id=125xBB4CAaq6LtNojvvScjoD41ukFBTsA&export=view

The important part of this schematic is port X5 which can be seen zoomed in below. The input required for this port are the CAN messages that needs to be converted 
to PWM signals. As can be seen, there are 12 pins in total where half of them are connected to the same wires similar to the power input for the bottom board. Note that 
the pins are connected to two different inputs in the sense that the odd numbers 1,3,5,7,9,11 lead to the common used CAN protocol input while the even numbers 2,4,6,8,10,12 
lead to an alternative input using the CAN FD protocol. For a basic CAN settup, use the odd numbers to use the standard CAN protocol where pin 1,3,5 and 7,9,11 can be used. The
CAN FD protocol is an alterantive way of using CAN that can handle larger data rates but it is not necessary. There are also other functions that can be utilized on the servo board
such as connecting externall sensors for instance but this is not required.


.. image:: https://drive.google.com/uc?id=15yBvgxj3UjFjfKRW26AWZTY5uM9hiwrB&export=view