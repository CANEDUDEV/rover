.. _boards:

Boards
======

The Rover comes equipped with three distinct boards, the IO board, the power board, and the servo board.
The IO board and the servo board are composed of a carrier board and a CPU board. The CPU boards are the same for
both the IO and servo board but have been programmed for their respective carrier board purposes. 

IO board
--------

The IO board is a general-purpose board for applications utilizing IO devices. It has ports for connecting switches, GPIO devices and
analog devices, as well as an I2C port and an SPI port.

Two reference applications are using the IO module, an SBUS receiver
application and an application utilizing joysticks.

The IO-board can be used in the car to control it through radio waves for example. The circuit schematic can be seen below. Together 
with a picture of the component. Note that the figure shows the IO board equipped with the CPU board on top.

.. raw:: html

   <a href="https://%DOMAIN%/wp-content/uploads/2023/08/20230803_io-node-schematic.pdf">
      <img src="https://%DOMAIN%/wp-content/uploads/2023/08/20230803_io-node-schematic-1.png" alt="Error rendering image. Click this link to go to PDF.">
   </a>

.. image:: https://%DOMAIN%/wp-content/uploads/2023/08/20230803_IO_card_pic.jpg


To connect this circuit to power, let us take a closer look at the power supply shown below. As can be seen, there are six pins in total where 
pins 1 and 4, 2 and 5, and 3 and 6 connect to the same internal bus. Since this is the power supply, pins 1 and 4 are useless and are not needed
except to connect standardized connectors called Dupont connectors that use three pins to the port.
 
.. image::  https://%DOMAIN%/wp-content/uploads/2023/08/20230803_IO_node_zoom_power.png

On the actual circuit board, an input is labeled AN4 which is used to enable communication with the radio transmitter. 

(TODO FIND A GOOD EXPLANATION OF WHY THE AN4 PORT IS USED FOR THIS) 

.. _power-board:

Power board
-----------

The power board's purpose is to monitor the Rover's battery and the power
distribution in the system. It has one power input to connect a battery
and two power outputs, one with unregulated voltage and one with regulated
voltage, configurable through software. It's capable of measuring the
current going out of the power outputs, as well as the battery cell
voltage. It's possible to connect up to two 50 ampere fuses in series to
the module, and it has over current protection. The current measurement
range can be adjusted using jumpers. A picture of the board is shown below.

(TODO READ UP ON THE POWER BOARD AND LOOK AT THE CHIP MORE)

.. raw:: html

   <a href="https://%DOMAIN%/wp-content/uploads/2023/08/20230803_power-node-schematic.pdf">
      <img src="https://%DOMAIN%/wp-content/uploads/2023/08/20230803_power-node-schematic-1.png" alt="Error rendering image. Click this link to go to PDF.">
   </a>


.. image:: https://%DOMAIN%/wp-content/uploads/2023/08/20230803_Battery_card_pic.jpg

.. _servo-board:

Servo board
-----------

The servo board is a general-purpose control module that can be used to
control servos or electronic speed controllers (ESCs). It has a PWM
output with voltage and current measurement, an H-bridge motor driver for
controlling brushed motors, and sensor input. Additionally, it has an
I2C port and an SPI port. A picture is shown below of the two identical cards.

.. image:: https://%DOMAIN%/wp-content/uploads/2023/08/20230803_Servo_modules_pic.jpg


(TODO: THE 2 SERVO NODES USED LOOK IDENTICAL BUT ARE CONNECTED TO EITHER THE ESC OR THE SERVO. THERE NEEDS TO BE A WAY TO DIFFERENTIATE THE TWO 
SO THAT THE CUSTOMER KNOWS WHICH MODULE IS USED FOR WHICH COMPONENT. ONE WAY COULD BE TO PROGRAM THE MODULE SO THAT IT SENSES WHAT INPUT/OUTPUT 
IS REQUIRED AND THEN USES WHATEVER SETTINGS THAT DEVICE NEEDS)

One of the reasons the servo board is a necessity is that many components use PWM signals as input to function.
The car however uses the CAN protocol to send messages to the ECUs in the system. This means that there needs to be a way to 
convert CAN messages to PWM which is the main function of the servo board. The module consists of two separate boards mounted which are
the CPU and servo board. To understand its required inputs and outputs let us start by looking at the servo board circuit schematic which is shown below.

.. raw:: html

   <a href="https://%DOMAIN%/wp-content/uploads/2023/08/20230803_servo-node-schematic.pdf">
      <img src="https://%DOMAIN%/wp-content/uploads/2023/08/20230803_servo-node-schematic-1.png" alt="Error rendering image. Click this link to go to PDF.">
   </a>

There are a lot of details here but the only things we need to worry about are certain ports that serve as either an input or an output.
The schematic has labels associated with a corresponding port on the board. These labels are named with an X followed by a number for instance
port X4 is located at the upper left corner of the schematic and can also be located on the actual circuit board at location X4. A zoomed-in
figure is shown below. This gives an idea of how to connect the ports by looking at the schematic, for instance, we can see that ports 1 and 4, 2 and 5
and 3 and 6 lead to the same connection suggesting a parallel circuit that can be used to connect other components over the same voltage or that require
the same signal. This port is used to supply power to the servo board and since there are 6 pins, other components can be connected through this node that 
require power from the battery. Note also that pins 1 and 4 have dead ends meaning that they do not perform any particular function other than ensuring that
standard 3-pin connectors can be connected to the port. The circuit is designed to handle an input voltage between 4.5-28 V and up to 3A, there exists
circuitry to prevent overcurrent.


.. image:: https://%DOMAIN%/wp-content/uploads/2023/08/20230803_servo_power_zoom.png

Similarly, a zoomed-in version of the servo module output at port X3 can be seen in the figure below. The way to see this port as an output and not an input 
is that the servo modules function is to convert CAN messages to PWM and since the labels of the ports are ground, SERVO VOUT and SERVO PWM suggesting this port 
needs to be connected to an external component requiring PWM like the ESC or the servo. There are some other ports as well but they are not required for a basic 
start-up of the car.


.. image:: https://%DOMAIN%/wp-content/uploads/2023/08/20230803_Servo_powerOut_zoom.png

It is now time to look at the upper board of the servo module which is the CPU board, the circuit schematic is shown below. 

.. raw:: html
    
   <a href="https://%DOMAIN%/wp-content/uploads/2023/08/20230803_cpu-node-schematic.pdf">
      <img src="https://%DOMAIN%/wp-content/uploads/2023/08/20230803_cpu-node-schematic-1.png" alt="Error rendering image. Click this link to go to PDF.">
   </a>

The important part of this schematic is port X5 which can be seen zoomed in below. The inputs required for this port are the CAN messages that need to be converted 
to PWM signals. As can be seen, there are 12 pins in total, and half of them are connected to the same buses similar to the power input for the bottom board. Note that 
the pins are connected to two different inputs in the sense that the odd numbers 1,3,5,7,9,11 lead to the commonly used CAN protocol input while the even numbers 2,4,6,8,10,12 
lead to an alternative input using the CAN FD protocol. For a basic CAN setup, use the odd numbers to use the standard CAN protocol where pins 1,3,5 and 7,9,11 can be used. The
CAN FD protocol is an alternative way of using CAN that can handle larger data rates but it is not necessary. Other functions can also be utilized on the servo board
such as connecting external sensors for instance but this is not required.


.. image:: https://%DOMAIN%/wp-content/uploads/2023/08/20230803_servo_canInput_zoom.png
