Getting started
===============
After following this sart up guide. you will have a fully functioning radio car using the CAN protocol.
There are several mechanical and electrical components in the set that need to be connected, the specifics about each of them can be found on the starting page.

step 1
-------------

Unbox the car and gather the different components. The car comes with a fully assembled chasit structure including wheels and motor, it looks like the picture below.
The rest of the components need to be mounted by hand which is what the rest of this guide will cover.

.. image:: https://drive.google.com/uc?id=1xftWTRuA5N00URit7-X2NXzx66F7GZ9u
   :alt: Image not accessible.


Next up is to start mounting the remainding parts to the car. For a logical overview on the coupling between the different components, see figure below.
Also included is a figure of an example of where these components can be mounted on the actuall car.
(Bilderna kommer förmodligen att uppdateras senare)

.. image:: https://drive.google.com/uc?id=1R5yPv4IsMnoWovTxBfCYj0nees4u8FNm&export=view
   :alt: Image not accessible.



.. image:: https://drive.google.com/uc?id=1JN_zs2vcc7DXGEU_Un5GlROAwJGZLdTx&export=view
   :alt: Image not accessible.


step 2
-------------
Connect the radio receiver.

Before assembling a complete CAN system, we will begin by building a simpler version of the car that only needs to run on a radio receiver
with no CAN communication. A big advantage with this approach is that it makes it easy to understand the basic function of the simpler components
in the system and it also makes it easier to later understand how CAN is integrated in the car. To clarify. A fully functioning car can be assembled
without a CAN communication system, the advantage of later integrating CAN is that it allows for simulations of real autonomous vechicles and easy integration
of externall components user might want in their system.

To make a car running only on radio communication, the figure below shows a logical coupling diagram with fewer components.

.. image:: https://drive.google.com/uc?id=13QfoT1vhVVftApaJ50B8O1bqxpXLPOYq&export=view
   :alt: Image not accessible.
   


Let us start by mounting the radio receiver to the car. Simply take the reciever and use dual lock bands to connectit to the car.
A video is available below for more detail.

.. video:: _assets/Radio_FINAL.mp4
   :width: 640
   :height: 480

step 3
-------------
Connect the ESC and the battery.

The ESC is responsible for controlling the motor and in the case of a system that only uses radio communication it needs to be
connected to the battery (for power), the Radio receiver (in order to get the PWM siganl that is needed by the ESC) and the motor that
it controls. Start by simply mounting the battery in the battery holder (see video below).

.. video:: _assets/Battery_Mounting_FINAL.mp4
   :width: 640
   :height: 480

Next, mount the ESC to the chasit with some dual lock band (see video below).

.. video:: _assets/ESC_montering_FINAL.mp4
   :width: 640
   :height: 480

Finally, connect the radio receiver to the ESC and the battery as well as the three wires to the motor. make sure that the wires are connected in
the proper order (if they are reversed, the motor will rotate in the opposite direction as intended). See video below for coupling instructions.

.. video:: _assets/Radio_full_mounting_FINAL.mp4
   :width: 640
   :height: 480

You should now have a fully functioning car operable through the radio receiver. Feel free to turn it on and take it for a test drive to verify
that it works as intended. See video below.


.. video:: _assets/Radio_test_drive_FINAL.mp4
   :width: 640
   :height: 480
  

step 4
-------------
Expanding with a CAN system.

...
