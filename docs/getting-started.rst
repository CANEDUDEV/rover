Getting started
===============
After following this sart up guide. you will have a fully functioning radio car using the CAN protocol.
There are several mechanical and electrical components in the set that need to be connected, the specifics about each of them can be found on the starting page.

step 1
-------------

Unbox the car and gather the different components. The car comes with a fully assembled chasit structure including wheels and motor, it looks like the picture below.
The rest of the components need to be mounted by hand which is what the rest of this guide will cover.

.. image:: https://%DOMAIN%/wp-content/uploads/2023/08/20230803_Chasit_pic.jpg


Next up is to start mounting the remainding parts to the car. For a logical overview on the coupling between the different components, see figure below.
Also included is a figure of an example of where these components can be mounted on the actuall car.
(Bilderna kommer förmodligen att uppdateras senare)

.. image:: https://%DOMAIN%/wp-content/uploads/2023/08/20230803_rover_koppling.png

.. image:: https://%DOMAIN%/wp-content/uploads/2023/08/20230803_CAN_koppling.png



step 2
-------------
Connect the radio receiver.

Before assembling a complete CAN system, we will begin by building a simpler version of the car that only needs to run on a radio receiver
with no CAN communication. A big advantage with this approach is that it makes it easy to understand the basic function of the simpler components
in the system and it also makes it easier to later understand how CAN is integrated in the car. To clarify. A fully functioning car can be assembled
without a CAN communication system, the advantage of later integrating CAN is that it allows for simulations of real autonomous vechicles and easy integration
of externall components user might want in their system.

To make a car running only on radio communication, the figure below shows a logical coupling diagram with fewer components.

.. image:: https://%DOMAIN%/wp-content/uploads/2023/08/20230803_Radio_koppling.png

Let us start by mounting the radio receiver to the car. Simply take the reciever and use dual lock bands to connectit to the car.
A video is available below for more detail.

.. raw:: html

    <iframe
        src="https://drive.google.com/file/d/1-3gWJnZZubKGLmURvWDrlAjw7OOtkZgL/preview"
        width="640"
        height="480"
        allowfullscreen>
    </iframe>


step 3
-------------
Connect the ESC and the battery.

The ESC is responsible for controlling the motor and in the case of a system that only uses radio communication it needs to be
connected to the battery (for power), the Radio receiver (in order to get the PWM siganl that is needed by the ESC) and the motor that
it controls. Start by simply mounting the battery in the battery holder (see video below).

.. raw:: html

    <iframe
        src="https://drive.google.com/file/d/1-4EV5S6nP1TR25fQqRmXC2eF8FLqkBos/preview"
        width="640"
        height="480"
        allowfullscreen>
    </iframe>


Next, mount the ESC to the chasit with some dual lock band (see video below).

.. raw:: html

    <iframe
        src="https://drive.google.com/file/d/1-455EEHtMENinTFDLyQ3desgI4G_R7z3/preview"
        width="640"
        height="480"
        allowfullscreen>
    </iframe>

Finally, connect the radio receiver to the ESC and the battery as well as the three wires to the motor. make sure that the wires are connected in
the proper order (if they are reversed, the motor will rotate in the opposite direction as intended). See video below for coupling instructions.

.. raw:: html

    <iframe
        src="https://drive.google.com/file/d/1-5Wft4Lpu3ftsbtzfde2yVR1hEQrILQ0/preview"
        width="640"
        height="480"
        allowfullscreen>
    </iframe>

You should now have a fully functioning car operable through the radio receiver. Feel free to turn it on and take it for a test drive to verify
that it works as intended. See video below.


.. raw:: html

    <iframe
        src="https://drive.google.com/file/d/1-BLLkLxk2fuAjFI8hj9rInPU9z_X1g7q/preview"
        width="640"
        height="480"
        allowfullscreen>
    </iframe>


step 4
-------------
Expanding with a CAN system.

...
