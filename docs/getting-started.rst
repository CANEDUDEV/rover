Getting Started
===============

After following this startup guide, you will have a fully functioning
radio-controlled car using the CAN protocol.

Unboxing
--------

Unbox the car and gather the different components. The car comes with a fully
assembled chassis structure, including wheels and a motor. It looks like the
picture below:

.. image:: https://%DOMAIN%/rover/_images/chassis.jpg

..
    The remaining components need to be manually mounted, and this guide will cover
    that process.

    Next, proceed to mount the remaining parts onto the car. For a clear overview
    of how the different components are connected, refer to the figure below.
    Additionally, an example of where these components can be mounted on the actual
    car is provided.

.. .. image:: https://%DOMAIN%/rover/_images/rover_koppling.png

.. .. image:: https://%DOMAIN%/rover/_images/CAN_koppling.png

Connect the Radio Receiver
--------------------------

Before assembling a complete CAN system, we will begin by building a simpler
version of the car that runs solely on a radio receiver without CAN
communication. This approach helps in understanding the basic function of the
simpler components in the system and makes it easier to later integrate CAN
functionality. It's important to note that a fully functional car can be
assembled without a CAN communication system. However, with CAN it's possible
to integrate external components into the system, and the Rover becomes more
like a real vehiclechassis
To create a car that runs solely on radio communication, refer to the logical
connection diagram below, which features fewer components.

.. image:: https://%DOMAIN%/rover/_images/radio-connection.svg

Let's begin by mounting the radio receiver onto the car. Secure the receiver in
place using dual-lock bands. For more detailed instructions, refer to the video
below:

.. raw:: html

    <iframe
        src="https://%DOMAIN%/rover/_videos/radio-mount.mp4"
        width="640"
        height="360"
        allowfullscreen>
    </iframe>

Connect the ESC and the Battery
-------------------------------

The ESC is responsible for controlling the motor. In a system that relies only
on radio communication, it must be connected to the battery (for power), the
radio receiver (to receive the necessary PWM signal), and the motor. Begin by
mounting the battery into the battery holder, as shown in the video below:

.. raw:: html

    <iframe
        src="https://%DOMAIN%/rover/_videos/battery-mount.mp4"
        width="640"
        height="360"
        allowfullscreen>
    </iframe>

Next, secure the ESC to the chassis using dual-lock bands, as demonstrated in
this video:

.. raw:: html

    <iframe
        src="https://%DOMAIN%/rover/_videos/esc-mount.mp4"
        width="640"
        height="360"
        allowfullscreen>
    </iframe>

Finally, connect the radio receiver to the ESC and the battery, along with the
three wires to the motor. Ensure that the wires are connected in the correct
order to avoid the motor rotating in the opposite direction. Refer to the video
below for detailed instructions:

.. raw:: html

    <iframe
        src="https://%DOMAIN%/rover/_videos/radio-connect.mp4"
        width="640"
        height="360"
        allowfullscreen>
    </iframe>

At this point, your car should be fully operational through the radio receiver.
Feel free to turn it on and take it for a test drive to verify its
functionality:

.. raw:: html

    <iframe
        src="https://%DOMAIN%/rover/_videos/test-drive.mp4"
        width="640"
        height="360"
        allowfullscreen>
    </iframe>

..
    Step 4
    ------

    Expanding the System with CAN.

    Now it's time to expand the current system by integrating additional
    electronics to enable a CAN communication system. This involves integrating
    two servo boards (to convert CAN messages to PWM for the servo and ESC) and
    an IO board (to translate the SBUS protocol from the radio receiver to
    CAN).

    Begin by mounting the two servo boards onto the car, as demonstrated in the
    following video:

    .. raw:: html

        <iframe
            src="https://drive.google.com/file/d/1RzTMQfw6jb4LUmNNbu-1rn1ORWKQUw6S/preview"
            width="640"
            height="480"
            allowfullscreen>
        </iframe>

    Next, proceed to mount the IO board, following a similar process as the
    servo boards. Refer to the video for guidance.

    .. servo_board_mounting_FINAL

