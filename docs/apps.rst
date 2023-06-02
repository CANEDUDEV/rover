.. _apps:

Applications
============

The Rover's boards come with some reference applications that can be used
to construct a basic radio controlled car. The basic layout consists of a
power board running the battery monitor application, two servo boards,
and one IO board.

The applications rely on the :ref:`can-kingdom` library in order to be configurable over CAN.

* :ref:`battery-monitor`
* :ref:`servo`
* :ref:`sbus`
* :ref:`joystick`

.. _battery-monitor:

Battery monitor
---------------

The power module monitors the battery and the power in the system.

The reference application measures the cell voltage and the current outputs,
and reports them over CAN. It handles the over current protection and provides
a low-voltage cutoff feature.


.. _servo:

Servo
-----

The reference application takes CAN messages and converts them into PWM output.
It also measures the servo voltage and the servo current draw, then reports
them over CAN.

.. doxygenfile:: apps/servo-node/include/app.h
   :project: Rover

.. _sbus:

SBUS receiver
-------------

The SBUS receiver reference application allows for controlling the Rover
using conventional radio controllers from the hobby market. It reads a
radio receiver's SBUS output using an analog port configured for UART
communication, then converts the received data to CAN messages containing
steering, throttle and trim signals.


.. _joystick:

Joystick
--------

The joystick application uses an analog port to read a joystick signal and
convert it into steering or throttle signals to control the Rover.
