.. _software:

Software
============

The Rover's boards come with some reference applications that can be used
to construct a basic radio controlled car. The basic Rover layout consists of two
servo boards, one to control the steering servo and one to control the ESC, and
one IO board to interface with the radio receiver. This requires two
applications, the :ref:`servo` application and the :ref:`sbus-receiver`
application. Both applications make use of the :ref:`can-kingdom` library.

.. _servo:

Servo
-----

This reference application takes CAN messages and converts them into PWM output.
It also measures the servo voltage and the servo current draw, then reports
them over CAN.

.. _sbus-receiver:

SBUS Receiver
-------------

The SBUS Receiver reference application allows for controlling the Rover
using conventional radio controllers from the hobby market. It reads a
radio receiver's SBUS output using an analog port configured for UART
communication, then converts the received data to CAN messages containing
steering, throttle and trim signals.

.. Commented out these parts for now.
..
    .. _battery-monitor:

    Battery monitor
    ---------------

    The power module monitors the battery and the power in the system.

    The reference application measures the cell voltage and the current outputs,
    and reports them over CAN. It handles the over current protection and provides
    a low-voltage cutoff feature.


.. _can-kingdom:

CAN Kingdom
-----------

CAN Kingdom is a higher-layer protocol based on CAN. This library
provides a hardware-agnostic implementation of the CAN Kingdom library.
Contains implementations of the mayor, the king and the data structures,
as well as an interface for creating a postmaster implementation, which
is hardware dependent.

.. _types:

types.h
```````
.. doxygenfile:: types.h
   :project: Rover


.. _mayor:

mayor.h
```````
.. doxygenfile:: mayor.h
   :project: Rover


.. _king:

king.h
``````
.. doxygenfile:: king.h
   :project: Rover
