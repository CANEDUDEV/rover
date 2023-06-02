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
range can be adjusted using jumpers.

.. _servo-board:

Servo board
-----------

The servo board is a general purpose control module that can be used to
control servos or electronic speed controllers (ESCs). It has a PWM
output with voltage and current measurement, an H-bridge motor driver for
controlling brushed motors, and a sensor input. Additionally, it has an
I2C port and an SPI port.
