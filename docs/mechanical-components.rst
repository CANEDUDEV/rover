Mechanical components
==========================
The car comes with a couple of mechanical components that need to be connected. This page serves as
an overview and goes into some more detail on each of them including data sheets.


Servo
----------

.. Link to manufacturer homepage: `Servo homepage <https://www.agf-rc.com/40kg-high-torque-waterproof-programmable-digital-servo-a73bhlw-p2876572.html>`_

Data sheet: `Servo data sheet <https://%DOMAIN%/wp-content/uploads/2023/08/20230803_Servo_DATASHEET.pdf>`_

The servo is used for steering the wheels of the car. It does this by receiving a PWM signal from
the servo board. A picture of the component is shown below.


.. image:: https://%DOMAIN%/wp-content/uploads/2023/08/20230803_servo_pic.jpg

Electronic speed controller (ESC)
--------------------------------------

Data sheet: `ESC data sheet <https://%DOMAIN%/wp-content/uploads/2023/08/20230803_ESC-manual-DATASHEET.pdf>`_

The ESC is a device needed to control the motor of the car. The motor requires a three-phase current to work which is why three cables are coming from the ESC (one for each phase) that go into the motor. 
The device also has a two-wire input for the batteries plus and minus connection and an input for the steering
signal (supplied through CAN or the radio module). At the power input to the ESC is a BEC (battery elimination circuit) that
transforms the voltage down to 6V/7.4V and 4A as long as it is connected to a battery with 2-4 cells (7.4-14.8V). It does this
to ensure that the electronics that are connected to the ESC get a proper and regulated voltage. However, the performance of the motor
still depends on the battery voltage. For the Rover, a 4-cell battery giving a voltage of 14.8V is necessary to deliver sufficient power
and get the car moving as intended. A 2-cell battery for instance can be connected without damaging the system but desired performance will most likely
not be achieved.


.. image:: https://%DOMAIN%/wp-content/uploads/2023/08/20230803_ESC_picture.jpg

.. image:: https://%DOMAIN%/wp-content/uploads/2023/08/20230803_ESC_picture2.jpg


Furthermore, the ESC does have tunable parameters that can be programmed either through the setup button or a program box that can be 
directly connected to the ESC to set the parameters via a USB cable can be controlled through a computer or PC (see picture below).

.. image:: https://%DOMAIN%/wp-content/uploads/2023/08/20230803_ESC_BOX.jpg


See the video below on how to program the ESC using the programmable box from Performa to set the parameters. See the link to the datasheet for 
information on the parameters.

.. raw:: html

    <iframe
        src="https://drive.google.com/file/d/1-7SQCuRQn6zKAawGKhffSwuqqt5DU_aD/preview"
        width="640"
        height="480"
        allowfullscreen>
    </iframe>


The motor
--------------------------------------

The motor comes already mounted to the car. The data-sheet can be viewed at the link below.

Data sheet: `Motor data sheet <https://%DOMAIN%/wp-content/uploads/2023/08/20230803_Motor_DATASHEET.pdf>`_


.. image:: https://%DOMAIN%/wp-content/uploads/2023/08/Motor_pic-scaled.jpg

Radio receiver
-----------------------

Data sheet: `Radio tranceiver data sheet <https://%DOMAIN%/wp-content/uploads/2023/08/20230803_Radio_DATASHEET.pdf>`_

The radio receiver can be used to receive radio signals through the air in the same way that radio cars work on the 
hobby market. A picture of the component can be seen below. The red box shows the area where the different channels are 
displayed on the module so it shows which channels are connected to which pins. If the user would like to connect to a 
specific channel, the data sheet can provide information on what the channels do. The important channels for this use case
are channel one which is responsible for steering the servo and channel two for controlling the motor.

.. image:: https://%DOMAIN%/wp-content/uploads/2023/08/20230803_radio_rec_pic.jpg

If using the radio receiver to steer the car, some useful things to think about include the following:

1. For best signal reception it is recommended to keep the antennas at a 90-degree angle relative to each other 
   so that both the vertical and horizontal polarization of the signal can be received without loss of information.

2. Keep the receiver protected from water in case it is operated during rain for instance. This can be done by wrapping
   it in a plastic bag for example. If water enters the receiver, uncontrolled steering may occur. Since the receiver 
   has some sensitive electronic components it might be good to wrap it in some isolating material that protects it from vibrations.

3. Metallic parts and other electrical components on the car could interfere with the signal. If the signal appears weak it might be
   better to mount it at a more isolated position. 
