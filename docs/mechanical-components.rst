Mechanical components
==========================
The car comes with a couple of mechanical components that need to be connected. This page serves as 
an overview and goes into some more detail on each of them including data sheets.

Servo
----------

.. Link to manufacturer homepage: `Servo homepage <https://www.agf-rc.com/40kg-high-torque-waterproof-programmable-digital-servo-a73bhlw-p2876572.html>`_

Data sheet: :asset:`Servo data sheet <Servo_DATASHEET.pdf>`

The servo is used for steering the wheels of the car. It does this by receiving a PWM signal from 
the servo board. A picture of the component is shown below.


.. image:: https://drive.google.com/uc?id=1i1bhM2dtNzfHx0FiK96dyRnRAxDNeL5z&export=view

Electronic speed controller (ESC) 
--------------------------------------

Data sheet:  :asset:`ESC data sheet <ESC-manual-DATASHEET.pdf>`

The ESC is a device needed to controll the motor of the car. The motor requires three phase current in order
to work which is why there are three cables comming from the ESC (one for each phase) that go into the motor. 
The device also has a two wire input for the batteries plus and minus connection and an input for the steering
signal (supplied through CAN or the radio module). At the power input to the ESC is a BEC (battery elimination cuircuit) that
transforms the voltage down to 6V/7.4V and 4A as long as it is connected to a battery with 2-4 cells (7.4-14.8V). It does this
in order to ensure that the electronics that are connected to the ESC get a proper and regulated voltage. However, the motors performance
still depends on the battery voltage. For the Rover, a 4 cell battery giving a voltage of 14.8V is necessary in order to deliver sufficient power
and get the car moving as intended. A 2 cell battery for instance can be connected without damaging the system but desired performance will most likely
not be achieved.


.. image:: https://drive.google.com/uc?id=1o7pKLdbDymQj3pnrf2bbd0NFg03X7ut-&export=view


.. image:: https://drive.google.com/uc?id=1NyzT6ajeuSeiZ9h1hH0nKVlNtu6jEymP&export=view


Furthermore, the ESC does have tunable parameters that can be programmed either through the setup button or a program box that can be 
directly connected to the ESC to set the parameters of via a USB cable can be controlled through a computer or PC (see picture below).

.. image:: https://drive.google.com/uc?id=1rGAaD9s1k0qyJvi1A1E2tPW1Wwgdn-u0&export=view
   :alt: Image not accessible.


See video below on how to programm the ESC using the programmable box from Performa to set the parameters. See the link to the data sheet for 
information on the parameters.

.. video:: _assets/ESC_BOX_FINAL.mp4
   :width: 640
   :height: 480


The motor 
--------------------------------------

The motor comes already mounted to the car. The data sheet can be viewed at the link below.

Data sheet:  :asset:`Motor data sheet <Motor_DATASHEET.pdf>`

(TO DO: INSERT A PICTURE OF THE MOTOR)



Radio receiver
-----------------------

Data sheet:  :asset:`Radio tranceiver data sheet <Radio_DATASHEET.pdf>`

The radio receiver can be used to receive radio signals through the air in the same way that radio cars work on the 
hobby market. A picture of the component can be seen below. The red box shows the area where the different channels are 
displayed on the module so it shows which channels are connected to which pins. If the user would like to conect to a 
specific channel, the data sheet can provide information on what the channels do. The important channels for this use case
are channel one which is responsible for steering the servo and channel two for controlling the motor.


.. image:: https://drive.google.com/uc?id=1AaTXbrkXECwe5JzpjtJ0V1GUqWj8VKUd&export=view


If using the radio receiver to steer the car, some usefull things to think about include the following:

1. For best signal reception it is recommended to keep the antennas at a 90 degree angle relative to each other 
   so that both the vertical and horizontal polarization of the signal can be received without loss of information.

2. Keep the receiver protected from water in case it is operated during rain for instance. This can be done by wraping
   it in a plastic bag for example. If water enters the receiver, uncontrolled steering may occur. Since the receiver 
   has some sensitive electronic components it might be good to wrap it in some isolating material that protects in from vibrations.

3. Metallic parts and other electrical components on the car could interfere with the signal. If the signal appears weak it might be
   better to mount it at a more isolted position. 