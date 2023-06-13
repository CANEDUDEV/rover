Mechanical components
==========================
The car comes with a couple of mechanical components that need to be connected. This page serves as 
an overview and goes into some more detail on each of them including data sheets.

Servo
----------

(provide link to data sheet)
`Servo data <https://www.agf-rc.com/40kg-high-torque-waterproof-programmable-digital-servo-a73bhlw-p2876572.html>`_

The servo is used for steering the wheels of the car. It does this by receiving a PWM signal from 
the servo board. A picture of the component is shown below.


.. image:: https://drive.google.com/uc?id=1i1bhM2dtNzfHx0FiK96dyRnRAxDNeL5z&export=view


Electronic speed controller (ESC) 
--------------------------------------

(provide link to data sheet) `ESC data <https://hobbyfactory.fi/en/p60803/performa-p1-combo-1-8-esc-120a-2100kv-motor>`_


The ESC is a device needed to controll the motor of the car. The motor requires three phase current in order
to work which is why there are three cables comming from the ESC (one for each phase) that go into the motor. 
The device also has a two wire input for the batteries plus and minus connection and an input for the steering
signal (supplied through CAN or the radio module). At the power input to the ESC is an BEC (battery eliminatory cuircuit) that
transforms the voltage down to 6V/7.4V and 4A (see datasheet). This means that the controller can be connected to a battery of any
voltage so there is no need for a specific battery. Pictures of the ESC can be seen below.


.. image:: https://drive.google.com/uc?id=1o7pKLdbDymQj3pnrf2bbd0NFg03X7ut-&export=view



.. image:: https://drive.google.com/uc?id=1NyzT6ajeuSeiZ9h1hH0nKVlNtu6jEymP&export=view


Radio receiver
-----------------------

Link to data sheet: `Radio reviever data sheet <http://radiolink.com.cn/doce/UploadFile/ProductFile/RC6GSManual.pdf>`_

The radio receiver can be used to receive radio signals through the air in the same way that radio cars work on the 
hobby market. A picture of the component can be seen below. The red box shows the area where the different channels are 
displayed on the module so it shows which channels are connected to which pins. If the user would like to conect to a 
specific channel, the data sheet can provide information on what the channels do.


.. image:: https://drive.google.com/uc?id=1AaTXbrkXECwe5JzpjtJ0V1GUqWj8VKUd&export=view


If using the radio receiver to steer the car, some usefull things to think about include the following:

1. For best signal reception it is recommended to keep the antennas at a 90 degree angle relative to each other 
   so that both the vertical and horizontal polarization of the signal can be received without loss of information.

2. Keep the receiver protected from water in case it is operated during rain for instance. This can be done by wraping
   it in a plastic bag for example. If water enters the receiver, uncontrolled steering may occur. Since the receiver 
   has some sensitive electronic components it might be good to wrap it in some isolating material that protects in from vibrations.

3. Metallic parts and other electrical components on the car could interfere with the signal. If the signal appears weak it might be
   better to mount it at a more isolted position. 