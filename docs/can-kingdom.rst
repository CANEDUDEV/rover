.. _can-kingdom:

CAN Kingdom
===========

CAN Kingdom is a higher-layer protocol based on CAN. This library
provides a hardware-agnostic implementation of the CAN Kingdom library.
Contains implementations of the mayor, the king and the data structures,
as well as an interface for creating a postmaster implementation, which
is hardware dependent.

* :ref:`types`
* :ref:`mayor`
* :ref:`king`

.. _types:

Types
-----
.. doxygenfile:: types.h
   :project: Rover


.. _mayor:

Mayor
-----
.. doxygenfile:: mayor.h
   :project: Rover


.. _king:

King
----
.. doxygenfile:: king.h
   :project: Rover
