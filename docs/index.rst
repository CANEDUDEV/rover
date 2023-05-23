.. Rover documentation master file, created by
   sphinx-quickstart on Mon May 15 13:56:20 2023.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to the Rover's documentation!
=====================================

.. toctree::
   :maxdepth: 2
   :caption: Contents:


Indices and tables
==================

* :ref:`genindex`
* :ref:`search`
* :ref:`can-kingdom`

  * :ref:`types`
  * :ref:`mayor`
  * :ref:`king`

* :ref:`servo`

.. _can-kingdom:

common/can-kingdom
==================
Provides an implementation of the CAN Kingdom library. Contains implementations
of the mayor, the king and the data structures, as well as an interface for
creating a postmaster implementation.

.. _types:

types.h
-------
.. doxygenfile:: types.h
   :project: Rover

.. _mayor:

mayor.h
-------
.. doxygenfile:: mayor.h
   :project: Rover

.. _king:

king.h
------
.. doxygenfile:: king.h
   :project: Rover

.. _servo:

servo-node
==========
Contains the code for the servo board hardware and a generic servo application.

.. doxygenfile:: apps/servo-node/include/app.h
   :project: Rover
