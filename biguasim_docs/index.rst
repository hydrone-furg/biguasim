Welcome to BiguaSim's documentation!
==============================================
.. image:: _static/images/logo-light.svg
   :alt: BiguaSim hero
   :class: main-logo

BiguaSim is a high-fidelity simulator developed by `Nautec <https://nautec.furg.br/>`_
at `Federal University of Rio Grande <https://www.furg.br>`_ .

Features
""""""""
#. 3+ rich worlds with various infrastructure for generating data or testing underwater algorithms
#. Complete with common underwater sensors including DVL, IMU, optical camera, various sonar, depth sensor, and more
#. Highly and easily configurable sensors and missions
#. Multi-agent missions, including optical and acoustic communications
#. Novel sonar simulation framework for simulating imaging, profiling, sidescan, and echosounder sonars
#. Imaging sonar implementation includes realistic noise modeling for small sim-2-real gap
#. Easy installation and simple, OpenAI Gym-like Python interface
#. High performance - simulation speeds of up to 2x real time are possible. Performance penalty only for what you need
#. Run headless or watch your agents learn
#. Linux and Windows support

Attribution and Relevent Publications
"""""""""""""""""""""""""""""""""""""
If you use BiguaSim in your research, please cite the following publications depending on the features you use as outlined below:

General BiguaSim use:
::
      
   @inproceedings{,
      author = {E. Potokar and S. Ashford and M. Kaess and J. Mangelson},
      title = {Holo{O}cean: An Underwater Robotics Simulator},
      booktitle = {Proc. IEEE Intl. Conf. on Robotics and Automation, ICRA},
      address = {Philadelphia, PA, USA},
      month = may,
      year = {2022}
   }

Simulation of Sonar (Imaging, Profiling, Sidescan) sensors:
::
   
   @inproceedings{Potokar22iros,
      author = {E. Potokar and K. Lay and K. Norman and D. Benham and T. Neilsen and M. Kaess and J. Mangelson},
      title = {Holo{O}cean: Realistic Sonar Simulation},
      booktitle = {Proc. IEEE/RSJ Intl. Conf. Intelligent Robots and Systems, IROS},
      address = {Kyoto, Japan},
      month = {Oct},
      year = {2022}
   }

.. toctree::
   :maxdepth: 2
   :caption: BiguaSim Documentation

   usage/installation
   usage/getting-started
   usage/usage
   packages/packages
   agents/agents
   sensors/sensors
   examples/examples
   develop/develop
   changelog/changelog

.. toctree::
   :maxdepth: 3
   :caption: API Documentation

   biguasim/index
   biguasim/agents
   biguasim/environments
   biguasim/spaces
   biguasim/commands
   biguasim/biguasimclient
   biguasim/packagemanager
   biguasim/sensors
   biguasim/lcm
   biguasim/dynamics
   biguasim/shmem
   biguasim/util
   biguasim/exceptions
   biguasim/weather


Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
