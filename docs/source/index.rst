.. Copyright 2016 The Cartographer Authors

.. Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

..      http://www.apache.org/licenses/LICENSE-2.0

.. Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

============
Cartographer
============

.. toctree::
   :maxdepth: 2
   :hidden:

`Cartographer`_ is a system that provides real-time simultaneous localization
and mapping `SLAM`_ across multiple platforms and sensor configurations.

.. _Cartographer: https://github.com/googlecartographer/cartographer
.. _SLAM: http://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping

Getting Started with ROS
========================

ROS integration is provided by the `Cartographer ROS repository`_. You will find
complete documentation for using Cartographer with ROS at the
`Cartographer ROS Read the Docs site`_.

.. _Cartographer ROS repository: https://github.com/googlecartographer/cartographer_ros
.. _Cartographer ROS Read the Docs site: https://google-cartographer-ros.readthedocs.io

Getting Started without ROS
===========================

On Ubuntu 14.04 (Trusty):

.. literalinclude:: ../../scripts/install_debs.sh
  :language: bash
  :linenos:
  :lines: 20-

.. literalinclude:: ../../scripts/install_ceres.sh
  :language: bash
  :linenos:
  :lines: 20-

.. literalinclude:: ../../scripts/install_cartographer.sh
  :language: bash
  :linenos:
  :lines: 20-
