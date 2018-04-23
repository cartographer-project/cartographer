.. Copyright 2018 The Cartographer Authors

.. Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

..      http://www.apache.org/licenses/LICENSE-2.0

.. Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

===========
Strict Mode
===========

Concept
=======

Cartographer follows a straight-forward input data philosophy that allows both users of the software to ensure that mapping results generated from provided input data are reproducible and explainable:


*Cartographer enforces certain invariants that constrain the input data it accepts, e.g. strict time-ordering is assumed within each sensor data type.
These invariants are enforced in the source code by CHECKs.
If an invariant is violated Cartographer fails fast and hard with a crash.*


Cartographer is now used in more and more production environments.
In idealized environment where the integrator has control over all parts of a system's software we still believe that the above philosophy is the right one for building a robust software system.
In todays e.g. robotics systems however often a mixture of open-source and closed-source software runs in concert.
As such the system integrator has limited control over parts of the system and handling faulty software states becomes a more involved, often domain-specific problem.
In such environments the 'cartographer_strict' command-line flag can be used to disable Cartographer's default behavior of enforcing input invariants and allow a higher-level monitoring component to make the ultimate decision about whether to shut down the system.

List of affected CHECKs
=======================

The following lists the input data invariants that Cartographer relaxes when the 'cartographer_strict' command-line flag is set to false.

