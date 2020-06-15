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

=================================
Migration tool for pbstream files
=================================

With the update of the pbstream serialization format as discussed in
`RFC-0021`_, previously serialized pbstream files are not loadable in
Cartographer 1.0 anymore.

In order to enable users to reuse previously generated pbstream files, we
provide a migration tool which converts pbstreams from Cartographer 0.3 to the
new serialization format used in Cartographer 1.0.

The tool is shipped as part of Cartographer's pbstream tool (`source`_) and once
built can be invoked as follows:::

  cartographer_pbstream migrate old.pbstream new.pbstream

The tool assumes that the first pbstream provided as commandline argument,
follows the serialization format of Cartographer 0.3. The resulting
1.0 pbstream will be saved to the second commandline argument location.

.. _RFC-0021: https://github.com/cartographer-project/rfcs/blob/master/text/0021-serialization-format.md
.. _source: https://github.com/cartographer-project/cartographer/blob/master/cartographer/io/pbstream_main.cc
