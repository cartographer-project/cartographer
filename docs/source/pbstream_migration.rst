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

The pbstream serialization format for 3D has changed to include additional
data (histograms) in each submap. Code to load old data by migrating
on-the-fly will be removed soon. Once this happened, users who wish to
migrate old pbstream files can use a migration tool.

The tool is shipped as part of Cartographer's pbstream tool (`source`_) and once
built can be invoked as follows:::

  cartographer_pbstream migrate old.pbstream new.pbstream

The tool assumes 3D data in the old submap format as input and converts it
to the currently used format version.

Migrating pre-1.0 pbstream files
================================

With the update of the pbstream serialization format as discussed in
`RFC-0021`_, previously serialized pbstream files are not loadable in
Cartographer 1.0 anymore.

In order to enable users to reuse previously generated pbstream files,
migration using an older version of the migration tool is necessary.
The current tool does not support this migration anymore. Please use
the version at Git SHA 6c889490e245cc5d9da15023249c6fc7119def3f.

.. _RFC-0021: https://github.com/cartographer-project/rfcs/blob/master/text/0021-serialization-format.md
.. _source: https://github.com/cartographer-project/cartographer/blob/master/cartographer/io/pbstream_main.cc
