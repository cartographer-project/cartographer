/*
 * Copyright 2018 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_IO_INTERNAL_PBSTREAM_MIGRATE_H_
#define CARTOGRAPHER_IO_INTERNAL_PBSTREAM_MIGRATE_H_

namespace cartographer {
namespace io {

// 'pbstream migrate' entry point. Commandline flags are assumed to be already
// parsed and removed from the remaining arguments.
int pbstream_migrate(int argc, char** argv);

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_INTERNAL_PBSTREAM_MIGRATE_H_
