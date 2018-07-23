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
