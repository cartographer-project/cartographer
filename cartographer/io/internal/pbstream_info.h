#ifndef CARTOGRAPHER_IO_INTERNAL_PBSTREAM_INFO_H_
#define CARTOGRAPHER_IO_INTERNAL_PBSTREAM_INFO_H_

namespace cartographer {
namespace io {

// info subtool for pbstream swiss army knife. The command line arguments are
// assumed to be parsed and removed from the remaining arguments already.
int pbstream_info(int argc, char* argv[]);

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_INTERNAL_PBSTREAM_INFO_H_
