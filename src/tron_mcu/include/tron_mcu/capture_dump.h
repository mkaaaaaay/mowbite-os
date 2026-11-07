#ifndef TRON_MCU_CAPTURE_DUMP_H
#define TRON_MCU_CAPTURE_DUMP_H

#include <istream>
#include <ostream>

namespace tron {

// A capture of the bytes the MCU sent, one chunk per line: "<seconds> <hex>". Writes one JSON line per sub packet
// with the time of the chunk that completed its frame (speed, sensor and motors decoded, other modules by number),
// then a summary line with the counts of the parser and of unreadable lines. Returns false if a line was unreadable.
bool DumpCapture(std::istream& in, std::ostream& out);

}  // namespace tron

#endif  // TRON_MCU_CAPTURE_DUMP_H
