#include <fstream>
#include <iostream>

#include "tron_mcu/capture_dump.h"

// tron_mcu_capture <capture>: the capture of the safety measurement as JSON lines on stdout; exit 1 on unreadable lines.
int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "usage: tron_mcu_capture <capture file, one \"<seconds> <hex>\" per line>\n";
    return 2;
  }
  std::ifstream in(argv[1]);
  if (!in) {
    std::cerr << "cannot open " << argv[1] << "\n";
    return 2;
  }
  return tron::DumpCapture(in, std::cout) ? 0 : 1;
}
