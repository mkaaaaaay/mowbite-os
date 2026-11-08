#ifndef MOWBITE_LIGHT_OUTPUT_H
#define MOWBITE_LIGHT_OUTPUT_H

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace mowbite_light {

// Where the SPI bytes of a frame go.
class Output {
 public:
  virtual ~Output() = default;
  // false and why when the frame did not go out.
  virtual bool Write(const std::vector<uint8_t>& frame, std::string* error) = 0;
};

// spidev in mode 0 with 8 bits at 8 MHz, one SPI_IOC_MESSAGE per frame; nullptr and why when it cannot be opened.
std::unique_ptr<Output> OpenSpidev(const std::string& device, std::string* error);

// Every frame replaces the file whole, for the simulation.
std::unique_ptr<Output> OpenFile(const std::string& path);

}  // namespace mowbite_light

#endif  // MOWBITE_LIGHT_OUTPUT_H
