#include <unistd.h>

#include <atomic>
#include <chrono>
#include <fstream>
#include <memory>
#include <mowbite_component/node_options.h>
#include <mowbite_msgs/msg/button_event.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sstream>
#include <string>
#include <thread>

#include "mowbite_watch/attach.h"
#include "tron_buttons/buttons.h"

namespace tron_buttons {

namespace {

using mowbite_msgs::msg::ButtonEvent;

constexpr auto kPeriod = std::chrono::milliseconds(20);

double MonotonicSeconds() {
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

uint8_t Button(tron::Key key) {
  switch (key) {
    case tron::Key::kStart:
      return ButtonEvent::BUTTON_START;
    case tron::Key::kHome:
      return ButtonEvent::BUTTON_HOME;
    case tron::Key::kPower:
      return ButtonEvent::BUTTON_POWER;
  }
  return 0;
}

}  // namespace

// Start, home and on of the Tron on hw/button, from the GPIO levels the kernel shows (gpio_file, a copy of
// /sys/kernel/debug/gpio the rootfs binds for the OS). Its own thread reads every 20 ms, so a slow read stalls nothing
// else. The on button cuts the power when it is let go after 3 s: the file systems are synced when it goes down and
// again when it is held.
class ButtonsNode : public rclcpp::Node {
 public:
  explicit ButtonsNode(const rclcpp::NodeOptions& options)
      : rclcpp::Node("tron_buttons", mowbite_component::StartupOnly(options)),
        hold_s_(declare_parameter("button_hold_s", 0.0)),
        keys_(hold_s_) {
    if (hold_s_ <= 0) {
      RCLCPP_ERROR(get_logger(), "button_hold_s %.2f: no presses without a hold time from the platform file", hold_s_);
      return;
    }
    gpio_file_ = declare_parameter("gpio_file", "/run/mowbite/gpio");
    gpio_[static_cast<std::size_t>(tron::Key::kStart)] = declare_parameter("start_gpio", 52);
    gpio_[static_cast<std::size_t>(tron::Key::kHome)] = declare_parameter("home_gpio", 141);
    gpio_[static_cast<std::size_t>(tron::Key::kPower)] = declare_parameter("power_gpio", 38);
    pub_ = create_publisher<ButtonEvent>("hw/button", rclcpp::QoS(10).reliable());
    watch_ = mowbite_watch::Attach(*this);
    thread_ = std::thread([this] { Loop(); });
  }

  ~ButtonsNode() override {
    running_ = false;
    if (thread_.joinable()) thread_.join();
    if (sync_.joinable()) sync_.join();
  }

 private:
  void Loop() {
    auto next = std::chrono::steady_clock::now();
    while (running_ && rclcpp::ok()) {
      next += kPeriod;
      std::this_thread::sleep_until(next);
      // After a stall the next sample is due now, not a row of them at once.
      next = std::max(next, std::chrono::steady_clock::now());
      Tick();
    }
  }

  void Tick() {
    std::array<std::optional<bool>, tron::kKeyCount> low{};
    std::ifstream file(gpio_file_);
    if (file) {
      std::ostringstream text;
      text << file.rdbuf();
      for (std::size_t i = 0; i < tron::kKeyCount; ++i) {
        low[i] = tron::GpioLow(text.str(), gpio_[i]);
      }
    }
    const bool readable = low[0] && low[1] && low[2];
    if (readable != readable_) {
      readable_ = readable;
      if (readable) {
        RCLCPP_INFO(get_logger(), "buttons readable in %s", gpio_file_.c_str());
      } else {
        RCLCPP_WARN(get_logger(), "buttons not readable in %s: no presses until they are", gpio_file_.c_str());
      }
    }

    for (const tron::KeyEvent& e : keys_.Sample(low, MonotonicSeconds())) {
      ButtonEvent msg;
      msg.stamp = now();
      msg.button = Button(e.key);
      msg.press = e.press == tron::KeyPress::kHeld ? ButtonEvent::PRESS_HELD : ButtonEvent::PRESS_SHORT;
      pub_->publish(msg);
      RCLCPP_INFO(get_logger(), "button %u press %u", msg.button, msg.press);
      if (e.key == tron::Key::kPower && e.press == tron::KeyPress::kHeld) {
        Sync();
      }
    }
    if (keys_.Began(tron::Key::kPower)) {
      Sync();
    }
  }

  // sync() can take seconds; it runs beside the samples, one at a time.
  void Sync() {
    if (syncing_) return;
    if (sync_.joinable()) sync_.join();
    syncing_ = true;
    sync_ = std::thread([this] {
      ::sync();
      syncing_ = false;
    });
  }

  const double hold_s_;
  std::string gpio_file_;
  std::array<int, tron::kKeyCount> gpio_{};
  tron::Keys keys_;
  bool readable_ = true;
  rclcpp::Publisher<ButtonEvent>::SharedPtr pub_;
  std::unique_ptr<mowbite_watch::Attached> watch_;
  std::atomic<bool> running_{true};
  std::atomic<bool> syncing_{false};
  std::thread thread_;
  std::thread sync_;
};

}  // namespace tron_buttons

RCLCPP_COMPONENTS_REGISTER_NODE(tron_buttons::ButtonsNode)
