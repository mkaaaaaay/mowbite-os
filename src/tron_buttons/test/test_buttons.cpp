#include <gtest/gtest.h>

#include <optional>
#include <string>
#include <vector>

#include "tron_buttons/buttons.h"

using namespace tron;

namespace {

// Lines as the Tron shows them (24.11.), start held.
const std::string kDebugfs =
    "gpiochip1: GPIOs 32-63, parent: platform/fec20000.gpio, gpio1:\n"
    " gpio-38  (                    |button              ) in  hi \n"
    " gpio-52  (                    |button1             ) in  lo IRQ \n"
    "gpiochip4: GPIOs 128-159, parent: platform/fec50000.gpio, gpio4:\n"
    " gpio-141 (                    |button2             ) in  hi \n";

constexpr double kStep = 0.02;
constexpr double kHold = 3.0;

using Levels = std::array<std::optional<bool>, kKeyCount>;

Levels Down(std::optional<Key> key) {
  Levels levels{false, false, false};
  if (key) {
    levels[static_cast<std::size_t>(*key)] = true;
  }
  return levels;
}

// Samples every 20 ms like the node and records the events.
class KeysTest : public ::testing::Test {
 protected:
  void Run(const Levels& levels, double seconds) {
    const double end = now_ + seconds;
    while (now_ < end - 1e-9) {
      now_ += kStep;
      for (const KeyEvent& e : keys_.Sample(levels, now_)) {
        events_.push_back(e);
      }
    }
  }

  Keys keys_{kHold};
  double now_ = 0;
  std::vector<KeyEvent> events_;
};

bool Is(const KeyEvent& e, Key key, KeyPress press) { return e.key == key && e.press == press; }

}  // namespace

TEST(GpioLow, ReadsTheLevelOfEachLine) {
  EXPECT_EQ(GpioLow(kDebugfs, 52), std::optional<bool>(true));
  EXPECT_EQ(GpioLow(kDebugfs, 141), std::optional<bool>(false));
  EXPECT_EQ(GpioLow(kDebugfs, 38), std::optional<bool>(false));
}

TEST(GpioLow, MissingOrUnreadableLineIsNone) {
  EXPECT_EQ(GpioLow(kDebugfs, 5), std::nullopt);
  EXPECT_EQ(GpioLow(kDebugfs, 3), std::nullopt);
  EXPECT_EQ(GpioLow(" gpio-52  (                    |button1             ) in  ?? \n", 52), std::nullopt);
  EXPECT_EQ(GpioLow("", 52), std::nullopt);
}

TEST_F(KeysTest, ShortPress) {
  Run(Down(std::nullopt), 0.2);
  Run(Down(Key::kHome), 0.3);
  EXPECT_TRUE(events_.empty());
  Run(Down(std::nullopt), 0.1);
  ASSERT_EQ(events_.size(), 1u);
  EXPECT_TRUE(Is(events_[0], Key::kHome, KeyPress::kShort));
}

TEST_F(KeysTest, HeldComesWhileStillDownAndNothingOnRelease) {
  Run(Down(std::nullopt), 0.2);
  Run(Down(Key::kStart), 2.9);
  EXPECT_TRUE(events_.empty());
  Run(Down(Key::kStart), 0.2);
  ASSERT_EQ(events_.size(), 1u);
  EXPECT_TRUE(Is(events_[0], Key::kStart, KeyPress::kHeld));
  Run(Down(Key::kStart), 5.0);
  Run(Down(std::nullopt), 0.2);
  EXPECT_EQ(events_.size(), 1u);
}

TEST_F(KeysTest, BounceShorterThanDebounceIsNoPress) {
  Run(Down(std::nullopt), 0.2);
  Run(Down(Key::kHome), kStep);
  Run(Down(std::nullopt), 0.2);
  EXPECT_TRUE(events_.empty());
  EXPECT_FALSE(keys_.Began(Key::kHome));
}

TEST_F(KeysTest, BounceDuringAPressDoesNotSplitIt) {
  Run(Down(std::nullopt), 0.2);
  Run(Down(Key::kStart), 2.0);
  Run(Down(std::nullopt), kStep);
  Run(Down(Key::kStart), 1.1);
  ASSERT_EQ(events_.size(), 1u);
  EXPECT_TRUE(Is(events_[0], Key::kStart, KeyPress::kHeld));
}

TEST_F(KeysTest, ButtonDownAtTheStartGivesNothingUntilLetGo) {
  Run(Down(Key::kStart), 5.0);
  Run(Down(std::nullopt), 0.2);
  EXPECT_TRUE(events_.empty());
  Run(Down(Key::kStart), 0.2);
  Run(Down(std::nullopt), 0.2);
  ASSERT_EQ(events_.size(), 1u);
  EXPECT_TRUE(Is(events_[0], Key::kStart, KeyPress::kShort));
}

TEST_F(KeysTest, GapInTheSamplesLosesThePress) {
  Run(Down(std::nullopt), 0.2);
  Run(Down(Key::kStart), 1.0);
  now_ += 1.0;
  Run(Down(Key::kStart), 2.5);
  Run(Down(std::nullopt), 0.2);
  EXPECT_TRUE(events_.empty());
}

TEST_F(KeysTest, UnreadableLineLosesThePress) {
  Run(Down(std::nullopt), 0.2);
  Run(Down(Key::kHome), 0.2);
  Run(Levels{false, std::nullopt, false}, kStep);
  Run(Down(Key::kHome), 0.2);
  Run(Down(std::nullopt), 0.2);
  EXPECT_TRUE(events_.empty());
  Run(Down(Key::kHome), 0.2);
  Run(Down(std::nullopt), 0.2);
  ASSERT_EQ(events_.size(), 1u);
  EXPECT_TRUE(Is(events_[0], Key::kHome, KeyPress::kShort));
}

TEST_F(KeysTest, TwoButtonsAtOnceGiveNothing) {
  Run(Down(std::nullopt), 0.2);
  Run(Down(Key::kStart), 0.5);
  Run(Levels{true, true, false}, 3.0);
  Run(Down(Key::kStart), 1.0);
  Run(Down(std::nullopt), 0.2);
  EXPECT_TRUE(events_.empty());
}

TEST_F(KeysTest, PowerBeganComesAtTheStartOfThePress) {
  Run(Down(std::nullopt), 0.2);
  EXPECT_FALSE(keys_.Began(Key::kPower));
  Run(Down(Key::kPower), 0.1);
  EXPECT_TRUE(keys_.Began(Key::kPower));
  EXPECT_FALSE(keys_.Began(Key::kPower));
  Run(Down(Key::kPower), 3.0);
  ASSERT_EQ(events_.size(), 1u);
  EXPECT_TRUE(Is(events_[0], Key::kPower, KeyPress::kHeld));
}

TEST_F(KeysTest, HeldCountsFromTheFirstSampleDown) {
  Run(Down(std::nullopt), 0.2);
  // The samples down run from the first one at hold_s - kStep after it: not yet held.
  Run(Down(Key::kPower), kHold);
  EXPECT_TRUE(events_.empty());
  Run(Down(Key::kPower), kStep);
  ASSERT_EQ(events_.size(), 1u);
}
