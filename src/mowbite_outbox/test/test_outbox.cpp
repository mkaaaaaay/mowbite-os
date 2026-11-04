#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include "mowbite_outbox/outbox.h"

using namespace mowbite_outbox;
using Clock = std::chrono::steady_clock;

TEST(Ring, KeepsTheOrder) {
  Ring<int> ring(4, Full::kDropOldest);
  for (int i = 0; i < 3; i++) EXPECT_TRUE(ring.Push(i));
  EXPECT_EQ(ring.TakeAll(), (std::vector<int>{0, 1, 2}));
  EXPECT_TRUE(ring.TakeAll().empty());
  EXPECT_EQ(ring.TakeDropped(), 0u);
}

TEST(Ring, FullDropsTheOldest) {
  Ring<int> ring(3, Full::kDropOldest);
  for (int i = 0; i < 3; i++) EXPECT_TRUE(ring.Push(i));
  EXPECT_FALSE(ring.Push(3));
  EXPECT_FALSE(ring.Push(4));
  EXPECT_EQ(ring.TakeAll(), (std::vector<int>{2, 3, 4}));
  EXPECT_EQ(ring.TakeDropped(), 2u);
  EXPECT_EQ(ring.TakeDropped(), 0u);
  // After a wrap it starts in order again.
  for (int i = 10; i < 17; i++) ring.Push(i);
  EXPECT_EQ(ring.TakeAll(), (std::vector<int>{14, 15, 16}));
  ring.Push(20);
  EXPECT_EQ(ring.TakeAll(), (std::vector<int>{20}));
}

TEST(Ring, FullDropsTheNewest) {
  Ring<int> ring(2, Full::kDropNewest);
  EXPECT_TRUE(ring.Push(1));
  EXPECT_TRUE(ring.Push(2));
  EXPECT_FALSE(ring.Push(3));
  EXPECT_EQ(ring.TakeAll(), (std::vector<int>{1, 2}));
  EXPECT_EQ(ring.TakeDropped(), 1u);
}

TEST(Latest, KeepsOnlyTheNewest) {
  Latest<std::string> latest;
  EXPECT_FALSE(latest.Take());
  latest.Set("a");
  latest.Set("b");
  latest.Set("c");
  EXPECT_EQ(latest.Take(), std::optional<std::string>("c"));
  EXPECT_FALSE(latest.Take());
  EXPECT_EQ(latest.TakeReplaced(), 2u);
  EXPECT_EQ(latest.TakeReplaced(), 0u);
}

TEST(Counter, CountsUntilTaken) {
  Counter counter;
  counter.Add();
  counter.Add(4);
  EXPECT_EQ(counter.Take(), 5u);
  EXPECT_EQ(counter.Take(), 0u);
}

TEST(Peak, KeepsTheLargest) {
  Peak peak;
  EXPECT_EQ(peak.Take().count, 0u);
  peak.Add(0.06);
  peak.Add(0.31);
  peak.Add(0.07);
  auto taken = peak.Take();
  EXPECT_EQ(taken.count, 3u);
  EXPECT_DOUBLE_EQ(taken.max, 0.31);
  EXPECT_EQ(peak.Take().count, 0u);
}

TEST(LogQueue, KeepsTheFirstLinesAndWakes) {
  Wakeup wakeup;
  LogQueue log(&wakeup);
  for (size_t i = 0; i < LogQueue::kCapacity + 10; i++) log.Warn(Format("line %zu", i));
  EXPECT_TRUE(wakeup.WaitFor(std::chrono::milliseconds(0)));
  auto lines = log.TakeAll();
  ASSERT_EQ(lines.size(), LogQueue::kCapacity);
  EXPECT_EQ(lines.front().text, "line 0");
  EXPECT_EQ(lines.front().level, LogLine::Level::kWarn);
  EXPECT_EQ(log.TakeDropped(), 10u);
}

TEST(Format, Truncates) {
  EXPECT_EQ(Format("%d %s", 7, "x"), "7 x");
  EXPECT_EQ(Format("%s", std::string(600, 'a').c_str()).size(), 511u);
}

TEST(Throttle, OnePerPeriod) {
  Throttle throttle(10.0);
  EXPECT_TRUE(throttle.Allow(100.0));
  EXPECT_FALSE(throttle.Allow(105.0));
  EXPECT_TRUE(throttle.Allow(110.0));
}

TEST(Wakeup, TimesOutWithoutNotice) {
  Wakeup wakeup;
  const auto start = Clock::now();
  EXPECT_FALSE(wakeup.WaitFor(std::chrono::milliseconds(30)));
  EXPECT_GE(Clock::now() - start, std::chrono::milliseconds(25));
  wakeup.Notify();
  EXPECT_TRUE(wakeup.WaitFor(std::chrono::seconds(5)));
  wakeup.Stop();
  EXPECT_FALSE(wakeup.WaitFor(std::chrono::seconds(5)));
  EXPECT_TRUE(wakeup.stopped());
}

// The producer goes on at its pace while the consumer hangs (a publish blocked in DDS): 1000 entries in under 5 ms,
// the rings keep the newest, the log the first lines, and nothing is lost before the rings are full.
TEST(Outbox, ProducerNeverWaitsForAHangingConsumer) {
  Wakeup wakeup;
  Ring<int> stream(128, Full::kDropOldest);
  Latest<int> latest;
  LogQueue log(&wakeup);
  std::atomic<bool> hanging{false};
  std::atomic<bool> done{false};
  std::thread consumer([&] {
    while (!done) {
      if (!wakeup.WaitFor(std::chrono::milliseconds(100)) && !done) continue;
      stream.TakeAll();
      log.TakeAll();
      // Hangs right after taking, the way a blocked publish would.
      hanging = true;
      std::this_thread::sleep_for(std::chrono::seconds(2));
      hanging = false;
    }
  });
  stream.Push(-1);
  wakeup.Notify();
  while (!hanging) std::this_thread::sleep_for(std::chrono::milliseconds(1));

  const auto start = Clock::now();
  for (int i = 0; i < 1000; i++) {
    stream.Push(i);
    latest.Set(i);
    log.Info(Format("entry %d", i));
    wakeup.Notify();
  }
  const auto took = Clock::now() - start;
  EXPECT_LT(took, std::chrono::milliseconds(5)) << std::chrono::duration<double>(took).count() << " s";
  EXPECT_TRUE(hanging);

  auto entries = stream.TakeAll();
  ASSERT_EQ(entries.size(), 128u);
  EXPECT_EQ(entries.front(), 872);
  EXPECT_EQ(entries.back(), 999);
  EXPECT_EQ(stream.TakeDropped(), 872u);
  EXPECT_EQ(latest.Take(), std::optional<int>(999));
  auto lines = log.TakeAll();
  ASSERT_EQ(lines.size(), LogQueue::kCapacity);
  EXPECT_EQ(lines.front().text, "entry 0");
  done = true;
  wakeup.Stop();
  consumer.join();
}
