// Based on EventHistory.h from xbot_monitoring (OpenMower), MIT License, Copyright (c) 2022 Clemens Elflein.
// See LICENSE.xbot_monitoring.
#pragma once

#include <algorithm>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

namespace mowbite_bridge {

using json = nlohmann::ordered_json;

class EventHistory {
 public:
  EventHistory() = default;

  // Keeps at most max_days day files and max_bytes on disk, the oldest days go first. Today is always kept, but stops
  // being written once it alone reaches max_bytes.
  void init(const std::string& base_dir, size_t max_days = 60, uintmax_t max_bytes = 20000000) {
    base_dir_ = base_dir;
    max_days_ = std::max<size_t>(max_days, 1);
    max_bytes_ = max_bytes;
    std::filesystem::create_directories(base_dir_);
    current_date_ = todayString();
    today_buffer_ = readLines(base_dir_ + "/" + current_date_ + ".jsonl");
    dir_index_ = scanDir();
    prune();
  }

  // Appends a raw JSON string. Persists to today's file.
  // On day rollover the in-memory buffer is cleared and a new file is started.
  void add(const std::string& json_payload) {
    std::lock_guard<std::mutex> lk(mutex_);
    std::string date = todayString();
    if (date != current_date_) {
      today_buffer_.clear();
      current_date_ = date;
      prune();
    }
    today_buffer_.push_back(json_payload);
    appendToDisk(json_payload);
    // The file of today may only have been created now, e.g. by the first event after a boot.
    if (std::find(dir_index_.begin(), dir_index_.end(), current_date_) == dir_index_.end()) {
      dir_index_.insert(dir_index_.begin(), current_date_);
      prune();
    }
  }

  // Returns today's events as a JSON array of objects (parsed on read).
  json getAll() const {
    std::lock_guard<std::mutex> lk(mutex_);
    if (current_date_ != todayString()) return json::array();
    return parseBuffer(today_buffer_);
  }

  // Returns events for the given date as a JSON array of objects.
  // If date matches today or is empty, served from memory (empty if day rolled over).
  // Otherwise read fresh from disk. Missing file -> empty array.
  json getAll(const std::string& date) const {
    std::lock_guard<std::mutex> lk(mutex_);
    const std::string today = todayString();
    if (date.empty() || date == today) {
      if (current_date_ != today) return json::array();
      return parseBuffer(today_buffer_);
    }
    return parseBuffer(readLines(base_dir_ + "/" + date + ".jsonl"));
  }

  // Deletes the file for the given date. If date is empty/nullopt, deletes ALL files.
  // Deleting today also clears the in-memory buffer.
  // Returns {"deleted": N}.
  json deleteHistory(const std::optional<std::string>& date) {
    std::lock_guard<std::mutex> lk(mutex_);
    int deleted = 0;
    for (auto it = dir_index_.begin(); it != dir_index_.end();) {
      if (date.has_value() && *it != *date) {
        ++it;
        continue;
      }
      std::error_code ec;
      if (std::filesystem::remove(base_dir_ + "/" + *it + ".jsonl", ec)) {
        if (*it == current_date_) today_buffer_.clear();
        ++deleted;
        it = dir_index_.erase(it);
      } else {
        ++it;
      }
    }
    return {{"deleted", deleted}};
  }

  // Returns available dates newest-first: ["20260612", "20260611", ...]
  // Only dates for which a file exists are returned.
  json listHistories() const {
    std::lock_guard<std::mutex> lk(mutex_);
    json arr = json::array();
    for (const auto& d : dir_index_) arr.push_back(d);
    return arr;
  }

 private:
  static std::string todayString() {
    std::time_t t = std::time(nullptr);
    std::tm tm_result{};
    localtime_r(&t, &tm_result);
    char buf[9];
    std::strftime(buf, sizeof(buf), "%Y%m%d", &tm_result);
    return buf;
  }

  // Scans base_dir_ once and returns dates newest-first for all *.jsonl files
  // whose stem is exactly 8 characters (YYYYMMDD).
  std::vector<std::string> scanDir() const {
    std::vector<std::string> dates;
    std::error_code ec;
    for (const auto& entry : std::filesystem::directory_iterator(base_dir_, ec)) {
      if (!entry.is_regular_file()) continue;
      const std::string name = entry.path().filename().string();
      if (name.size() != 14 || name.substr(8) != ".jsonl") continue;
      dates.push_back(name.substr(0, 8));
    }
    std::sort(dates.begin(), dates.end(), std::greater<std::string>());
    return dates;
  }

  static std::vector<std::string> readLines(const std::string& path) {
    std::vector<std::string> lines;
    std::ifstream f(path);
    if (!f.is_open()) return lines;
    std::string line;
    while (std::getline(f, line)) {
      if (!line.empty()) lines.push_back(std::move(line));
    }
    return lines;
  }

  static json parseBuffer(const std::vector<std::string>& buf) {
    json arr = json::array();
    for (const auto& line : buf) {
      try {
        arr.push_back(json::parse(line));
      } catch (...) {
        // skip malformed lines
      }
    }
    return arr;
  }

  // Drops the oldest day files beyond max_days_ or max_bytes_, never today's.
  void prune() {
    std::error_code ec;
    uintmax_t total = 0;
    for (const auto& d : dir_index_) {
      const uintmax_t size = std::filesystem::file_size(base_dir_ + "/" + d + ".jsonl", ec);
      if (!ec) total += size;
    }
    while (!dir_index_.empty() && dir_index_.back() != current_date_ &&
           (dir_index_.size() > max_days_ || total > max_bytes_)) {
      const std::string path = base_dir_ + "/" + dir_index_.back() + ".jsonl";
      const uintmax_t size = std::filesystem::file_size(path, ec);
      if (!ec) total -= std::min(total, size);
      std::filesystem::remove(path, ec);
      dir_index_.pop_back();
    }
  }

  void appendToDisk(const std::string& json_payload) {
    const std::string path = base_dir_ + "/" + current_date_ + ".jsonl";
    std::error_code ec;
    const uintmax_t size = std::filesystem::file_size(path, ec);
    if (!ec && size + json_payload.size() + 1 > max_bytes_) return;
    std::ofstream f(path, std::ios::app);
    if (f.is_open()) {
      f << json_payload << "\n";
    }
  }

  std::string base_dir_;
  std::string current_date_;
  std::vector<std::string> today_buffer_;
  std::vector<std::string> dir_index_;  // dates newest-first, populated at init, updated on day rollover
  size_t max_days_ = 60;
  uintmax_t max_bytes_ = 20000000;
  mutable std::mutex mutex_;
};

}  // namespace mowbite_bridge
