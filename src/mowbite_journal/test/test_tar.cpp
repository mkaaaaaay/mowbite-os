#include <gtest/gtest.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>

#include "mowbite_journal/tar.h"

namespace fs = std::filesystem;

namespace mowbite_journal {
namespace {

std::string Slurp(const fs::path& path) {
  std::ifstream file(path, std::ios::binary);
  std::ostringstream text;
  text << file.rdbuf();
  return text.str();
}

class TarTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = fs::temp_directory_path() / ("tar_test_" + std::to_string(getpid()) + "_" +
                                        ::testing::UnitTest::GetInstance()->current_test_info()->name());
    fs::remove_all(dir_);
    fs::create_directories(dir_ / "src");
    for (size_t size : {0, 1, 511, 512, 513, 5000}) {
      std::string bytes(size, '\0');
      for (size_t i = 0; i < size; ++i) bytes[i] = static_cast<char>((i * 7919 + size) & 0xFF);
      const fs::path path = dir_ / "src" / (std::to_string(size) + ".bin");
      std::ofstream(path, std::ios::binary) << bytes;
      contents_[path.filename().string()] = bytes;
      entries_.push_back({"run/" + path.filename().string(), path.string(), "", size, 1700000000});
    }
    long_name_ = std::string(80, 'd') + "/" + std::string(90, 'n') + ".txt";
    entries_.push_back({"manifest.json", "", "{\"incident\": 1}\n", 0, 1700000001});
    entries_.push_back({long_name_, "", "deep", 0, 1700000002});
  }
  void TearDown() override {
    fs::remove_all(dir_);
  }

  static std::string Whole(const VirtualTar& tar) {
    std::string out, error;
    EXPECT_TRUE(tar.Read(0, static_cast<size_t>(tar.size()), &out, &error)) << error;
    return out;
  }

  fs::path dir_;
  std::vector<TarEntry> entries_;
  std::map<std::string, std::string> contents_;
  std::string long_name_;
};

TEST_F(TarTest, SizeIsHeadersPaddedDataAndTwoZeroBlocks) {
  const VirtualTar tar(entries_);
  ASSERT_TRUE(tar.error().empty()) << tar.error();
  uint64_t expected = 1024;
  for (const auto& entry : entries_) {
    const uint64_t size = entry.path.empty() ? entry.data.size() : entry.size;
    expected += 512 + (size + 511) / 512 * 512;
  }
  EXPECT_EQ(tar.size(), expected);
  const std::string whole = Whole(tar);
  EXPECT_EQ(whole.size(), expected);
  EXPECT_EQ(whole.substr(whole.size() - 1024), std::string(1024, '\0'));
  EXPECT_EQ(whole, Whole(VirtualTar(entries_)));
}

TEST_F(TarTest, PiecesMakeTheWhole) {
  const VirtualTar tar(entries_);
  const std::string whole = Whole(tar);
  for (size_t piece : {1, 7, 100, 511, 512, 513, 4096, 70000}) {
    std::string joined, out, error;
    for (uint64_t at = 0; at < tar.size(); at += piece) {
      ASSERT_TRUE(tar.Read(at, piece, &out, &error)) << error;
      joined += out;
    }
    EXPECT_EQ(joined, whole) << piece;
  }
  std::string out, error;
  ASSERT_TRUE(tar.Read(tar.size(), 10, &out, &error)) << error;
  EXPECT_TRUE(out.empty());
  EXPECT_FALSE(tar.Read(tar.size() + 1, 10, &out, &error));
}

TEST_F(TarTest, TarListsAndExtractsTheSameBytes) {
  const VirtualTar tar(entries_);
  const fs::path archive = dir_ / "all.tar";
  std::ofstream(archive, std::ios::binary) << Whole(tar);
  const fs::path listing = dir_ / "list.txt";
  ASSERT_EQ(std::system(("tar -tvf " + archive.string() + " > " + listing.string() + " 2>&1").c_str()), 0)
      << Slurp(listing);
  const std::string list = Slurp(listing);
  EXPECT_NE(list.find("-rw-r--r-- 0/0"), std::string::npos) << list;
  EXPECT_NE(list.find(long_name_), std::string::npos) << list;
  const fs::path out = dir_ / "out";
  fs::create_directories(out);
  ASSERT_EQ(std::system(("tar -xf " + archive.string() + " -C " + out.string()).c_str()), 0);
  for (const auto& [name, bytes] : contents_) EXPECT_EQ(Slurp(out / "run" / name), bytes) << name;
  EXPECT_EQ(Slurp(out / "manifest.json"), "{\"incident\": 1}\n");
  EXPECT_EQ(Slurp(out / long_name_), "deep");
  struct stat st {};
  ASSERT_EQ(stat((out / "manifest.json").c_str(), &st), 0);
  EXPECT_EQ(st.st_mtime, 1700000001);
}

TEST_F(TarTest, AFileThatShrankIsAnError) {
  const VirtualTar tar(entries_);
  const fs::path file = dir_ / "src" / "5000.bin";
  fs::resize_file(file, 4000);
  std::string out, error;
  EXPECT_FALSE(tar.Read(0, static_cast<size_t>(tar.size()), &out, &error));
  EXPECT_NE(error.find("5000.bin"), std::string::npos) << error;
  EXPECT_TRUE(out.empty());
  fs::remove(file);
  EXPECT_FALSE(tar.Read(0, static_cast<size_t>(tar.size()), &out, &error));
  EXPECT_NE(error.find("cannot open"), std::string::npos) << error;
}

TEST_F(TarTest, NamesUstarCannotHoldFailTheBuild) {
  for (const std::string& name : {std::string(101, 'x'), std::string("/abs"), std::string("a/../b"), std::string()}) {
    const VirtualTar tar({{name, "", "x", 0, 0}});
    EXPECT_FALSE(tar.error().empty()) << name;
    std::string out, error;
    EXPECT_FALSE(tar.Read(0, 10, &out, &error)) << name;
  }
  const VirtualTar old({{"a", "", "x", 0, -1}});
  EXPECT_FALSE(old.error().empty());
}

}  // namespace
}  // namespace mowbite_journal
