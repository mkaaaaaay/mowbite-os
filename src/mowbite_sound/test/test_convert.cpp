#include <gtest/gtest.h>
#include <stdlib.h>

#include <filesystem>
#include <fstream>
#include <iterator>

#include "mowbite_sound/convert.h"
#include "mowbite_sound/wav.h"

using namespace mowbite_sound;
namespace fs = std::filesystem;

namespace {

struct TempDir {
  TempDir() {
    char pattern[] = "/tmp/mowbite_sound_XXXXXX";
    path = ::mkdtemp(pattern);
  }
  ~TempDir() { fs::remove_all(path); }
  std::string path;
};

std::string ReadAll(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  return std::string(std::istreambuf_iterator<char>(in), {});
}

void WriteAll(const std::string& path, const std::string& bytes) {
  std::ofstream(path, std::ios::binary) << bytes;
}

int Peak(const Wav& wav) {
  int peak = 0;
  for (const int16_t sample : wav.samples) peak = std::max(peak, std::abs(static_cast<int>(sample)));
  return peak;
}

}  // namespace

TEST(Wav, RoundTripsAndRefusesOtherFormats) {
  const Wav wav{24000, {0, 1000, -1000, 32767, -32768}};
  std::string error;
  const auto back = DecodeWav(EncodeWav(wav), &error);
  ASSERT_TRUE(back) << error;
  EXPECT_EQ(back->rate, 24000u);
  EXPECT_EQ(back->samples, wav.samples);

  std::string stereo = EncodeWav(wav);
  stereo[22] = 2;
  EXPECT_FALSE(DecodeWav(stereo, &error));
  EXPECT_FALSE(DecodeWav(EncodeWav({4000, {0}}), &error));
  EXPECT_FALSE(DecodeWav("RIFF....WAVE", &error));
  std::string cut = EncodeWav(wav);
  cut.resize(cut.size() - 2);
  EXPECT_FALSE(DecodeWav(cut, &error));
}

TEST(DecodeMp3, MonoAtItsRateAfterTheTag) {
  std::string error;
  const auto wav = DecodeMp3(ReadAll(TEST_DATA "/tone.mp3"), &error);
  ASSERT_TRUE(wav) << error;
  EXPECT_EQ(wav->rate, 24000u);
  EXPECT_NEAR(wav->Seconds(), 1.0, 0.15);
  EXPECT_GT(Peak(*wav), 1000);
}

TEST(DecodeMp3, MixesTwoChannelsToOne) {
  std::string error;
  const auto wav = DecodeMp3(ReadAll(TEST_DATA "/stereo.mp3"), &error);
  ASSERT_TRUE(wav) << error;
  EXPECT_EQ(wav->rate, 44100u);
  EXPECT_NEAR(wav->Seconds(), 0.5, 0.1);
  // One channel carries the tone, the other is silent: the mix has half its height.
  const auto mono = DecodeMp3(ReadAll(TEST_DATA "/tone.mp3"), &error);
  ASSERT_TRUE(mono);
  EXPECT_LT(Peak(*wav), Peak(*mono) * 3 / 4);
  EXPECT_GT(Peak(*wav), Peak(*mono) / 4);
}

TEST(DecodeMp3, RefusesWhatIsNoMp3) {
  std::string error;
  EXPECT_FALSE(DecodeMp3(std::string(5000, 'x'), &error));
  EXPECT_FALSE(DecodeMp3("", &error));
  EXPECT_FALSE(DecodeMp3(EncodeWav({24000, std::vector<int16_t>(24000, 0)}), &error));
}

TEST(Converter, WritesAWavOfTheMp3) {
  TempDir dir;
  std::string error;
  ASSERT_TRUE(RunProgram({CONVERTER, TEST_DATA "/tone.mp3", dir.path + "/tone.wav"}, 30, &error)) << error;
  const auto wav = DecodeWav(ReadAll(dir.path + "/tone.wav"), &error);
  ASSERT_TRUE(wav) << error;
  EXPECT_EQ(wav->rate, 24000u);
  EXPECT_NEAR(wav->Seconds(), 1.0, 0.15);
}

TEST(Converter, TellsWhyAFileFails) {
  TempDir dir;
  WriteAll(dir.path + "/broken.mp3", std::string(3000, 'x'));
  std::string error;
  EXPECT_FALSE(RunProgram({CONVERTER, dir.path + "/broken.mp3", dir.path + "/out.wav"}, 30, &error));
  EXPECT_NE(error.find("exited with 1: no MP3 frames"), std::string::npos) << error;

  WriteAll(dir.path + "/large.mp3", std::string((2 << 20) + 1, '\0'));
  EXPECT_FALSE(RunProgram({CONVERTER, dir.path + "/large.mp3", dir.path + "/out.wav"}, 30, &error));
  EXPECT_NE(error.find("larger than 2 MB"), std::string::npos) << error;

  EXPECT_FALSE(RunProgram({CONVERTER, dir.path + "/missing.mp3", dir.path + "/out.wav"}, 30, &error));
  EXPECT_NE(error.find("cannot open"), std::string::npos) << error;
}

TEST(RunProgram, KillsWhatTakesTooLongAndNamesAMissingProgram) {
  std::string error;
  EXPECT_FALSE(RunProgram({"/bin/sleep", "5"}, 0.3, &error));
  EXPECT_NE(error.find("took longer than"), std::string::npos) << error;
  EXPECT_FALSE(RunProgram({"/nonexistent/convert"}, 1, &error));
  EXPECT_NE(error.find("cannot start"), std::string::npos) << error;
  EXPECT_TRUE(RunProgram({"/bin/true"}, 5, &error)) << error;
}
