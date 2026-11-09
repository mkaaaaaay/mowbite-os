// Stand-in for the media library in tests, loaded with dlopen like the real one. It hands out H.264 made up from
// the settings it was given, or the access units of an Annex B file named in FAKE_MPP_STREAM (in a loop).
// FAKE_MPP_FAIL names one step that fails: create, init, config, header, import, put, packet.
// fake_mpp_report() returns what was set and done as text, fake_mpp_live() the number of objects not yet released.
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "tron_vision/h264_rtp.h"
#include "tron_vision/mpp_abi.h"

using tron_vision::mpp::Api;
using tron_vision::mpp::BufferInfo;
using tron_vision::mpp::Handle;

namespace {

struct Frame {
  uint32_t width = 0;
  uint32_t height = 0;
  uint32_t hor_stride = 0;
  uint32_t ver_stride = 0;
  int format = -1;
  Handle buffer = nullptr;
  int64_t pts = 0;
};

struct Packet {
  std::vector<uint8_t> own;
  uint8_t* pos = nullptr;
  size_t length = 0;
  size_t room = 0;
};

struct Buffer {
  BufferInfo info;
};

struct Config {
  std::map<std::string, int64_t> values;
};

struct Context {
  Api api{};
  std::map<std::string, int64_t> applied;
  int64_t output_wait = 0;
  bool wait_before_init = false;
  bool initialised = false;
  int sei = -1;
  int header_mode = -1;
  uint64_t frames = 0;
  std::vector<uint8_t> pending;
  std::vector<std::vector<uint8_t>> units;
};

std::ostringstream report;
int live = 0;
std::string report_text;

bool Fails(const char* step) {
  const char* fail = std::getenv("FAKE_MPP_FAIL");  // NOLINT(concurrency-mt-unsafe)
  return fail != nullptr && std::strcmp(fail, step) == 0;
}

void StartCode(std::vector<uint8_t>& out) {
  out.insert(out.end(), {0, 0, 0, 1});
}

// SPS and PPS that carry profile, constraint flags and level of the settings in the usual three bytes.
std::vector<uint8_t> Header(const Context& c) {
  const auto value = [&c](const char* name) {
    const auto it = c.applied.find(name);
    return it != c.applied.end() ? it->second : 0;
  };
  // The high half forces a flag, the low half holds its value; set0 is the top bit of the byte.
  const int64_t flags = value("h264:constraint_set");
  uint8_t constraint = 0;
  for (int i = 0; i < 6; i++) {
    if ((flags >> (16 + i) & 1) != 0 && (flags >> i & 1) != 0) constraint |= static_cast<uint8_t>(0x80 >> i);
  }
  std::vector<uint8_t> out;
  StartCode(out);
  out.insert(out.end(), {0x67, static_cast<uint8_t>(value("h264:profile")), constraint,
                         static_cast<uint8_t>(value("h264:level")), 0xaa});
  StartCode(out);
  out.insert(out.end(), {0x68, 0xce, 0x3c, 0x80});
  return out;
}

// Access units of an Annex B file: a unit ends with its slice.
std::vector<std::vector<uint8_t>> ReadUnits(const char* path) {
  std::ifstream in(path, std::ios::binary);
  const std::vector<uint8_t> data((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  std::vector<std::vector<uint8_t>> units;
  std::vector<uint8_t> unit;
  for (const tron_vision::NalUnit& nal : tron_vision::SplitAnnexB(data.data(), data.size())) {
    StartCode(unit);
    unit.insert(unit.end(), nal.data, nal.data + nal.size);
    if (nal.Type() == 1 || nal.Type() == tron_vision::kNalIdr) {
      units.push_back(unit);
      unit.clear();
    }
  }
  return units;
}

int PutFrame(Handle context, Handle frame) {
  auto* c = static_cast<Context*>(context);
  const auto* f = static_cast<const Frame*>(frame);
  if (Fails("put") || f->buffer == nullptr) return -1;
  const BufferInfo& info = static_cast<const Buffer*>(f->buffer)->info;
  if (c->frames < 3) {
    report << "frame " << f->width << "x" << f->height << " stride " << f->hor_stride << "x" << f->ver_stride
           << " format " << f->format << " pts " << f->pts << " fd " << info.fd << " index " << info.index << " size "
           << info.size << " type " << info.type << "\n";
  }
  c->pending.clear();
  if (!c->units.empty()) {
    c->pending = c->units[c->frames % c->units.size()];
  } else {
    const int64_t gop = c->applied.count("rc:gop") != 0 ? c->applied["rc:gop"] : 25;
    const bool idr = c->frames % static_cast<uint64_t>(gop > 0 ? gop : 25) == 0;
    if (idr && c->header_mode == tron_vision::mpp::kHeaderEachIdr) c->pending = Header(*c);
    StartCode(c->pending);
    c->pending.push_back(idr ? 0x65 : 0x41);
    c->pending.insert(c->pending.end(), idr ? 3000 : 300, static_cast<uint8_t>(c->frames + 1));
  }
  c->frames++;
  return 0;
}

int GetPacket(Handle context, Handle* packet) {
  auto* c = static_cast<Context*>(context);
  *packet = nullptr;
  // A wait that was not set to blocking before the init gives the packet too late.
  if (Fails("packet") || !c->wait_before_init || c->output_wait != tron_vision::mpp::kWaitForever) return 0;
  auto* p = new Packet;
  p->own = c->pending;
  p->pos = p->own.data();
  p->length = p->own.size();
  live++;
  *packet = p;
  return 0;
}

int Control(Handle context, int command, void* param) {
  auto* c = static_cast<Context*>(context);
  switch (command) {
    case tron_vision::mpp::kSetOutputTimeout:
      c->output_wait = *static_cast<const int64_t*>(param);
      c->wait_before_init = !c->initialised;
      return 0;
    case tron_vision::mpp::kEncGetConfig: return 0;
    case tron_vision::mpp::kEncSetConfig:
      if (Fails("config")) return -6;
      c->applied = static_cast<const Config*>(param)->values;
      for (const auto& v : c->applied) report << "cfg " << v.first << "=" << v.second << "\n";
      return 0;
    case tron_vision::mpp::kEncSetSei:
      c->sei = *static_cast<const int*>(param);
      report << "sei " << c->sei << "\n";
      return 0;
    case tron_vision::mpp::kEncSetHeaderMode:
      c->header_mode = *static_cast<const int*>(param);
      report << "header_mode " << c->header_mode << "\n";
      return 0;
    case tron_vision::mpp::kEncGetHeader: {
      if (Fails("header")) return -1;
      auto* p = static_cast<Packet*>(param);
      std::vector<uint8_t> header = Header(*c);
      if (!c->units.empty()) {
        // The parameter sets of the file, which open its first unit.
        tron_vision::ParameterSets sets;
        if (!tron_vision::FindParameterSets(c->units[0].data(), c->units[0].size(), sets)) return -1;
        header.clear();
        StartCode(header);
        header.insert(header.end(), sets.sps.begin(), sets.sps.end());
        StartCode(header);
        header.insert(header.end(), sets.pps.begin(), sets.pps.end());
      }
      if (header.size() > p->room) return -1;
      std::memcpy(p->pos, header.data(), header.size());
      p->length = header.size();
      return 0;
    }
    default:
      report << "unknown command " << command << "\n";
      return -1;
  }
}

}  // namespace

extern "C" {

int mpp_create(Handle* context, Api** api) {
  if (Fails("create")) return -1;
  auto* c = new Context;
  c->api.size = sizeof(Api);
  c->api.encode_put_frame = PutFrame;
  c->api.encode_get_packet = GetPacket;
  c->api.control = Control;
  const char* stream = std::getenv("FAKE_MPP_STREAM");  // NOLINT(concurrency-mt-unsafe)
  if (stream != nullptr) c->units = ReadUnits(stream);
  live++;
  *context = c;
  *api = &c->api;
  return 0;
}

int mpp_init(Handle context, int type, int coding) {
  report << "init " << type << " " << coding << "\n";
  if (Fails("init")) return -1002;
  static_cast<Context*>(context)->initialised = true;
  return 0;
}

int mpp_destroy(Handle context) {
  delete static_cast<Context*>(context);
  live--;
  return 0;
}

int mpp_enc_cfg_init(Handle* config) {
  *config = new Config;
  live++;
  return 0;
}

int mpp_enc_cfg_deinit(Handle config) {
  delete static_cast<Config*>(config);
  live--;
  return 0;
}

int mpp_enc_cfg_set_s32(Handle config, const char* name, int32_t value) {
  static_cast<Config*>(config)->values[name] = value;
  return 0;
}

int mpp_enc_cfg_set_u32(Handle config, const char* name, uint32_t value) {
  static_cast<Config*>(config)->values[name] = value;
  return 0;
}

int mpp_frame_init(Handle* frame) {
  *frame = new Frame;
  live++;
  return 0;
}

int mpp_frame_deinit(Handle* frame) {
  delete static_cast<Frame*>(*frame);
  *frame = nullptr;
  live--;
  return 0;
}

void mpp_frame_set_width(Handle frame, uint32_t value) {
  static_cast<Frame*>(frame)->width = value;
}

void mpp_frame_set_height(Handle frame, uint32_t value) {
  static_cast<Frame*>(frame)->height = value;
}

void mpp_frame_set_hor_stride(Handle frame, uint32_t value) {
  static_cast<Frame*>(frame)->hor_stride = value;
}

void mpp_frame_set_ver_stride(Handle frame, uint32_t value) {
  static_cast<Frame*>(frame)->ver_stride = value;
}

void mpp_frame_set_fmt(Handle frame, int format) {
  static_cast<Frame*>(frame)->format = format;
}

void mpp_frame_set_buffer(Handle frame, Handle buffer) {
  static_cast<Frame*>(frame)->buffer = buffer;
}

void mpp_frame_set_pts(Handle frame, long long pts) {
  static_cast<Frame*>(frame)->pts = pts;
}

int mpp_buffer_import_with_tag(Handle, BufferInfo* info, Handle* buffer, const char*, const char*) {
  if (Fails("import")) return -1;
  *buffer = new Buffer{*info};
  report << "import fd " << info->fd << "\n";
  live++;
  return 0;
}

int mpp_buffer_put_with_caller(Handle buffer, const char*) {
  delete static_cast<Buffer*>(buffer);
  live--;
  return 0;
}

int mpp_packet_init(Handle* packet, void* data, size_t size) {
  auto* p = new Packet;
  p->pos = static_cast<uint8_t*>(data);
  p->length = size;
  p->room = size;
  live++;
  *packet = p;
  return 0;
}

int mpp_packet_deinit(Handle* packet) {
  delete static_cast<Packet*>(*packet);
  *packet = nullptr;
  live--;
  return 0;
}

void mpp_packet_set_length(Handle packet, size_t length) {
  static_cast<Packet*>(packet)->length = length;
}

void* mpp_packet_get_pos(Handle packet) {
  return static_cast<Packet*>(packet)->pos;
}

size_t mpp_packet_get_length(Handle packet) {
  return static_cast<Packet*>(packet)->length;
}

// Everything reported since the last call.
const char* fake_mpp_report() {
  report_text = report.str();
  report.str("");
  return report_text.c_str();
}

int fake_mpp_live() {
  return live;
}

}  // extern "C"
