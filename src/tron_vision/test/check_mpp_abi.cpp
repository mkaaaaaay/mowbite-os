// Compares include/tron_vision/mpp_abi.h with the headers of the media library. It is built only when the build is
// given their place (-DTRON_VISION_MPP_INCLUDE=<mpp>/inc); it passes by compiling.
#include <cstddef>
#include <type_traits>

#include "mpp_buffer.h"
#include "mpp_frame.h"
#include "mpp_packet.h"
#include "rk_mpi.h"
#include "rk_venc_cfg.h"
#include "tron_vision/mpp_abi.h"

namespace mpp = tron_vision::mpp;

// The type of a library call with every enum taken as int, as the declarations here have it.
template <typename T>
using Plain = std::conditional_t<std::is_enum_v<T>, int, T>;
template <typename R, typename... A>
auto AsDeclared(R (*)(A...)) -> Plain<R> (*)(Plain<A>...);
template <typename Fn, typename Real>
constexpr bool kSame = std::is_same_v<Fn, decltype(AsDeclared(static_cast<Real>(nullptr)))>;
#define CHECK_CALL(alias, call) static_assert(kSame<mpp::alias, decltype(&(call))>, #call)

static_assert(sizeof(MPP_RET) == sizeof(int) && sizeof(MpiCmd) == sizeof(int) && sizeof(MppPollType) == sizeof(int));
static_assert(static_cast<int>(mpp::kContextEncoder) == static_cast<int>(MPP_CTX_ENC));
static_assert(static_cast<int>(mpp::kCodingAvc) == static_cast<int>(MPP_VIDEO_CodingAVC));
static_assert(static_cast<int>(mpp::kFormatYuyv) == static_cast<int>(MPP_FMT_YUV422_YUYV));
static_assert(static_cast<int>(mpp::kBufferExternalDma) == static_cast<int>(MPP_BUFFER_TYPE_EXT_DMA));
static_assert(static_cast<int>(mpp::kRateControlCbr) == static_cast<int>(MPP_ENC_RC_MODE_CBR));
static_assert(static_cast<int>(mpp::kHeaderEachIdr) == static_cast<int>(MPP_ENC_HEADER_MODE_EACH_IDR));
static_assert(static_cast<int>(mpp::kSeiOff) == static_cast<int>(MPP_ENC_SEI_MODE_DISABLE));
static_assert(static_cast<int>(mpp::kWaitForever) == static_cast<int>(MPP_POLL_BLOCK));
static_assert(static_cast<int>(mpp::kSetOutputTimeout) == static_cast<int>(MPP_SET_OUTPUT_TIMEOUT));
static_assert(static_cast<int>(mpp::kEncSetConfig) == static_cast<int>(MPP_ENC_SET_CFG));
static_assert(static_cast<int>(mpp::kEncGetConfig) == static_cast<int>(MPP_ENC_GET_CFG));
static_assert(static_cast<int>(mpp::kEncGetHeader) == static_cast<int>(MPP_ENC_GET_HDR_SYNC));
static_assert(static_cast<int>(mpp::kEncSetSei) == static_cast<int>(MPP_ENC_SET_SEI_CFG));
static_assert(static_cast<int>(mpp::kEncSetHeaderMode) == static_cast<int>(MPP_ENC_SET_HEADER_MODE));

static_assert(sizeof(mpp::Api) == sizeof(MppApi));
static_assert(offsetof(mpp::Api, encode_put_frame) == offsetof(MppApi, encode_put_frame));
static_assert(offsetof(mpp::Api, encode_get_packet) == offsetof(MppApi, encode_get_packet));
static_assert(offsetof(mpp::Api, control) == offsetof(MppApi, control));
static_assert(kSame<decltype(mpp::Api::encode_put_frame), decltype(MppApi::encode_put_frame)>);
static_assert(kSame<decltype(mpp::Api::encode_get_packet), decltype(MppApi::encode_get_packet)>);
static_assert(kSame<decltype(mpp::Api::control), decltype(MppApi::control)>);

static_assert(sizeof(mpp::BufferInfo) == sizeof(MppBufferInfo));
static_assert(offsetof(mpp::BufferInfo, type) == offsetof(MppBufferInfo, type));
static_assert(offsetof(mpp::BufferInfo, size) == offsetof(MppBufferInfo, size));
static_assert(offsetof(mpp::BufferInfo, address) == offsetof(MppBufferInfo, ptr));
static_assert(offsetof(mpp::BufferInfo, handle) == offsetof(MppBufferInfo, hnd));
static_assert(offsetof(mpp::BufferInfo, fd) == offsetof(MppBufferInfo, fd));
static_assert(offsetof(mpp::BufferInfo, index) == offsetof(MppBufferInfo, index));

static_assert(std::is_same_v<decltype(&mpp_create), MPP_RET (*)(MppCtx*, MppApi**)>);
CHECK_CALL(InitFn, mpp_init);
CHECK_CALL(DestroyFn, mpp_destroy);
CHECK_CALL(ConfigInitFn, mpp_enc_cfg_init);
CHECK_CALL(ConfigDeinitFn, mpp_enc_cfg_deinit);
CHECK_CALL(ConfigSetS32Fn, mpp_enc_cfg_set_s32);
CHECK_CALL(ConfigSetU32Fn, mpp_enc_cfg_set_u32);
CHECK_CALL(FrameInitFn, mpp_frame_init);
CHECK_CALL(FrameDeinitFn, mpp_frame_deinit);
CHECK_CALL(FrameSetU32Fn, mpp_frame_set_width);
CHECK_CALL(FrameSetU32Fn, mpp_frame_set_height);
CHECK_CALL(FrameSetU32Fn, mpp_frame_set_hor_stride);
CHECK_CALL(FrameSetU32Fn, mpp_frame_set_ver_stride);
CHECK_CALL(FrameSetFormatFn, mpp_frame_set_fmt);
CHECK_CALL(FrameSetBufferFn, mpp_frame_set_buffer);
CHECK_CALL(FrameSetPtsFn, mpp_frame_set_pts);
static_assert(std::is_same_v<decltype(&mpp_buffer_import_with_tag),
                             MPP_RET (*)(MppBufferGroup, MppBufferInfo*, MppBuffer*, const char*, const char*)>);
CHECK_CALL(BufferPutFn, mpp_buffer_put_with_caller);
CHECK_CALL(PacketInitFn, mpp_packet_init);
CHECK_CALL(PacketDeinitFn, mpp_packet_deinit);
CHECK_CALL(PacketSetLengthFn, mpp_packet_set_length);
CHECK_CALL(PacketGetPosFn, mpp_packet_get_pos);
CHECK_CALL(PacketGetLengthFn, mpp_packet_get_length);
