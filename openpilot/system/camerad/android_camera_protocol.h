#pragma once

#include <cstddef>
#include <cstdint>

namespace android_camera {

constexpr uint32_t kMagic = 0x4d414341;  // "ACAM" on little-endian hosts.
constexpr uint16_t kVersion = 1;
constexpr uint32_t kMaxBuffers = 18;

enum class MessageType : uint16_t {
  hello = 1,
  config = 2,
  frame = 3,
};

struct MessageHeader {
  uint32_t magic;
  uint16_t version;
  MessageType type;
  uint32_t size;
};

struct HelloMessage {
  MessageHeader header;
  uint32_t stream_type;
  uint32_t width;
  uint32_t height;
  uint32_t buffer_count;
};

struct ConfigMessage {
  MessageHeader header;
  uint32_t width;
  uint32_t height;
  uint32_t stride;
  uint32_t uv_offset;
  uint32_t buffer_size;
  uint32_t buffer_count;
};

struct FrameMessage {
  MessageHeader header;
  uint32_t slot;
  uint32_t reserved;
  uint64_t frame_id;
  uint64_t timestamp_sof;
  uint64_t timestamp_eof;
};

template <typename T>
constexpr MessageHeader make_header(MessageType type) {
  return {kMagic, kVersion, type, sizeof(T)};
}

inline bool valid_header(const MessageHeader &header, MessageType type, size_t size) {
  return header.magic == kMagic && header.version == kVersion && header.type == type && header.size == size;
}

static_assert(sizeof(MessageHeader) == 12);
static_assert(sizeof(HelloMessage) == 28);
static_assert(sizeof(ConfigMessage) == 36);
static_assert(sizeof(FrameMessage) == 48);

}  // namespace android_camera
