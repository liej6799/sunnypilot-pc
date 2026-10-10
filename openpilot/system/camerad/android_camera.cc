#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <poll.h>
#include <unistd.h>

#include "cereal/messaging/messaging.h"
#include "common/swaglog.h"
#include "msgq/visionipc/visionipc_server.h"
#include "system/camerad/android_camera_protocol.h"
#include "system/camerad/cameras/nv12_info.h"

namespace {

using android_camera::ConfigMessage;
using android_camera::FrameMessage;
using android_camera::HelloMessage;
using android_camera::MessageType;

constexpr VisionStreamType kRoadStream = 0;
constexpr char kRoadState[] = "narrowRoadCameraState";
constexpr char kDefaultSocketPath[] = "/run/android-camerad/camerad.sock";

std::atomic<bool> do_exit = false;

void signal_handler(int) {
  do_exit = true;
}

bool receive_exact(int fd, void *data, size_t size) {
  ssize_t received;
  do {
    received = recv(fd, data, size, 0);
  } while (received < 0 && errno == EINTR);
  return received == static_cast<ssize_t>(size);
}

bool send_config(int fd, const ConfigMessage &config, const std::vector<int> &fds) {
  std::array<char, CMSG_SPACE(android_camera::kMaxBuffers * sizeof(int))> control = {};
  iovec iov = {.iov_base = const_cast<ConfigMessage *>(&config), .iov_len = sizeof(config)};
  msghdr message = {};
  message.msg_iov = &iov;
  message.msg_iovlen = 1;
  message.msg_control = control.data();
  message.msg_controllen = CMSG_SPACE(fds.size() * sizeof(int));

  cmsghdr *cmsg = CMSG_FIRSTHDR(&message);
  cmsg->cmsg_level = SOL_SOCKET;
  cmsg->cmsg_type = SCM_RIGHTS;
  cmsg->cmsg_len = CMSG_LEN(fds.size() * sizeof(int));
  std::memcpy(CMSG_DATA(cmsg), fds.data(), fds.size() * sizeof(int));

  ssize_t sent;
  do {
    sent = sendmsg(fd, &message, MSG_NOSIGNAL);
  } while (sent < 0 && errno == EINTR);
  return sent == static_cast<ssize_t>(sizeof(config));
}

int create_server_socket(const std::string &path) {
  if (path.size() >= sizeof(sockaddr_un::sun_path)) {
    LOGE("Android camera socket path is too long: %s", path.c_str());
    return -1;
  }

  const std::filesystem::path socket_path(path);
  std::error_code error;
  std::filesystem::create_directories(socket_path.parent_path(), error);
  if (error) {
    LOGE("Failed to create Android camera socket directory: %s", error.message().c_str());
    return -1;
  }

  int fd = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
  if (fd < 0) {
    LOGE("Failed to create Android camera socket: %s", std::strerror(errno));
    return -1;
  }

  sockaddr_un address = {};
  address.sun_family = AF_UNIX;
  std::memcpy(address.sun_path, path.c_str(), path.size() + 1);
  unlink(path.c_str());
  if (bind(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0 || listen(fd, 1) < 0) {
    LOGE("Failed to bind Android camera socket %s: %s", path.c_str(), std::strerror(errno));
    close(fd);
    return -1;
  }
  chmod(path.c_str(), 0666);
  return fd;
}

void publish_camera_state(PubMaster &publisher, const FrameMessage &frame) {
  MessageBuilder message;
  auto state = message.initEvent().initNarrowRoadCameraState();
  state.setFrameId(static_cast<uint32_t>(frame.frame_id));
  state.setFrameIdSensor(static_cast<uint32_t>(frame.frame_id));
  state.setRequestId(static_cast<uint32_t>(frame.frame_id));
  state.setTimestampSof(frame.timestamp_sof);
  state.setTimestampEof(frame.timestamp_eof);
  state.setProcessingTime(std::max(0.0, (nanos_since_boot() - frame.timestamp_eof) / 1e9));
  auto transform = state.initTransform(9);
  transform.set(0, 1.0F);
  transform.set(4, 1.0F);
  transform.set(8, 1.0F);
  publisher.send(kRoadState, message);
}

}  // namespace

int main() {
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  const char *socket_env = std::getenv("ANDROID_CAMERA_SOCKET");
  const std::string socket_path = socket_env != nullptr ? socket_env : kDefaultSocketPath;
  int server_fd = create_server_socket(socket_path);
  if (server_fd < 0) return 1;

  LOGW("Android camerad waiting on %s", socket_path.c_str());
  std::unique_ptr<VisionIpcServer> vipc;
  std::unique_ptr<PubMaster> publisher;
  uint32_t width = 0;
  uint32_t height = 0;
  uint32_t stride = 0;
  uint32_t uv_offset = 0;
  uint32_t buffer_size = 0;
  uint32_t buffer_count = 0;

  while (!do_exit) {
    pollfd server_poll = {.fd = server_fd, .events = POLLIN, .revents = 0};
    int poll_result = poll(&server_poll, 1, 500);
    if (poll_result < 0) {
      if (errno == EINTR) continue;
      LOGE("Android camera socket poll failed: %s", std::strerror(errno));
      break;
    }
    if (poll_result == 0) continue;

    int client_fd = accept4(server_fd, nullptr, nullptr, SOCK_CLOEXEC);
    if (client_fd < 0) {
      if (errno == EINTR) continue;
      LOGE("Android camera accept failed: %s", std::strerror(errno));
      break;
    }

    HelloMessage hello = {};
    if (!receive_exact(client_fd, &hello, sizeof(hello)) ||
        !android_camera::valid_header(hello.header, MessageType::hello, sizeof(hello)) ||
        hello.stream_type != kRoadStream || hello.width < 320 || hello.width > 4096 ||
        hello.height < 240 || hello.height > 3072 || (hello.width & 1) || (hello.height & 1) ||
        hello.buffer_count < 3 || hello.buffer_count > android_camera::kMaxBuffers) {
      LOGE("Rejected invalid Android camera hello");
      close(client_fd);
      continue;
    }

    if (vipc == nullptr) {
      width = hello.width;
      height = hello.height;
      buffer_count = hello.buffer_count;
      uint32_t y_height;
      uint32_t uv_height;
      std::tie(stride, y_height, uv_height, buffer_size) = get_nv12_info(width, height);
      (void)uv_height;
      uv_offset = stride * y_height;

      vipc = std::make_unique<VisionIpcServer>("camerad");
      vipc->create_buffers_with_sizes(kRoadStream, buffer_count, width, height, buffer_size, stride, uv_offset);
      vipc->start_listener();
      publisher = std::make_unique<PubMaster>(std::vector<const char *>{kRoadState});
      LOGW("Android camerad configured %ux%u, stride %u, %u buffers", width, height, stride, buffer_count);
    } else if (hello.width != width || hello.height != height || hello.buffer_count != buffer_count) {
      LOGE("Android camera format changed; restart camerad to renegotiate");
      close(client_fd);
      continue;
    }

    std::vector<int> fds;
    fds.reserve(buffer_count);
    for (uint32_t i = 0; i < buffer_count; ++i) {
      fds.push_back(vipc->get_buffer(kRoadStream, i)->fd);
    }
    ConfigMessage config = {
      .header = android_camera::make_header<ConfigMessage>(MessageType::config),
      .width = width,
      .height = height,
      .stride = stride,
      .uv_offset = uv_offset,
      .buffer_size = buffer_size,
      .buffer_count = buffer_count,
    };
    if (!send_config(client_fd, config, fds)) {
      LOGE("Failed to send VisionIPC buffers to Android: %s", std::strerror(errno));
      close(client_fd);
      continue;
    }

    LOGW("Android camera connected");
    while (!do_exit) {
      pollfd client_poll = {.fd = client_fd, .events = POLLIN, .revents = 0};
      poll_result = poll(&client_poll, 1, 500);
      if (poll_result < 0) {
        if (errno == EINTR) continue;
        break;
      }
      if (poll_result == 0) continue;
      if ((client_poll.revents & POLLIN) == 0) break;

      FrameMessage frame = {};
      if (!receive_exact(client_fd, &frame, sizeof(frame))) break;
      if (!android_camera::valid_header(frame.header, MessageType::frame, sizeof(frame)) || frame.slot >= buffer_count) {
        LOGE("Rejected invalid Android camera frame message");
        break;
      }

      VisionBuf *buffer = vipc->get_buffer(kRoadStream, frame.slot);
      buffer->set_frame_id(frame.frame_id);
      VisionIpcBufExtra extra = {
        .frame_id = static_cast<uint32_t>(frame.frame_id),
        .timestamp_sof = frame.timestamp_sof,
        .timestamp_eof = frame.timestamp_eof,
        .valid = true,
      };
      vipc->send(buffer, &extra);
      publish_camera_state(*publisher, frame);
    }
    LOGW("Android camera disconnected");
    close(client_fd);
  }

  close(server_fd);
  unlink(socket_path.c_str());
  return 0;
}
