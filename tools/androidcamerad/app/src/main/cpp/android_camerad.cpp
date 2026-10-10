#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <android/log.h>
#include <camera/NdkCameraDevice.h>
#include <camera/NdkCameraManager.h>
#include <jni.h>
#include <linux/dma-buf.h>
#include <media/NdkImage.h>
#include <media/NdkImageReader.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/un.h>
#include <unistd.h>

#include "android_camera_protocol.h"

namespace {

#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, "SunnypilotCamera", __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, "SunnypilotCamera", __VA_ARGS__)

using android_camera::ConfigMessage;
using android_camera::FrameMessage;
using android_camera::HelloMessage;
using android_camera::MessageType;

constexpr uint32_t kRoadStream = 0;
constexpr int64_t kFrameIntervalNs = 50'000'000;

struct SharedBuffer {
  int fd = -1;
  uint8_t *address = nullptr;
  size_t mapping_size = 0;
};

class CameraBridge {
 public:
  CameraBridge(std::string socket_path, std::string requested_camera_id, int requested_width,
               int requested_height, int requested_buffer_count)
      : socket_path_(std::move(socket_path)), requested_camera_id_(std::move(requested_camera_id)),
        requested_width_(requested_width), requested_height_(requested_height),
        requested_buffer_count_(std::clamp(requested_buffer_count, 3, static_cast<int>(android_camera::kMaxBuffers))) {}

  ~CameraBridge() {
    stop();
  }

  bool start() {
    manager_ = ACameraManager_create();
    if (manager_ == nullptr || !select_camera_and_size()) return false;

    media_status_t media_status = AImageReader_newWithUsage(
        width_, height_, AIMAGE_FORMAT_YUV_420_888, AHARDWAREBUFFER_USAGE_CPU_READ_OFTEN, 4, &reader_);
    if (media_status != AMEDIA_OK) {
      LOGE("AImageReader_newWithUsage failed: %d", media_status);
      return false;
    }

    AImageReader_ImageListener image_listener = {.context = this, .onImageAvailable = on_image_available};
    if (AImageReader_setImageListener(reader_, &image_listener) != AMEDIA_OK ||
        AImageReader_getWindow(reader_, &window_) != AMEDIA_OK) {
      LOGE("Unable to configure ImageReader listener/window");
      return false;
    }

    running_ = true;
    connector_ = std::thread(&CameraBridge::connection_loop, this);

    ACameraDevice_StateCallbacks device_callbacks = {
      .context = this,
      .onDisconnected = on_camera_disconnected,
      .onError = on_camera_error,
    };
    camera_status_t status = ACameraManager_openCamera(manager_, camera_id_.c_str(), &device_callbacks, &device_);
    if (status != ACAMERA_OK) {
      LOGE("ACameraManager_openCamera(%s) failed: %d", camera_id_.c_str(), status);
      stop();
      return false;
    }

    if (!create_capture_session()) {
      stop();
      return false;
    }
    LOGI("Capturing camera %s at %dx%d, publishing at 20 FPS", camera_id_.c_str(), width_, height_);
    return true;
  }

  void stop() {
    if (!running_.exchange(false) && manager_ == nullptr) return;

    {
      std::lock_guard<std::mutex> lock(buffer_mutex_);
      disconnect_locked();
    }
    if (connector_.joinable()) connector_.join();

    if (session_ != nullptr) {
      ACameraCaptureSession_stopRepeating(session_);
      ACameraCaptureSession_close(session_);
      session_ = nullptr;
    }
    if (request_ != nullptr) {
      ACaptureRequest_free(request_);
      request_ = nullptr;
    }
    if (target_ != nullptr) {
      ACameraOutputTarget_free(target_);
      target_ = nullptr;
    }
    if (outputs_ != nullptr && output_ != nullptr) {
      ACaptureSessionOutputContainer_remove(outputs_, output_);
    }
    if (output_ != nullptr) {
      ACaptureSessionOutput_free(output_);
      output_ = nullptr;
    }
    if (outputs_ != nullptr) {
      ACaptureSessionOutputContainer_free(outputs_);
      outputs_ = nullptr;
    }
    if (device_ != nullptr) {
      ACameraDevice_close(device_);
      device_ = nullptr;
    }
    if (reader_ != nullptr) {
      AImageReader_delete(reader_);
      reader_ = nullptr;
      window_ = nullptr;
    }
    if (manager_ != nullptr) {
      ACameraManager_delete(manager_);
      manager_ = nullptr;
    }
  }

 private:
  static void on_camera_disconnected(void *context, ACameraDevice *) {
    LOGE("Camera disconnected");
    static_cast<CameraBridge *>(context)->running_ = false;
  }

  static void on_camera_error(void *context, ACameraDevice *, int error) {
    LOGE("Camera device error: %d", error);
    static_cast<CameraBridge *>(context)->running_ = false;
  }

  static void on_session_closed(void *, ACameraCaptureSession *) {}
  static void on_session_ready(void *, ACameraCaptureSession *) {}
  static void on_session_active(void *, ACameraCaptureSession *) {}

  static void on_image_available(void *context, AImageReader *reader) {
    static_cast<CameraBridge *>(context)->handle_image(reader);
  }

  bool select_camera_and_size() {
    ACameraIdList *camera_ids = nullptr;
    if (ACameraManager_getCameraIdList(manager_, &camera_ids) != ACAMERA_OK || camera_ids == nullptr) {
      LOGE("Unable to enumerate cameras");
      return false;
    }

    if (!requested_camera_id_.empty()) {
      camera_id_ = requested_camera_id_;
    } else {
      for (int i = 0; i < camera_ids->numCameras; ++i) {
        ACameraMetadata *metadata = nullptr;
        if (ACameraManager_getCameraCharacteristics(manager_, camera_ids->cameraIds[i], &metadata) != ACAMERA_OK) continue;
        ACameraMetadata_const_entry facing = {};
        bool is_back = ACameraMetadata_getConstEntry(metadata, ACAMERA_LENS_FACING, &facing) == ACAMERA_OK &&
                       facing.count > 0 && facing.data.u8[0] == ACAMERA_LENS_FACING_BACK;
        ACameraMetadata_free(metadata);
        if (is_back) {
          camera_id_ = camera_ids->cameraIds[i];
          break;
        }
      }
      if (camera_id_.empty() && camera_ids->numCameras > 0) camera_id_ = camera_ids->cameraIds[0];
    }
    ACameraManager_deleteCameraIdList(camera_ids);
    if (camera_id_.empty()) {
      LOGE("No camera is available");
      return false;
    }

    ACameraMetadata *metadata = nullptr;
    if (ACameraManager_getCameraCharacteristics(manager_, camera_id_.c_str(), &metadata) != ACAMERA_OK) {
      LOGE("Unable to read characteristics for camera %s", camera_id_.c_str());
      return false;
    }
    ACameraMetadata_const_entry configurations = {};
    if (ACameraMetadata_getConstEntry(metadata, ACAMERA_SCALER_AVAILABLE_STREAM_CONFIGURATIONS, &configurations) != ACAMERA_OK) {
      ACameraMetadata_free(metadata);
      LOGE("Camera has no stream configuration metadata");
      return false;
    }

    double best_score = std::numeric_limits<double>::infinity();
    for (uint32_t i = 0; i + 3 < configurations.count; i += 4) {
      int format = configurations.data.i32[i];
      int candidate_width = configurations.data.i32[i + 1];
      int candidate_height = configurations.data.i32[i + 2];
      int input = configurations.data.i32[i + 3];
      if (format != AIMAGE_FORMAT_YUV_420_888 || input != 0 || (candidate_width & 1) || (candidate_height & 1)) continue;

      double requested_aspect = static_cast<double>(requested_width_) / requested_height_;
      double candidate_aspect = static_cast<double>(candidate_width) / candidate_height;
      double aspect_error = std::abs(std::log(candidate_aspect / requested_aspect));
      double area_error = std::abs(std::log(static_cast<double>(candidate_width) * candidate_height /
                                            (static_cast<double>(requested_width_) * requested_height_)));
      double score = aspect_error * 4.0 + area_error;
      if (score < best_score) {
        best_score = score;
        width_ = candidate_width;
        height_ = candidate_height;
      }
    }
    ACameraMetadata_free(metadata);

    if (width_ == 0 || height_ == 0) {
      LOGE("Camera %s has no YUV_420_888 output", camera_id_.c_str());
      return false;
    }
    return true;
  }

  bool create_capture_session() {
    if (ACaptureSessionOutputContainer_create(&outputs_) != ACAMERA_OK ||
        ACaptureSessionOutput_create(window_, &output_) != ACAMERA_OK ||
        ACaptureSessionOutputContainer_add(outputs_, output_) != ACAMERA_OK ||
        ACameraOutputTarget_create(window_, &target_) != ACAMERA_OK ||
        ACameraDevice_createCaptureRequest(device_, TEMPLATE_RECORD, &request_) != ACAMERA_OK ||
        ACaptureRequest_addTarget(request_, target_) != ACAMERA_OK) {
      LOGE("Unable to create camera capture request");
      return false;
    }

    ACameraCaptureSession_stateCallbacks session_callbacks = {
      .context = this,
      .onClosed = on_session_closed,
      .onReady = on_session_ready,
      .onActive = on_session_active,
    };
    if (ACameraDevice_createCaptureSession(device_, outputs_, &session_callbacks, &session_) != ACAMERA_OK) {
      LOGE("Unable to create camera capture session");
      return false;
    }
    if (ACameraCaptureSession_setRepeatingRequest(session_, nullptr, 1, &request_, nullptr) != ACAMERA_OK) {
      LOGE("Unable to start repeating camera request");
      return false;
    }
    return true;
  }

  void connection_loop() {
    while (running_) {
      bool connected;
      {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        connected = socket_fd_ >= 0;
      }
      if (!connected) connect_buffers();
      for (int i = 0; i < 10 && running_; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
  }

  void connect_buffers() {
    if (socket_path_.size() >= sizeof(sockaddr_un{}.sun_path)) {
      LOGE("VisionIPC bridge socket path is too long");
      running_ = false;
      return;
    }

    int socket_fd = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
    if (socket_fd < 0) return;
    timeval timeout = {.tv_sec = 2, .tv_usec = 0};
    setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    sockaddr_un address = {};
    address.sun_family = AF_UNIX;
    std::memcpy(address.sun_path, socket_path_.c_str(), socket_path_.size() + 1);
    if (connect(socket_fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
      close(socket_fd);
      return;
    }

    HelloMessage hello = {
      .header = android_camera::make_header<HelloMessage>(MessageType::hello),
      .stream_type = kRoadStream,
      .width = static_cast<uint32_t>(width_),
      .height = static_cast<uint32_t>(height_),
      .buffer_count = static_cast<uint32_t>(requested_buffer_count_),
    };
    if (send(socket_fd, &hello, sizeof(hello), MSG_NOSIGNAL) != static_cast<ssize_t>(sizeof(hello))) {
      close(socket_fd);
      return;
    }

    ConfigMessage config = {};
    std::array<char, CMSG_SPACE(android_camera::kMaxBuffers * sizeof(int))> control = {};
    iovec iov = {.iov_base = &config, .iov_len = sizeof(config)};
    msghdr message = {};
    message.msg_iov = &iov;
    message.msg_iovlen = 1;
    message.msg_control = control.data();
    message.msg_controllen = control.size();
    ssize_t received = recvmsg(socket_fd, &message, 0);
    if (received != static_cast<ssize_t>(sizeof(config)) ||
        !android_camera::valid_header(config.header, MessageType::config, sizeof(config)) ||
        config.width != static_cast<uint32_t>(width_) || config.height != static_cast<uint32_t>(height_) ||
        config.buffer_count != static_cast<uint32_t>(requested_buffer_count_)) {
      close(socket_fd);
      return;
    }

    std::vector<int> fds;
    for (cmsghdr *cmsg = CMSG_FIRSTHDR(&message); cmsg != nullptr; cmsg = CMSG_NXTHDR(&message, cmsg)) {
      if (cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SCM_RIGHTS) {
        size_t count = (cmsg->cmsg_len - CMSG_LEN(0)) / sizeof(int);
        const int *received_fds = reinterpret_cast<const int *>(CMSG_DATA(cmsg));
        fds.insert(fds.end(), received_fds, received_fds + count);
      }
    }
    if (fds.size() != config.buffer_count) {
      for (int fd : fds) close(fd);
      close(socket_fd);
      return;
    }

    std::vector<SharedBuffer> buffers;
    buffers.reserve(fds.size());
    size_t mapping_size = static_cast<size_t>(config.buffer_size) + sizeof(uint64_t);
    for (int fd : fds) {
      void *mapping = mmap(nullptr, mapping_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
      if (mapping == MAP_FAILED) {
        for (SharedBuffer &buffer : buffers) {
          munmap(buffer.address, buffer.mapping_size);
          close(buffer.fd);
        }
        close(fd);
        for (size_t i = buffers.size() + 1; i < fds.size(); ++i) close(fds[i]);
        close(socket_fd);
        return;
      }
      std::memset(mapping, 0, mapping_size);
      buffers.push_back({fd, static_cast<uint8_t *>(mapping), mapping_size});
    }

    {
      std::lock_guard<std::mutex> lock(buffer_mutex_);
      if (!running_) {
        for (SharedBuffer &buffer : buffers) {
          munmap(buffer.address, buffer.mapping_size);
          close(buffer.fd);
        }
        close(socket_fd);
        return;
      }
      disconnect_locked();
      socket_fd_ = socket_fd;
      stride_ = config.stride;
      uv_offset_ = config.uv_offset;
      buffer_size_ = config.buffer_size;
      buffers_ = std::move(buffers);
      LOGI("Connected to VisionIPC bridge with %zu buffers", buffers_.size());
    }
  }

  static void sync_dma_buffer(int fd, uint64_t flags) {
    struct dma_buf_sync sync = {.flags = flags};
    if (ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync) < 0 && errno != ENOTTY && errno != EINVAL) {
      LOGE("DMA_BUF_IOCTL_SYNC failed: %s", std::strerror(errno));
    }
  }

  static uint8_t plane_value(const uint8_t *data, int length, int row_stride, int pixel_stride, int row, int column) {
    int64_t index = static_cast<int64_t>(row) * row_stride + static_cast<int64_t>(column) * pixel_stride;
    return index >= 0 && index < length ? data[index] : 0;
  }

  void handle_image(AImageReader *reader) {
    AImage *image = nullptr;
    if (AImageReader_acquireLatestImage(reader, &image) != AMEDIA_OK || image == nullptr) return;

    int64_t timestamp = 0;
    AImage_getTimestamp(image, &timestamp);
    int64_t next_timestamp = next_timestamp_.load();
    if (!running_ || (next_timestamp != 0 && timestamp < next_timestamp)) {
      AImage_delete(image);
      return;
    }

    int planes = 0;
    if (AImage_getNumberOfPlanes(image, &planes) != AMEDIA_OK || planes < 3) {
      AImage_delete(image);
      return;
    }

    uint8_t *plane_data[3] = {};
    int plane_length[3] = {};
    int row_stride[3] = {};
    int pixel_stride[3] = {};
    bool valid = true;
    for (int plane = 0; plane < 3; ++plane) {
      valid = valid && AImage_getPlaneData(image, plane, &plane_data[plane], &plane_length[plane]) == AMEDIA_OK;
      valid = valid && AImage_getPlaneRowStride(image, plane, &row_stride[plane]) == AMEDIA_OK;
      valid = valid && AImage_getPlanePixelStride(image, plane, &pixel_stride[plane]) == AMEDIA_OK;
    }
    if (!valid) {
      AImage_delete(image);
      return;
    }

    std::lock_guard<std::mutex> lock(buffer_mutex_);
    if (socket_fd_ < 0 || buffers_.empty()) {
      AImage_delete(image);
      return;
    }

    uint64_t frame_id = next_frame_id_++;
    uint32_t slot = static_cast<uint32_t>(frame_id % buffers_.size());
    SharedBuffer &buffer = buffers_[slot];
    sync_dma_buffer(buffer.fd, DMA_BUF_SYNC_START | DMA_BUF_SYNC_WRITE);

    uint8_t *y_target = buffer.address;
    for (int row = 0; row < height_; ++row) {
      uint8_t *destination = y_target + static_cast<size_t>(row) * stride_;
      if (pixel_stride[0] == 1 && static_cast<int64_t>(row) * row_stride[0] + width_ <= plane_length[0]) {
        std::memcpy(destination, plane_data[0] + static_cast<size_t>(row) * row_stride[0], width_);
      } else {
        for (int column = 0; column < width_; ++column) {
          destination[column] = plane_value(plane_data[0], plane_length[0], row_stride[0], pixel_stride[0], row, column);
        }
      }
    }

    uint8_t *uv_target = buffer.address + uv_offset_;
    for (int row = 0; row < height_ / 2; ++row) {
      uint8_t *destination = uv_target + static_cast<size_t>(row) * stride_;
      for (int column = 0; column < width_ / 2; ++column) {
        destination[column * 2] = plane_value(plane_data[1], plane_length[1], row_stride[1], pixel_stride[1], row, column);
        destination[column * 2 + 1] = plane_value(plane_data[2], plane_length[2], row_stride[2], pixel_stride[2], row, column);
      }
    }
    *reinterpret_cast<uint64_t *>(buffer.address + buffer_size_) = frame_id;
    sync_dma_buffer(buffer.fd, DMA_BUF_SYNC_END | DMA_BUF_SYNC_WRITE);

    FrameMessage frame = {
      .header = android_camera::make_header<FrameMessage>(MessageType::frame),
      .slot = slot,
      .reserved = 0,
      .frame_id = frame_id,
      .timestamp_sof = static_cast<uint64_t>(timestamp),
      .timestamp_eof = static_cast<uint64_t>(timestamp),
    };
    ssize_t sent = send(socket_fd_, &frame, sizeof(frame), MSG_DONTWAIT | MSG_NOSIGNAL);
    if (sent == static_cast<ssize_t>(sizeof(frame))) {
      if (next_timestamp == 0) next_timestamp = timestamp;
      do {
        next_timestamp += kFrameIntervalNs;
      } while (next_timestamp <= timestamp);
      next_timestamp_ = next_timestamp;
    } else if (sent < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      LOGE("VisionIPC bridge disconnected: %s", std::strerror(errno));
      disconnect_locked();
    }
    AImage_delete(image);
  }

  void disconnect_locked() {
    if (socket_fd_ >= 0) {
      close(socket_fd_);
      socket_fd_ = -1;
    }
    for (SharedBuffer &buffer : buffers_) {
      if (buffer.address != nullptr) munmap(buffer.address, buffer.mapping_size);
      if (buffer.fd >= 0) close(buffer.fd);
    }
    buffers_.clear();
    stride_ = 0;
    uv_offset_ = 0;
    buffer_size_ = 0;
  }

  std::string socket_path_;
  std::string requested_camera_id_;
  int requested_width_;
  int requested_height_;
  int requested_buffer_count_;
  std::string camera_id_;
  int width_ = 0;
  int height_ = 0;

  std::atomic<bool> running_ = false;
  std::atomic<int64_t> next_timestamp_ = 0;
  uint64_t next_frame_id_ = 0;
  std::thread connector_;
  std::mutex buffer_mutex_;
  int socket_fd_ = -1;
  uint32_t stride_ = 0;
  uint32_t uv_offset_ = 0;
  uint32_t buffer_size_ = 0;
  std::vector<SharedBuffer> buffers_;

  ACameraManager *manager_ = nullptr;
  ACameraDevice *device_ = nullptr;
  AImageReader *reader_ = nullptr;
  ANativeWindow *window_ = nullptr;
  ACaptureSessionOutputContainer *outputs_ = nullptr;
  ACaptureSessionOutput *output_ = nullptr;
  ACameraOutputTarget *target_ = nullptr;
  ACaptureRequest *request_ = nullptr;
  ACameraCaptureSession *session_ = nullptr;
};

std::string get_string(JNIEnv *env, jstring value) {
  if (value == nullptr) return {};
  const char *characters = env->GetStringUTFChars(value, nullptr);
  if (characters == nullptr) return {};
  std::string result(characters);
  env->ReleaseStringUTFChars(value, characters);
  return result;
}

}  // namespace

extern "C" JNIEXPORT jlong JNICALL
Java_ai_sunnypilot_camerabridge_CameraService_nativeStart(JNIEnv *env, jclass, jstring socket_path,
                                                           jstring camera_id, jint width, jint height,
                                                           jint buffer_count) {
  auto bridge = std::make_unique<CameraBridge>(get_string(env, socket_path), get_string(env, camera_id),
                                                width, height, buffer_count);
  if (!bridge->start()) return 0;
  return reinterpret_cast<jlong>(bridge.release());
}

extern "C" JNIEXPORT void JNICALL
Java_ai_sunnypilot_camerabridge_CameraService_nativeStop(JNIEnv *, jclass, jlong handle) {
  delete reinterpret_cast<CameraBridge *>(handle);
}
