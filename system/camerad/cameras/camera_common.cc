#include "system/camerad/cameras/camera_common.h"

#include <cassert>
#include <string>

#include "common/swaglog.h"
#include "system/camerad/cameras/spectra.h"


void CameraBuf::init(SpectraCamera *cam, VisionIpcServer * v, int frame_cnt, VisionStreamType type) {
  vipc_server = v;
  stream_type = type;
  frame_buf_count = frame_cnt;

  const SensorInfo *sensor = cam->sensor.get();

  // RAW frames from ISP
  if (cam->cc.output_type != ISP_IFE_PROCESSED) {
    camera_bufs_raw = std::make_unique<VisionBuf[]>(frame_buf_count);

    const int raw_frame_size = (sensor->frame_height + sensor->extra_height) * sensor->frame_stride;
    for (int i = 0; i < frame_buf_count; i++) {
      camera_bufs_raw[i].allocate(raw_frame_size);
    }
    LOGD("allocated %d buffers", frame_buf_count);
  }

  vipc_server->create_buffers_with_sizes(stream_type, VIPC_BUFFER_COUNT, out_img_width, out_img_height, cam->yuv_size, cam->stride, cam->uv_offset);
  LOGD("created %d YUV vipc buffers with size %dx%d", VIPC_BUFFER_COUNT, cam->stride, cam->y_height);
}

CameraBuf::~CameraBuf() {
  if (camera_bufs_raw != nullptr) {
    for (int i = 0; i < frame_buf_count; i++) {
      camera_bufs_raw[i].free();
    }
  }
}

void CameraBuf::sendFrameToVipc() {
  assert(cur_buf_idx >=0 && cur_buf_idx < frame_buf_count);

  if (camera_bufs_raw) {
    cur_camera_buf = &camera_bufs_raw[cur_buf_idx];
  }

  cur_yuv_buf = vipc_server->get_buffer(stream_type, cur_buf_idx);

  VisionIpcBufExtra extra = {
    cur_frame_data.frame_id,
    cur_frame_data.timestamp_sof,
    cur_frame_data.timestamp_eof,
  };
  cur_yuv_buf->set_frame_id(cur_frame_data.frame_id);
  vipc_server->send(cur_yuv_buf, &extra);
}

// common functions

kj::Array<uint8_t> get_raw_frame_image(const CameraBuf *b) {
  const uint8_t *dat = (const uint8_t *)b->cur_camera_buf->addr;

  kj::Array<uint8_t> frame_image = kj::heapArray<uint8_t>(b->cur_camera_buf->len);
  uint8_t *resized_dat = frame_image.begin();

  memcpy(resized_dat, dat, b->cur_camera_buf->len);

  return kj::mv(frame_image);
}

float calculate_exposure_value(const CameraBuf *b, Rect ae_xywh, int x_skip, int y_skip) {
  int lum_med;
  uint32_t lum_binning[256] = {0};
  // [op9] prefer the RAW Bayer buffer for AE when available (ISP_RAW_OUTPUT path): the
  // software debayer's brightness/gamma LUT otherwise brightens the NV12 Y and fools AE
  // into under-exposing (gain pinned at 1x, weak raw signal, crushed dynamic range ->
  // "white light became black"). Measuring pre-LUT raw (10-bit) makes AE expose the sensor
  // correctly regardless of the tone curve applied downstream.
  const bool use_raw = (b->cur_camera_buf != nullptr) && (b->cur_camera_buf->addr != nullptr);
  const uint16_t *raw_ptr = use_raw ? (const uint16_t *)b->cur_camera_buf->addr : nullptr;
  const uint8_t *pix_ptr = b->cur_yuv_buf->y;
  // [op9ae] NV12 rows are stride-pitched (4096), not out_img_width-pitched. Indexing
  // with width (4000 on the imx689) walks diagonally through the Y plane -> AE
  // measures garbage. Use the vipc buffer's real stride.
  const int pitch = (int)b->cur_yuv_buf->stride;

  unsigned int lum_total = 0;
  for (int y = ae_xywh.y; y < ae_xywh.y + ae_xywh.h; y += y_skip) {
    for (int x = ae_xywh.x; x < ae_xywh.x + ae_xywh.w; x += x_skip) {
      // raw is 10-bit in a 16-bit container (PLAIN16_10); >>2 to a [0..255] bin so the
      // existing median/256 return path is reused with the same grey-fraction targets.
      uint8_t lum = use_raw ? (uint8_t)(raw_ptr[(y * pitch) + x] >> 2) : pix_ptr[(y * pitch) + x];
      lum_binning[lum]++;
      lum_total += 1;
    }
  }
  if (lum_total == 0) return 0.5f;  // [op9ae] degenerate rect guard (avoid div-by-zero -> grey=1.0 -> AE slams dark)

  // Find mean lumimance value
  unsigned int lum_cur = 0;
  for (lum_med = 255; lum_med >= 0; lum_med--) {
    lum_cur += lum_binning[lum_med];

    if (lum_cur >= lum_total / 2) {
      break;
    }
  }

  return lum_med / 256.0;
}

int open_v4l_by_name_and_index(const char name[], int index, int flags) {
  for (int v4l_index = 0; /**/; ++v4l_index) {
    std::string v4l_name = util::read_file(util::string_format("/sys/class/video4linux/v4l-subdev%d/name", v4l_index));
    if (v4l_name.empty()) return -1;
    if (v4l_name.find(name) == 0) {
      if (index == 0) {
        return HANDLE_EINTR(open(util::string_format("/dev/v4l-subdev%d", v4l_index).c_str(), flags));
      }
      index--;
    }
  }
}
