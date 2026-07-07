#include "system/camerad/cameras/camera_common.h"
#include "system/camerad/cameras/spectra.h"

#include <poll.h>
#include <sys/ioctl.h>

#include <algorithm>
#include <cassert>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>

#include "common/params.h"
#include "common/swaglog.h"


ExitHandler do_exit;

// [op9ae] warm-up: first N frames run at a fixed fast/mid exposure (keeps frames
// arriving quickly for first-frame-sync; avoids reacting to garbage BPS scratch).
static const int AE_WARMUP_FRAMES = 4;
static const int AE_WARMUP_T = 800;  // mid integration, fast enough for the sync watchdog
static const int AE_WARMUP_G = 0;    // 1.0x

// [op9ae] AE brightness target. The upstream DYNAMIC target-grey curve depends on
// cur_ev * ev_scale magnitudes calibrated to comma's OX03C10/OS04C10 — not these Sony
// sensors, so it lands arbitrarily dark (~0.13) here. A fixed target is the reliable
// lever (ev_scale cancels out of desired_ev, so it only fed the dynamic curve anyway).
//
// PER-CAMERA targets (2026-07 tune vs stock HAL ground truth, indoor):
//   stock IMX689 main -> 50ms + ~4.0x gain (ISO ~400);  IMX766 ultra-wide -> 50ms + ~10x (ISO ~1000).
// A single shared target made the road (IMX689) too bright and the wide (IMX766) too dark,
// because the two sensors differ ~2.5x in sensitivity and share the grey/EV loop.
// ROAD target 0.11: VERIFIED against stock — locking IMX689 to the exact stock registers
// (4642 lines / 4.0x, OP9_FIX_ROAD_T/G) produced a correctly-exposed image whose measured
// grey is ~0.109; setting the AE target to 0.11 makes the closed loop converge to the same
// ~4.25-4.5x gain / 50 ms, matching stock (was 0.24 -> AE over-drove to ~6x -> too bright).
// Indexed by camera_num: 0 = IMX689 ROAD/main, 1 = IMX766 WIDE/ultra-wide, 2 = driver.
// Per-camera override: OP9_AE_TARGET_ROAD / OP9_AE_TARGET_WIDE; global OP9_AE_TARGET still
// wins if set (0 = restore dynamic curve).
// WIDE target 0.08: the IMX766 now responds to gain via the custom 0x0B8E register, but its
// FLL is pinned at native timing (can't extend past ~33ms without crashing the SoC), so it's
// exposure-limited and leans on analog gain. Locking it to stock's registers (4966/10x) measured
// grey ~0.070 for a correct exposure, so target ~0.08 (a touch above to keep gain climbing in the
// dark). Overridable via OP9_AE_TARGET_WIDE.
static const float AE_DEFAULT_TARGETS[3] = {0.11f, 0.08f, 0.30f};  // {road/689, wide/766, driver}

// [op9sync] SOF phase-lock: the two OP9 sensors free-run with different mode frame
// periods (road 67.140ms vs wide 67.336ms -> 196us/frame relative drift), so their SOF
// offset sweeps +-33ms every ~23s and modeld's +-10ms frame pairing drops ~30% of frames.
// The road (imx766) mode FLL is its legal minimum (can't shorten), so the road cam tracks
// the slower wide cam: per frame, trim the road sensor's FRM_LENGTH_LINES to match the
// wide period and steer the SOF offset to zero. Single camerad event thread -> plain statics.
static uint64_t sync_wide_sof = 0;        // latest WIDE_ROAD SOF (ns)
static int64_t sync_wide_period_ns = 0;   // EMA of the wide cam's frame period
static const int env_sync_log = getenv("OP9_SYNC_LOG") ? std::max(1, atoi(getenv("OP9_SYNC_LOG"))) : 0;

// [op9sync] PASSIVE monitor only: logs the road-vs-wide SOF phase error. It never
// touches the sensor. Mid-stream per-frame FLL i2c writes were tried and correlated
// with hard SoC crash-dumps (silent, no kernel trace, evidence lost to page cache) --
// see camera/README.md. Sync is instead achieved with init-time period matching
// (imx766.cc OP9_ROAD_FLL) + phase-aligned deferred sensor start (camerad_thread).
static void sof_sync_monitor(const SpectraCamera &camera) {
  if (!env_sync_log || camera.cc.stream_type != VISION_STREAM_ROAD) return;
  if (!sync_wide_sof || !sync_wide_period_ns) return;
  if (camera.buf.cur_frame_data.frame_id % env_sync_log != 0) return;
  const int64_t p = sync_wide_period_ns;
  const uint64_t sof = camera.buf.cur_frame_data.timestamp_sof;
  if ((int64_t)(sof - sync_wide_sof) > 500000000LL) return;  // wide stalled
  int64_t e = ((int64_t)(sof - sync_wide_sof) % p + p) % p;  // phase error in [0, p)
  int64_t d = std::min(e, p - e);                            // distance to alignment
  fprintf(stderr, "[op9sync] road f%u e=%.3fms |d|=%.3fms (wide p=%.4fms)\n",
          camera.buf.cur_frame_data.frame_id, e / 1e6, d / 1e6, p / 1e6);
}

// for debugging
const bool env_debug_frames = getenv("DEBUG_FRAMES") != nullptr;
const bool env_log_raw_frames = getenv("LOG_RAW_FRAMES") != nullptr;
const bool env_ctrl_exp_from_params = getenv("CTRL_EXP_FROM_PARAMS") != nullptr;


class CameraState {
public:
  SpectraCamera camera;
  int exposure_time = 5;
  bool dc_gain_enabled = false;
  int dc_gain_weight = 0;
  int gain_idx = 0;
  float analog_gain_frac = 0;

  float cur_ev[3] = {};
  float best_ev_score = 0;
  int new_exp_g = 0;
  int new_exp_t = 0;

  Rect ae_xywh = {};
  float measured_grey_fraction = 0;
  float target_grey_fraction = 0.125;

  float fl_pix = 0;
  std::unique_ptr<PubMaster> pm;

  CameraState(SpectraMaster *master, const CameraConfig &config) : camera(master, config) {};
  ~CameraState();
  void init(VisionIpcServer *v);
  void update_exposure_score(float desired_ev, int exp_t, int exp_g_idx, float exp_gain);
  void set_camera_exposure(float grey_frac);
  void set_exposure_rect();
  void sendState();

  float get_gain_factor() const {
    return (1 + dc_gain_weight * (camera.sensor->dc_gain_factor-1) / camera.sensor->dc_gain_max_weight);
  }
};

void CameraState::init(VisionIpcServer *v) {
  camera.camera_open(v);

  if (!camera.enabled) return;

  fl_pix = camera.cc.focal_len / camera.sensor->pixel_size_mm / camera.sensor->out_scale;
  set_exposure_rect();

  dc_gain_weight = camera.sensor->dc_gain_min_weight;
  gain_idx = camera.sensor->analog_gain_rec_idx;
  cur_ev[0] = cur_ev[1] = cur_ev[2] = get_gain_factor() * camera.sensor->sensor_analog_gains[gain_idx] * exposure_time;

  pm = std::make_unique<PubMaster>(std::vector{camera.cc.publish_name});
}

CameraState::~CameraState() {}

void CameraState::set_exposure_rect() {
  // set areas for each camera, shouldn't be changed
  std::vector<std::pair<Rect, float>> ae_targets = {
    // (Rect, F)
    std::make_pair((Rect){96, 400, 1734, 524}, 567.0),  // wide
    std::make_pair((Rect){96, 160, 1734, 986}, 2648.0), // road
    std::make_pair((Rect){96, 242, 1736, 906}, 567.0)   // driver
  };
  int h_ref = 1208;
  /*
    exposure target intrinsics is
    [
      [F, 0, 0.5*ae_xywh[2]]
      [0, F, 0.5*H-ae_xywh[1]]
      [0, 0, 1]
    ]
  */
  auto ae_target = ae_targets[camera.cc.camera_num];
  Rect xywh_ref = ae_target.first;
  float fl_ref = ae_target.second;

  ae_xywh = (Rect){
    std::max(0, (int)camera.buf.out_img_width / 2 - (int)(fl_pix / fl_ref * xywh_ref.w / 2)),
    std::max(0, (int)camera.buf.out_img_height / 2 - (int)(fl_pix / fl_ref * (h_ref / 2 - xywh_ref.y))),
    std::min((int)(fl_pix / fl_ref * xywh_ref.w), (int)camera.buf.out_img_width / 2 + (int)(fl_pix / fl_ref * xywh_ref.w / 2)),
    std::min((int)(fl_pix / fl_ref * xywh_ref.h), (int)camera.buf.out_img_height / 2 + (int)(fl_pix / fl_ref * (h_ref / 2 - xywh_ref.y)))
  };
  // [op9ae] the reference geometry assumes comma's 1928x1208 buffers; on the OP9 sensor
  // modes the computed w can extend past the row end (cam0: x=0 w=4179 on a 4000-wide
  // image) -> AE samples out-of-row pixels. Clamp to the actual output image.
  ae_xywh.x = std::clamp(ae_xywh.x, 0, (int)camera.buf.out_img_width - 2);
  ae_xywh.y = std::clamp(ae_xywh.y, 0, (int)camera.buf.out_img_height - 2);
  ae_xywh.w = std::clamp(ae_xywh.w, 2, (int)camera.buf.out_img_width - ae_xywh.x);
  ae_xywh.h = std::clamp(ae_xywh.h, 2, (int)camera.buf.out_img_height - ae_xywh.y);
  fprintf(stderr, "[op9ae] cam%d ae rect x=%d y=%d w=%d h=%d (img %dx%d)\n", camera.cc.camera_num,
          ae_xywh.x, ae_xywh.y, ae_xywh.w, ae_xywh.h, (int)camera.buf.out_img_width, (int)camera.buf.out_img_height);
}

void CameraState::update_exposure_score(float desired_ev, int exp_t, int exp_g_idx, float exp_gain) {
  float score = camera.sensor->getExposureScore(desired_ev, exp_t, exp_g_idx, exp_gain, gain_idx);
  if (score < best_ev_score) {
    new_exp_t = exp_t;
    new_exp_g = exp_g_idx;
    best_ev_score = score;
  }
}

void CameraState::set_camera_exposure(float grey_frac) {
  if (!camera.enabled) return;

  // [op9ae DIAG] Fixed-exposure lock to reproduce the STOCK HAL register values and isolate
  // sensor-side vs ISP-side brightness issues (bringup). Per-stream: OP9_FIX_ROAD_T/OP9_FIX_ROAD_G
  // set the ROAD (IMX689) linecount + analog-gain INDEX; OP9_FIX_WIDE_T/OP9_FIX_WIDE_G the WIDE
  // (IMX766). When set, AE is bypassed and the exact fixed values are programmed every frame.
  // Stock ground truth (indoor): ROAD 4642 lines / 4.0x (idx 26); WIDE 4966 lines / 10.0x (idx 43).
  {
    const char *fT = nullptr, *fG = nullptr;
    if (camera.cc.stream_type == VISION_STREAM_ROAD)      { fT = getenv("OP9_FIX_ROAD_T"); fG = getenv("OP9_FIX_ROAD_G"); }
    else if (camera.cc.stream_type == VISION_STREAM_WIDE_ROAD) { fT = getenv("OP9_FIX_WIDE_T"); fG = getenv("OP9_FIX_WIDE_G"); }
    if (fT && fG) {
      exposure_time = std::clamp(atoi(fT), camera.sensor->exposure_time_min, camera.sensor->exposure_time_max);
      gain_idx = std::clamp(atoi(fG), camera.sensor->analog_gain_min_idx, camera.sensor->analog_gain_max_idx);
      analog_gain_frac = camera.sensor->sensor_analog_gains[gain_idx];
      dc_gain_enabled = false;
      cur_ev[camera.buf.cur_frame_data.frame_id % 3] = exposure_time * analog_gain_frac * get_gain_factor();
      auto fix = camera.sensor->getExposureRegisters(exposure_time, gain_idx, false);
      camera.sensors_i2c(fix.data(), fix.size(), CAM_SENSOR_PACKET_OPCODE_SENSOR_CONFIG, camera.sensor->data_word);
      static const int fix_log = getenv("OP9_AE_LOG") ? std::max(1, atoi(getenv("OP9_AE_LOG"))) : 0;
      if (fix_log && (camera.buf.cur_frame_data.frame_id % fix_log == 0))
        fprintf(stderr, "[op9fix] cam%d f%u FIXED t=%d g=%d(%.2fx) grey=%.4f\n",
                camera.cc.camera_num, camera.buf.cur_frame_data.frame_id, exposure_time, gain_idx, analog_gain_frac, grey_frac);
      return;
    }
  }
  // [op9sync] track the wide cam's SOF + frame period (the sync reference)
  if (camera.cc.stream_type == VISION_STREAM_WIDE_ROAD) {
    const uint64_t sof = camera.buf.cur_frame_data.timestamp_sof;
    if (sync_wide_sof) {
      int64_t d = (int64_t)(sof - sync_wide_sof);
      if (d > 45000000LL && d < 95000000LL) {  // reject gaps (recovery/drops)
        if (!sync_wide_period_ns && env_sync_log)
          fprintf(stderr, "[op9sync] wide reference locked: period %.4fms\n", d / 1e6);
        sync_wide_period_ns = sync_wide_period_ns ? (sync_wide_period_ns * 7 + d) / 8 : d;
      }
    }
    sync_wide_sof = sof;
  }
  // [op9ae] The first few BPS outputs contain uninitialized/garbage data (grey reads ~1.0).
  // Do NOT skip programming during them: the first-frame-sync watchdog needs frames to keep
  // arriving fast, so the sensor must be pulled off the slow OP9_INTEG startup exposure
  // immediately. Instead program a fixed, fast, mid exposure for the warm-up frames and only
  // start the closed-loop control once real data is flowing.
  if (camera.buf.cur_frame_data.frame_id < AE_WARMUP_FRAMES) {
    exposure_time = AE_WARMUP_T;
    gain_idx = AE_WARMUP_G;
    analog_gain_frac = camera.sensor->sensor_analog_gains[gain_idx];
    dc_gain_enabled = false;
    cur_ev[camera.buf.cur_frame_data.frame_id % 3] = exposure_time * analog_gain_frac * get_gain_factor();
    auto warm = camera.sensor->getExposureRegisters(exposure_time, gain_idx, false);
    camera.sensors_i2c(warm.data(), warm.size(), CAM_SENSOR_PACKET_OPCODE_SENSOR_CONFIG, camera.sensor->data_word);
    return;
  }
  // [op9ae] pitch-black median is exactly 0 -> desired_ev divides by zero. Floor it.
  grey_frac = std::max(grey_frac, 1.0f / 256);
  std::vector<double> target_grey_minimums = {0.1, 0.1, 0.125}; // wide, road, driver

  const float dt = 0.05;

  const float ts_grey = 10.0;
  // [op9ae] upstream ts_ev=0.05 (k_ev=0.5) assumes the exact 3-frame apply latency of
  // comma's per-request sensor programming. Our free-run + immediate-i2c path has a
  // longer, jittery latency (GPH latch + BPS pipeline) and k_ev=0.5 limit-cycles
  // (grey 0.05<->0.33). Slow the EV filter so delayed feedback stays stable.
  const float ts_ev = 0.4;

  const float k_grey = (dt / ts_grey) / (1.0 + dt / ts_grey);
  const float k_ev = (dt / ts_ev) / (1.0 + dt / ts_ev);

  // It takes 3 frames for the commanded exposure settings to take effect. The first frame is already started by the time
  // we reach this function, the other 2 are due to the register buffering in the sensor.
  // Therefore we use the target EV from 3 frames ago, the grey fraction that was just measured was the result of that control action.
  // TODO: Lower latency to 2 frames, by using the histogram outputted by the sensor we can do AE before the debayering is complete

  const auto &sensor = camera.sensor;
  // Offset idx by one to not get stuck in self loop
  const float cur_ev_ = cur_ev[(camera.buf.cur_frame_data.frame_id - 1) % 3] * sensor->ev_scale;

  // Scale target grey between min and 0.4 depending on lighting conditions
  float new_target_grey = std::clamp(0.4 - 0.3 * log2(1.0 + sensor->target_grey_factor*cur_ev_) / log2(6000.0), target_grey_minimums[camera.cc.camera_num], 0.4);
  // [op9ae] fixed brightness target (see AE_DEFAULT_TARGETS). >0 -> fixed; =0 -> dynamic curve.
  // Priority: global OP9_AE_TARGET > per-camera OP9_AE_TARGET_ROAD/WIDE > compiled default.
  static const float ae_target_global = getenv("OP9_AE_TARGET") ? (float)atof(getenv("OP9_AE_TARGET")) : -1.0f;
  static const float ae_target_road = getenv("OP9_AE_TARGET_ROAD") ? (float)atof(getenv("OP9_AE_TARGET_ROAD")) : -1.0f;
  static const float ae_target_wide = getenv("OP9_AE_TARGET_WIDE") ? (float)atof(getenv("OP9_AE_TARGET_WIDE")) : -1.0f;
  float ae_target_env = AE_DEFAULT_TARGETS[std::min(camera.cc.camera_num, 2)];
  if (ae_target_global >= 0.0f) ae_target_env = ae_target_global;                                       // global (incl 0 = dynamic)
  else if (camera.cc.stream_type == VISION_STREAM_ROAD && ae_target_road >= 0.0f) ae_target_env = ae_target_road;
  else if (camera.cc.stream_type == VISION_STREAM_WIDE_ROAD && ae_target_wide >= 0.0f) ae_target_env = ae_target_wide;
  if (ae_target_env > 0.0f) new_target_grey = ae_target_env;
  float target_grey = (1.0 - k_grey) * target_grey_fraction + k_grey * new_target_grey;

  float desired_ev = std::clamp(cur_ev_ / sensor->ev_scale * target_grey / grey_frac, sensor->min_ev, sensor->max_ev);
  float k = (1.0 - k_ev) / 3.0;
  desired_ev = (k * cur_ev[0]) + (k * cur_ev[1]) + (k * cur_ev[2]) + (k_ev * desired_ev);

  best_ev_score = 1e6;
  new_exp_g = 0;
  new_exp_t = 0;

  // Hysteresis around high conversion gain
  // We usually want this on since it results in lower noise, but turn off in very bright day scenes
  bool enable_dc_gain = dc_gain_enabled;
  if (!enable_dc_gain && target_grey < sensor->dc_gain_on_grey) {
    enable_dc_gain = true;
    dc_gain_weight = sensor->dc_gain_min_weight;
  } else if (enable_dc_gain && target_grey > sensor->dc_gain_off_grey) {
    enable_dc_gain = false;
    dc_gain_weight = sensor->dc_gain_max_weight;
  }

  if (enable_dc_gain && dc_gain_weight < sensor->dc_gain_max_weight) {dc_gain_weight += 1;}
  if (!enable_dc_gain && dc_gain_weight > sensor->dc_gain_min_weight) {dc_gain_weight -= 1;}

  std::string gain_bytes, time_bytes;
  if (env_ctrl_exp_from_params) {
    static Params params;
    gain_bytes = params.get("CameraDebugExpGain");
    time_bytes = params.get("CameraDebugExpTime");
  }

  if (gain_bytes.size() > 0 && time_bytes.size() > 0) {
    // Override gain and exposure time
    gain_idx = std::stoi(gain_bytes);
    exposure_time = std::stoi(time_bytes);

    new_exp_g = gain_idx;
    new_exp_t = exposure_time;
    enable_dc_gain = false;
  } else {
    // Simple brute force optimizer to choose sensor parameters to reach desired EV
    int min_g = std::max(gain_idx - 1, sensor->analog_gain_min_idx);
    int max_g = std::min(gain_idx + 1, sensor->analog_gain_max_idx);
    for (int g = min_g; g <= max_g; g++) {
      float gain = sensor->sensor_analog_gains[g] * get_gain_factor();

      // Compute optimal time for given gain
      int t = std::clamp(int(std::round(desired_ev / gain)), sensor->exposure_time_min, sensor->exposure_time_max);

      // Only go below recommended gain when absolutely necessary to not overexpose
      if (g < sensor->analog_gain_rec_idx && t > 20 && g < gain_idx) {
        continue;
      }

      update_exposure_score(desired_ev, t, g, gain);
    }
  }

  measured_grey_fraction = grey_frac;
  target_grey_fraction = target_grey;

  analog_gain_frac = sensor->sensor_analog_gains[new_exp_g];
  gain_idx = new_exp_g;
  exposure_time = new_exp_t;
  dc_gain_enabled = enable_dc_gain;

  float gain = analog_gain_frac * get_gain_factor();
  cur_ev[camera.buf.cur_frame_data.frame_id % 3] = exposure_time * gain;

  // [op9ae] AE-driven ISP digital gain, mirroring the stock HAL "ISP Digital Gain" (rises 1.0x ->
  // ~1.85-2.0x once analog gain + exposure are maxed and the image is still below target; captured
  // on both the main IMX689 and the wide IMX766). The software debayer reads camera.isp_dgain.
  // Ramp slowly (avoid flicker): when railed at max gain & near-max exposure and still short of
  // target, step up toward the cap; otherwise decay back to 1.0x. OP9_ISP_DGAIN forces a fixed value.
  {
    static const int idg_fixed = getenv("OP9_ISP_DGAIN") ? atoi(getenv("OP9_ISP_DGAIN")) : 0;
    if (idg_fixed > 0) {
      camera.isp_dgain = idg_fixed;
    } else {
      const int IDG_MAX = 400;   // 4.0x cap (stock ISP digital gain reaches ~4.0x in very dark scenes)
      const int IDG_MIN = 100;   // 1.0x
      const bool gain_railed = (gain_idx >= sensor->analog_gain_max_idx);
      const bool exp_railed  = (exposure_time >= sensor->exposure_time_max - 4);
      const bool too_dark    = (grey_frac < target_grey * 0.9f);
      const bool too_bright  = (grey_frac > target_grey * 1.1f);
      if (gain_railed && exp_railed && too_dark) {
        camera.isp_dgain = std::min(IDG_MAX, camera.isp_dgain + 2);   // ramp up
      } else if (too_bright || !gain_railed) {
        camera.isp_dgain = std::max(IDG_MIN, camera.isp_dgain - 2);   // decay down
      }
    }
  }

  // LOGE("ae - camera %d, cur_t %.5f, sof %.5f, dt %.5f", camera.cc.camera_num, 1e-9 * nanos_since_boot(), 1e-9 * camera.buf.cur_frame_data.timestamp_sof, 1e-9 * (nanos_since_boot() - camera.buf.cur_frame_data.timestamp_sof));

  auto exp_reg_array = sensor->getExposureRegisters(exposure_time, new_exp_g, dc_gain_enabled);
  camera.sensors_i2c(exp_reg_array.data(), exp_reg_array.size(), CAM_SENSOR_PACKET_OPCODE_SENSOR_CONFIG, camera.sensor->data_word);
  sof_sync_monitor(camera);  // [op9sync] passive phase telemetry (never writes the sensor)

  // [op9ae] AE telemetry: OP9_AE_LOG=<N> prints every Nth frame per camera.
  static const int ae_log_every = getenv("OP9_AE_LOG") ? std::max(1, atoi(getenv("OP9_AE_LOG"))) : 0;
  if (ae_log_every && (camera.buf.cur_frame_data.frame_id % ae_log_every == 0)) {
    fprintf(stderr, "[op9ae] cam%d f%u grey=%.4f target=%.3f desired_ev=%.0f -> t=%d g=%d(%.2fx)\n",
            camera.cc.camera_num, camera.buf.cur_frame_data.frame_id, grey_frac, target_grey,
            desired_ev, exposure_time, gain_idx, analog_gain_frac);
  }
}

void CameraState::sendState() {
  camera.buf.sendFrameToVipc();

  MessageBuilder msg;
  auto framed = (msg.initEvent().*camera.cc.init_camera_state)();
  const FrameMetadata &meta = camera.buf.cur_frame_data;
  framed.setFrameId(meta.frame_id);
  framed.setRequestId(meta.request_id);
  framed.setTimestampEof(meta.timestamp_eof);
  framed.setTimestampSof(meta.timestamp_sof);
  framed.setIntegLines(exposure_time);
  framed.setGain(analog_gain_frac * get_gain_factor());
  framed.setHighConversionGain(dc_gain_enabled);
  framed.setMeasuredGreyFraction(measured_grey_fraction);
  framed.setTargetGreyFraction(target_grey_fraction);
  framed.setProcessingTime(meta.processing_time);

  const float ev = cur_ev[meta.frame_id % 3];
  const float perc = util::map_val(ev, camera.sensor->min_ev, camera.sensor->max_ev, 0.0f, 100.0f);
  framed.setExposureValPercent(perc);
  framed.setSensor(camera.sensor->image_sensor);

  // Log raw frames for road camera
  if (env_log_raw_frames && camera.cc.stream_type == VISION_STREAM_ROAD && meta.frame_id % 100 == 5) {  // no overlap with qlog decimation
    framed.setImage(get_raw_frame_image(&camera.buf));
  }

  set_camera_exposure(calculate_exposure_value(&camera.buf, ae_xywh, 2, camera.cc.stream_type != VISION_STREAM_DRIVER ? 2 : 4));

  // Send the message
  pm->send(camera.cc.publish_name, msg);
}

void camerad_thread() {
  // TODO: centralize enabled handling

  VisionIpcServer v("camerad"); fprintf(stderr,"[op9] A: after VisionIpcServer\n"); fflush(stderr);

  // *** initial ISP init ***
  SpectraMaster m;
  fprintf(stderr,"[op9] B: before m.init\n"); fflush(stderr); m.init(); fprintf(stderr,"[op9] C: after m.init\n"); fflush(stderr);

  // *** per-cam init ***
  std::vector<std::unique_ptr<CameraState>> cams;
  for (const auto &config : ALL_CAMERA_CONFIGS) {
    if (!config.enabled) continue;  // [op9] only construct/probe ENABLED cams -> isolated single-camera test, no cross-CCI interference
    auto cam = std::make_unique<CameraState>(&m, config);
    cam->init(&v);
    cams.emplace_back(std::move(cam));
  }

  v.start_listener();

  // start devices
  LOG("-- Starting devices");
  for (auto &cam : cams) cam->camera.sensors_start();

  // poll events
  LOG("-- Dequeueing Video events");
  while (!do_exit) {
    struct pollfd fds[1] = {{.fd = m.video0_fd, .events = POLLPRI}};
    int ret = poll(fds, std::size(fds), 1000);
    if (ret < 0) {
      if (errno == EINTR || errno == EAGAIN) continue;
      LOGE("poll failed (%d - %d)", ret, errno);
      break;
    }

    if (!(fds[0].revents & POLLPRI)) continue;

    struct v4l2_event ev = {0};
    ret = HANDLE_EINTR(ioctl(fds[0].fd, VIDIOC_DQEVENT, &ev));
    if (ret == 0) {
      if (ev.type == V4L_EVENT_CAM_REQ_MGR_EVENT) {
        struct cam_req_mgr_message *event_data = (struct cam_req_mgr_message *)ev.u.data;
        if (env_debug_frames) {
          printf("sess_hdl 0x%6X, link_hdl 0x%6X, frame_id %llu, req_id %llu, timestamp %.2f ms, sof_status %d\n", event_data->session_hdl, event_data->u.frame_msg.link_hdl,
                 event_data->u.frame_msg.frame_id, event_data->u.frame_msg.request_id, event_data->u.frame_msg.timestamp/1e6, event_data->u.frame_msg.sof_status);
          do_exit = do_exit || event_data->u.frame_msg.frame_id > (1*20);
        }

        for (auto &cam : cams) {
          if (event_data->session_hdl == cam->camera.session_handle) {
            if (cam->camera.handle_camera_event(event_data)) {
              cam->sendState();
            }
            break;
          }
        }
      } else {
        LOGE("unhandled event %d\n", ev.type);
      }
    } else {
      LOGE("VIDIOC_DQEVENT failed, errno=%d", errno);
    }
  }
}
