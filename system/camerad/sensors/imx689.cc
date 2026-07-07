#include <algorithm>
#include <cmath>
#include <iterator>
#include <cstdlib>

#include "system/camerad/sensors/sensor.h"
#include "system/camerad/sensors/generated/imx689_mode_init.h"  // model-constant tables (no per-unit data)
#include "system/camerad/sensors/sensor_qsc.h"                  // runtime QSC splice from EEPROM

// Sony IMX689 (OnePlus 9, main wide = cam-sensor@0, CSIPHY 1, slot 0).
// Confirmed by kernel dmesg: sensor_id 0x689, slave_addr 0x34, slot 0.
// Modeled on the IMX766 driver (same Sony IMX register conventions); ISP
// tuning params are placeholders until tuned for the IMX689. Probe-capable.
//
// TODO(op9/imx689): mipi_data_rate / mipi_settle are placeholders carried over
// from IMX766's 2x2-binned mode -- capture the real CSIPHY 1 values from a HAL
// open of the wide camera (CAM_START_PHYDEV dmesg line). The streaming init is
// built below from generated/imx689_mode_init.h + the EEPROM QSC (see
// docs/SENSOR-CALIBRATION-EEPROM.md).

namespace {

const float sensor_analog_gains_IMX689[] = {
    1.0, 1.0625, 1.125, 1.1875, 1.25, 1.3125, 1.375, 1.4375, 1.5, 1.5625, 1.6875,
    1.8125, 1.9375, 2.0, 2.125, 2.25, 2.375, 2.5, 2.625, 2.75, 2.875, 3.0,
    3.125, 3.375, 3.625, 3.875, 4.0, 4.25, 4.5, 4.75, 5.0, 5.25, 5.5,
    5.75, 6.0, 6.25, 6.5, 7.0, 7.5, 8.0, 8.5, 9.0, 9.5, 10.0,
    10.5, 11.0, 11.5, 12.0, 12.5, 13.0, 13.5, 14.0, 14.5, 15.0, 15.5};

// [op9ae] Sony analog gain register code from linear gain: gain = 1024/(1024-code).
// Ceiling 960 (16x) is the highest code proven on the OP9 sensors (OP9_GAIN=960).
inline uint16_t sony_gain_code(float gain) {
  int code = (int)lroundf(1024.0f - 1024.0f / std::max(gain, 1.0f));
  return (uint16_t)std::clamp(code, 0, 960);
}

}  // namespace

IMX689::IMX689() {
  image_sensor = cereal::FrameData::ImageSensor::IMX689;
  bayer_pattern = CAM_ISP_PATTERN_BAYER_RGRGRG;  // IMX689 RGGB; verify on tune
  pixel_size_mm = 0.0012;  // IMX689 native 1.2um cell (placeholder)
  data_word = false;       // Sony IMX: WORD addr, BYTE data (same as IMX766)

  // 2x2-binned 4000x3000 RAW10 (placeholder, matches IMX766 binned mode until
  // the real IMX689 mode register table is extracted). out_scale=1 for first
  // bring-up (no IFE scaler).
  out_scale = 1;
  // [op9/stock] OP9_SENSOR_CROP_W narrows the sensor OUTPUT (digital crop below) so the IFE INPUT
  // fits a SINGLE IFE (the native 4000-wide input forces dual-IFE + the overflow-prone pixel path).
  frame_width = getenv("OP9_SENSOR_CROP_W") ? atoi(getenv("OP9_SENSOR_CROP_W")) : 4000;
  // [op9] +1 so demosaic(frame_height-1)=3000 even; op9_y_h=3000 overrides odd io_cfg on the custom
  // kernel. STOCK has no op9_y_h param, so the WM would expect 3001 vs the sensor's 3000 Y_OUT_SIZE
  // -> image size violation. Use the even 3000 on the stock-crop path (no GPU demosaic there).
  frame_height = getenv("OP9_SENSOR_CROP_H") ? atoi(getenv("OP9_SENSOR_CROP_H"))
               : (getenv("OP9_SENSOR_CROP_W") ? 3000 : 3001);
  frame_stride = frame_width * 2;  // [op9] PLAIN16_10 output (2B/px, 16-byte aligned) for legal RDI/RAW WM packer

  extra_height = 0;
  frame_offset = 0;

  // [op9] Stream on/off is the SMIA++ standard 0x0100 write (not per-unit).
  start_reg_array = {{0x0100, 0x01}};

  // [op9] Build the streaming init entirely from model-constant tables + THIS
  // unit's per-unit calibration read/derived from the sensor EEPROM at runtime.
  // NO per-unit data is compiled into the binary. Recipe (unified with imx766):
  //   PRE[0:lsc_pre_index] + LSC(eeprom mesh) + PRE[lsc_pre_index:]
  //   + QSC(eeprom copy) + QSC_tail(binned) + post
  // See sensors/sensor_qsc.h + docs/SENSOR-CALIBRATION-EEPROM.md.
  SensorInitSpec qsc{};
  qsc.pre = imx689_mode_init_pre;   qsc.pre_n = std::size(imx689_mode_init_pre);
  qsc.post = imx689_mode_init_post; qsc.post_n = std::size(imx689_mode_init_post);
  qsc.lsc_pre_index = 655;          // splice LSC after the first 655 pre regs (imx689_init_meta.json)
  qsc.qsc_reg_lo = 0xd000;          qsc.qsc_len = 3072;
  qsc.eeprom_qsc_off = 7936;
  qsc.qsc_tail = true;              qsc.qsc_tail_reg_lo = 0xdc00;  // binned QSC (mean of 4 phases)
  // LSC: 4ch x 12x16 grid, block-center bilinear /2 of the EEPROM 17x13 mesh @0x1A00.
  qsc.lsc.reg_lo = 0x9b00; qsc.lsc.channels = 4; qsc.lsc.rows = 12; qsc.lsc.cols = 16;
  qsc.lsc.mesh_off = 6656; qsc.lsc.hdr_bytes = 6; qsc.lsc.mesh_w = 17; qsc.lsc.mesh_h = 13; qsc.lsc.div = 2;
  qsc.eeprom_path = "/mnt/vendor/persist/camera/eeprom_imx689_p24c128e.bin";
  init_reg_array = build_sensor_init(qsc);
  // [op9] env-tunable FRAME-0 coarse_integration. The mode-init table (imx689_mode_init.h)
  // writes coarse=0x0bd6(3030) which is what frame 0 actually integrates with (the sensor
  // latches any runtime exposure write for frame 1+ only, and frame 1+ never DMA due to the
  // continuity limit). Patch it here so frame 0 can be short-exposed for bright scenes.
  if (getenv("OP9_INTEG0")) {
    int c0 = atoi(getenv("OP9_INTEG0"));
    for (auto &r : init_reg_array) {
      if (r.reg_addr == 0x0202) r.reg_data = (c0 >> 8) & 0xff;
      else if (r.reg_addr == 0x0203) r.reg_data = c0 & 0xff;
    }
  }
  // [op9/stock] digital-crop the binned 4000-wide output to OP9_SENSOR_CROP_W (center), so the CSID/IFE
  // receive a narrower line that fits a single IFE. DIG_CROP_X_OFFSET(0x0408/9) + DIG_CROP_IMAGE_WIDTH
  // (0x040c/d) + X_OUT_SIZE(0x034c/d). Sensor still reads full analog; only the digital output is cropped.
  if (getenv("OP9_SENSOR_CROP_W")) {
    int cw = atoi(getenv("OP9_SENSOR_CROP_W"));
    int cx = ((4000 - cw) / 2) & ~3;  // center, 4-aligned (Bayer quad)
    for (auto &r : init_reg_array) {
      if (r.reg_addr == 0x0408) r.reg_data = (cx >> 8) & 0xff;        // DIG_CROP_X_OFFSET hi
      else if (r.reg_addr == 0x0409) r.reg_data = cx & 0xff;          // lo
      else if (r.reg_addr == 0x040c) r.reg_data = (cw >> 8) & 0xff;   // DIG_CROP_IMAGE_WIDTH hi
      else if (r.reg_addr == 0x040d) r.reg_data = cw & 0xff;          // lo
      else if (r.reg_addr == 0x034c) r.reg_data = (cw >> 8) & 0xff;   // X_OUT_SIZE hi
      else if (r.reg_addr == 0x034d) r.reg_data = cw & 0xff;          // lo
    }
  }
  // [op9/stock] OP9_SENSOR_CROP_H: vertical digital crop (DIG_CROP_Y_OFFSET 0x040a/b +
  // DIG_CROP_IMAGE_HEIGHT 0x040e/f + Y_OUT_SIZE 0x034e/f). With both W+H cropped small, a raw/FD
  // output needs NO IFE downscaler (input==output) -> the WM matches -> drains -> streams on stock.
  if (getenv("OP9_SENSOR_CROP_H")) {
    int ch = atoi(getenv("OP9_SENSOR_CROP_H"));
    int cy = ((3000 - ch) / 2) & ~3;  // center, 4-aligned
    for (auto &r : init_reg_array) {
      if (r.reg_addr == 0x040a) r.reg_data = (cy >> 8) & 0xff;        // DIG_CROP_Y_OFFSET hi
      else if (r.reg_addr == 0x040b) r.reg_data = cy & 0xff;          // lo
      else if (r.reg_addr == 0x040e) r.reg_data = (ch >> 8) & 0xff;   // DIG_CROP_IMAGE_HEIGHT hi
      else if (r.reg_addr == 0x040f) r.reg_data = ch & 0xff;          // lo
      else if (r.reg_addr == 0x034e) r.reg_data = (ch >> 8) & 0xff;   // Y_OUT_SIZE hi
      else if (r.reg_addr == 0x034f) r.reg_data = ch & 0xff;          // lo
    }
  }
  // [op9orient] image orientation reg 0x0101: bit0=H-mirror, bit1=V-flip (0=0deg,
  // 1=mirror, 2=flip, 3=180deg). Mode table shipped 0x03 (upside-down); upright is 0x00
  // (verified by sweep). Default 0x00, OP9_ORIENT overrides.
  {
    int o = getenv("OP9_ORIENT") ? (atoi(getenv("OP9_ORIENT")) & 0x03) : 0x00;
    for (auto &r : init_reg_array) if (r.reg_addr == 0x0101) r.reg_data = o;
    fprintf(stderr, "[op9orient] imx689 0x0101=0x%02x\n", o);
  }
  // [op9sync] SOF period match, on THIS (wide) sensor because it is the timing-tolerant
  // one: RDI raw -> in-camerad software debayer, no BPS/captured-blob coupling. (Retiming
  // the imx766 road cam instead hard-crashes the SoC -- its BPS replay blobs + CPAS/NOC
  // programming were captured at native timing; see imx766.cc + camera/README.md.)
  // Mode FLL 0x0C16=3094 x 21.764us = 67.336ms vs the road's 67.140ms -> 196us/frame
  // drift (SOF offset sweeps the full period every ~23s, modeld drops ~30% of pairings).
  // FLL 3085 = 67.143ms, ~3us/frame residual. Readout is 3000 rows -> 85 blanking lines
  // remain (>> the ~24-line Sony minimum). Applied at INIT, before stream-on.
  // OP9_WIDE_FLL overrides; 0 keeps the mode value.
  {
    int fll = getenv("OP9_WIDE_FLL") ? atoi(getenv("OP9_WIDE_FLL")) : 0;
    if (fll > 0) {
      for (auto &r : init_reg_array) {
        if (r.reg_addr == 0x0340) r.reg_data = (fll >> 8) & 0xff;
        if (r.reg_addr == 0x0341) r.reg_data = fll & 0xff;
      }
      fprintf(stderr, "[op9sync] imx689 init FRM_LENGTH_LINES=%d (period match to imx766 road)\n", fll);
    }
  }
  // [op9] init applied as fixed-size chunks by spectra.cc (one giant CCI packet
  // overflows the kernel CDM); leave init_group_sizes empty for that path.
  apply_init_exposure = true;  // [op9] initial exposure before stream-on (Sony IMX)

  // Sony IMX689 sensor ID: 0x0016/0x0017 (WORD) == 0x0689 (confirmed dmesg).
  probe_reg_addr = 0x0016;
  probe_expected_data = 0x0689;

  bits_per_pixel = 10;
  mipi_format = CAM_FORMAT_MIPI_RAW_10;
  frame_data_type = CSI_RAW10;
  mclk_frequency = 19200000;       // 19.2 MHz (OnePlus 9 DT clock-rates)
  // TODO(op9/imx689): capture real CSIPHY 1 datarate/settle from a HAL wide-cam
  // open (CAM_START_PHYDEV dmesg). IMX766 binned values used as placeholder.
  mipi_data_rate = 1925500000ULL;
  mipi_settle = 2800000000ULL;     // 2.8 us
  mipi_cphy = true;  // [op9] stock IMX689 = C-PHY 3-trio (CSID lane_type:1 lane_num:3); was defaulting to D-PHY -> CSIPHY could not decode -> no MIPI

  readout_time_ns = 11000000;

  // --- exposure / gain (placeholders, reuse IMX766-style) ---
  ev_scale = 150.0;
  dc_gain_factor = 1;
  dc_gain_min_weight = 1;
  dc_gain_max_weight = 1;
  dc_gain_on_grey = 0.9;
  dc_gain_off_grey = 1.0;
  exposure_time_min = 2;
  // [op9ae] max coarse integration lines. ~3000 = the 3000-row binned mode's full frame at
  // native rate; beyond FLL the sensor auto-extends the frame (fps drops, more light).
  // OP9_AE_MAXT trades fps for low-light exposure (8000 proven stable through the chain).
  exposure_time_max = getenv("OP9_AE_MAXT") ? atoi(getenv("OP9_AE_MAXT")) : 3000;
  analog_gain_min_idx = 0x0;
  analog_gain_rec_idx = 0x0;
  analog_gain_max_idx = 54;  // [op9ae] 15.5x (code 958); was 0x28=8.5x
  analog_gain_cost_delta = -1;
  analog_gain_cost_low = 0.4;
  analog_gain_cost_high = 6.4;
  for (int i = 0; i <= analog_gain_max_idx; i++) {
    sensor_analog_gains[i] = sensor_analog_gains_IMX689[i];
  }
  min_ev = exposure_time_min * sensor_analog_gains[analog_gain_min_idx];
  max_ev = exposure_time_max * dc_gain_factor * sensor_analog_gains[analog_gain_max_idx];
  target_grey_factor = 0.01;

  // --- ISP params (placeholders) ---
  black_level = 64;  // RAW10 typical
  color_correct_matrix = {
    0x000000c2, 0x00000fe0, 0x00000fde,
    0x00000fa7, 0x000000d9, 0x00001000,
    0x00000fca, 0x00000fef, 0x000000c7,
  };
  gamma_lut_rgb = {};
  for (int i = 0; i < 65; i++) {
    float fx = i / 64.0;
    gamma_lut_rgb.push_back((uint32_t)((10*fx)/(1+9*fx)*1023.0 + 0.5));
  }
  prepare_gamma_lut();

  linearization_lut = std::vector<uint32_t>(36, 0);
  linearization_pts = {0x0fff0fff, 0x0fff0fff, 0x0fff0fff, 0x0fff0fff};
  vignetting_lut = std::vector<uint32_t>(221, 0);
}

std::vector<i2c_random_wr_payload> IMX689::getExposureRegisters(int exposure_time, int new_exp_g, bool dc_gain_enabled) const {
  // Sony IMX coarse integration time @ 0x0202/0x0203, analog gain @ 0x0204/0x0205.
  // [op9ae] new_exp_g is a gain INDEX into sensor_analog_gains (upstream semantic).
  // The register wants the Sony gain CODE (gain = 1024/(1024-code)); writing the raw
  // index programmed ~1.0x for every index, so AE could never brighten the image.
  // Bracket with grouped-parameter-hold (0x0104) so integ+gain latch on the same frame.
  uint32_t e = (uint32_t)std::clamp(exposure_time, 2, exposure_time_max);
  int g = std::clamp(new_exp_g, analog_gain_min_idx, analog_gain_max_idx);
  uint16_t code = sony_gain_code(sensor_analog_gains[g]);
  return {
    {0x0104, 1},
    {0x0202, (uint16_t)((e >> 8) & 0xff)},
    {0x0203, (uint16_t)(e & 0xff)},
    {0x0204, (uint16_t)((code >> 8) & 0xff)},
    {0x0205, (uint16_t)(code & 0xff)},
    {0x0104, 0},
  };
}

float IMX689::getExposureScore(float desired_ev, int exp_t, int exp_g_idx, float exp_gain, int gain_idx) const {
  float score = std::abs(desired_ev - (exp_t * exp_gain));
  score += std::abs(exp_g_idx - analog_gain_rec_idx) * analog_gain_cost_low;
  return score;
}

int IMX689::getSlaveAddress(int port) const {
  // OnePlus 9: IMX689 i2c slave (8-bit) = 0x34 (confirmed dmesg slot 0).
  // One entry per port.
  return 0x34;
}
