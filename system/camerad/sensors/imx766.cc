#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iterator>

#include "system/camerad/sensors/sensor.h"
#include "system/camerad/sensors/generated/imx766_mode_init.h"  // model-constant tables (no per-unit data)
#include "system/camerad/sensors/sensor_qsc.h"                  // runtime QSC splice from EEPROM

// Sony IMX766 (OnePlus 9). Modeled on the os04c10 driver; ISP tuning params are
// placeholders (reuse os04c10 values) until tuned for the IMX766. Probe-capable.

namespace {

const float sensor_analog_gains_IMX766[] = {
    1.0, 1.0625, 1.125, 1.1875, 1.25, 1.3125, 1.375, 1.4375, 1.5, 1.5625, 1.6875,
    1.8125, 1.9375, 2.0, 2.125, 2.25, 2.375, 2.5, 2.625, 2.75, 2.875, 3.0,
    3.125, 3.375, 3.625, 3.875, 4.0, 4.25, 4.5, 4.75, 5.0, 5.25, 5.5,
    5.75, 6.0, 6.25, 6.5, 7.0, 7.5, 8.0, 8.5, 9.0, 9.5, 10.0,
    10.5, 11.0, 11.5, 12.0, 12.5, 13.0, 13.5, 14.0, 14.5, 15.0, 15.5,
    // [op9ae] extended range: the IMX766 FLL is pinned at native timing so exposure caps at ~33ms
    // (vs stock's 50ms); the custom 0x0B8E gain reg supports up to 64x (gain*64 <= 4095), so allow
    // more analog gain to reach stock-equivalent brightness. Stock swept this sensor to ~64x.
    16.0, 17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 24.0, 26.0, 28.0, 30.0, 32.0};

// [op9ae] Standard Sony analog gain code (gain = 1024/(1024-code)). UNUSED on the IMX766:
// this sensor uses the custom 0x0B8E gain register (gain*64) instead (see getExposureRegisters).
// Kept for reference / other Sony modes. [[maybe_unused]] to satisfy -Werror=unused-function.
[[maybe_unused]] inline uint16_t sony_gain_code(float gain) {
  int code = (int)lroundf(1024.0f - 1024.0f / std::max(gain, 1.0f));
  return (uint16_t)std::clamp(code, 0, 960);
}

}  // namespace

IMX766::IMX766() {
  image_sensor = cereal::FrameData::ImageSensor::IMX766;
  bayer_pattern = CAM_ISP_PATTERN_BAYER_RGRGRG;  // IMX766 RGGB; verify on tune
  pixel_size_mm = 0.0016;  // 2x2-binned cell ~1.6um (0.8um native x2)
  data_word = false;       // Sony IMX: WORD addr, BYTE data (confirmed by HAL capture)

  // Mode: 2x2-binned full-FOV 4096x3072 RAW10 C-PHY (the mode the HAL actually
  // streams; init built below from generated/imx766_mode_init.h + EEPROM QSC).
  // out_scale=1 for first bring-up; switch to out_scale=2 -> 2048x1536 once frames are confirmed.
  out_scale = 1;
  frame_width = 4096;   // [op9] stock ultrawide mode 0x034C=0x1000
  frame_height = 3072;  // [op9] 0x034E=0x0C00
  frame_stride = frame_width * 2;  // [op9] PLAIN16_10 output (2B/px, 16-byte aligned) for RDI WM

  extra_height = 0;
  frame_offset = 0;

  // [op9] Stream on/off is the SMIA++ standard 0x0100 write (not per-unit).
  start_reg_array = {{0x0100, 0x01}};

  // [op9] Build the streaming init from model-constant tables + THIS unit's QSC
  // calibration, read from the sensor EEPROM at runtime. NO per-unit data is
  // compiled into the binary. The QSC window (regs 0xc800..0xd3ff =
  // eeprom[8144:11216]) is spliced between the pre/post tables. If the EEPROM
  // can't be read, init streams without QSC (raw OK; cooked shading degraded).
  // See docs/SENSOR-CALIBRATION-EEPROM.md.
  // imx766 has no LSC block and no derived QSC tail (only the QSC copy).
  SensorInitSpec qsc{};
  qsc.pre = imx766_mode_init_pre;   qsc.pre_n = std::size(imx766_mode_init_pre);
  qsc.post = imx766_mode_init_post; qsc.post_n = std::size(imx766_mode_init_post);
  qsc.qsc_reg_lo = 0xc800;          qsc.qsc_len = 3072;
  qsc.eeprom_qsc_off = 8144;
  qsc.eeprom_path = "/mnt/vendor/persist/camera/eeprom_imx766_gt24p128ca2.bin";
  init_reg_array = build_sensor_init(qsc);
  // [op9orient] image orientation reg 0x0101: bit0=H-mirror, bit1=V-flip (0=0deg,
  // 1=mirror, 2=flip, 3=180deg). The mode table shipped 0x03 (=180deg = upside-down on
  // this mount); the sensor is actually upright at 0x00 (verified by the OP9_ORIENT
  // sweep: only 0x00/0x03 keep RGGB colors; 0x00 puts gravity down). Default 0x00,
  // OP9_ORIENT overrides. (Sony sensors can't do 90deg at readout.)
  {
    int o = getenv("OP9_ORIENT") ? (atoi(getenv("OP9_ORIENT")) & 0x03) : 0x00;
    for (auto &r : init_reg_array) if (r.reg_addr == 0x0101) r.reg_data = o;
    fprintf(stderr, "[op9orient] imx766 0x0101=0x%02x\n", o);
  }
  // [op9sync] DO NOT retime this sensor -- default OFF, debug only. The imx766's RDI
  // feeds the offline BPS with captured replay blobs + CPAS/NOC bandwidth programming
  // captured at the NATIVE mode timing; ANY frame-length change (even a +10-line init
  // override, bisected 2026-07-06) hard-crashes the SoC (silent NOC/watchdog crash-dump)
  // within ~3min of streaming. Period matching is done on the imx689 wide instead
  // (software-debayer path, no BPS coupling) -- see imx689.cc OP9_WIDE_FLL.
  {
    int fll = getenv("OP9_ROAD_FLL") ? atoi(getenv("OP9_ROAD_FLL")) : 0;
    if (fll > 0) {
      for (auto &r : init_reg_array) {
        if (r.reg_addr == 0x0340) r.reg_data = (fll >> 8) & 0xff;
        if (r.reg_addr == 0x0341) r.reg_data = fll & 0xff;
      }
      fprintf(stderr, "[op9sync] imx766 init FRM_LENGTH_LINES=%d (DANGER: crashes the SoC, debug only)\n", fll);
    }
  }
  // [op9] init applied as fixed-size chunks by spectra.cc (one giant CCI packet
  // overflows the kernel CDM); leave init_group_sizes empty for that path.
  apply_init_exposure = true;
  mipi_cphy = true;  // [op9] IMX766 streams C-PHY 3-trio (HAL: is_3phase=1, lane_cnt=3)

  // Sony IMX766 sensor ID: 0x0016/0x0017 (WORD) == 0x0766
  probe_reg_addr = 0x0016;
  probe_expected_data = 0x0766;

  bits_per_pixel = 10;
  mipi_format = CAM_FORMAT_MIPI_RAW_10;
  frame_data_type = CSI_RAW10;
  mclk_frequency = 19200000;       // 19.2 MHz (OnePlus 9 DT clock-rates)
  mipi_data_rate = 1925500000ULL;  // [op9] 4000x3000 mode (PLL-derived)
  mipi_settle = 2800000000ULL;     // 2.8 us (HAL CSIPHY log)

  readout_time_ns = 11000000;

  // [op9sync] 4000x3000 binned mode: FRM_LENGTH_LINES 0x0CEE=3310 (mode minimum),
  // measured frame period 67.1403ms -> line time 20284ns (= LINE_LENGTH_PCK 0x3D00
  // at the mode's pixel rate). Used to phase-lock this sensor's SOF to the wide cam.
  frame_length_lines = 3310;
  line_time_ns = 20284;

  // --- exposure / gain (placeholders, reuse os04c10-style) ---
  ev_scale = 150.0;
  dc_gain_factor = 1;
  dc_gain_min_weight = 1;
  dc_gain_max_weight = 1;
  dc_gain_on_grey = 0.9;
  dc_gain_off_grey = 1.0;
  exposure_time_min = 2;
  // [op9ae] max coarse integration lines. The IMX766 FLL is PINNED at its native 3310 (the BPS
  // replay blobs + CPAS/NOC bandwidth are captured at that exact timing; changing FLL -- at init
  // OR per-frame -- hard-crashes the SoC within ~3min, see the [op9sync]/ctor notes and the
  // 2026-07 reboot). So exposure can't exceed ~FLL: cap just below 3310 (leave a few guard lines
  // for the coarse-integ margin). Stock reaches 50ms because its HAL safely extends FLL every
  // frame; we can't, so we compensate with the custom 0x0B8E analog gain instead (getExposureRegisters).
  // OP9_AE_MAXT overrides (do NOT exceed 3300 unless you also lift the FLL, which crashes).
  exposure_time_max = getenv("OP9_AE_MAXT") ? atoi(getenv("OP9_AE_MAXT")) : 3300;
  analog_gain_min_idx = 0x0;
  analog_gain_rec_idx = 0x0;
  analog_gain_max_idx = 66;  // [op9ae] 32.0x via custom 0x0B8E (compensates the FLL-capped ~33ms exposure)
  analog_gain_cost_delta = -1;
  analog_gain_cost_low = 0.4;
  analog_gain_cost_high = 6.4;
  for (int i = 0; i <= analog_gain_max_idx; i++) {
    sensor_analog_gains[i] = sensor_analog_gains_IMX766[i];
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

std::vector<i2c_random_wr_payload> IMX766::getExposureRegisters(int exposure_time, int new_exp_g, bool dc_gain_enabled) const {
  // [op9ae] IMX766 uses a CUSTOM (OPLUS) analog-gain register 0x0B8E (NOT the standard Sony
  // 0x0204). Decoded from the stock CamX HAL's live CCI writes (sid 0x1a) while sweeping AE
  // (see stock_ae_reference.md). Writing 0x0204 had ZERO effect -> the ultra-wide stayed dark.
  //   0x0B8E (16-bit) = round(gain * 64)   VERIFIED: 0x01D1=465 -> 7.27x; 0x02E6=742 -> 11.59x
  //   0x0202/0203 = EXP (coarse integration time), 0x020E/020F = 0x0100 digital gain (1.0x).
  //
  // *** DO NOT write FLL (0x0340) or 0x3128 per-frame. *** The stock HAL co-writes FLL every
  // frame, but on our free-running (non-HAL) path a per-frame FLL/0x3128 change hard-crashes
  // the SoC (silent NOC/watchdog reboot within a few minutes -- reproduced 2026-07, also see
  // the [op9sync] notes). FLL is instead pinned ONCE at init (frame_length_lines, see the ctor
  // OP9_ROAD_FLL / mode-init table); exposure is clamped below it here. That fixes the 50 ms
  // integration window without touching FLL mid-stream.
  uint32_t e = (uint32_t)std::clamp(exposure_time, 2, exposure_time_max);
  int g = std::clamp(new_exp_g, analog_gain_min_idx, analog_gain_max_idx);
  uint16_t gcode = (uint16_t)std::clamp((int)lroundf(sensor_analog_gains[g] * 64.0f), 64, 4095);  // [op9] gain*64
  return {
    {0x0104, 0x01},                              // GPH on
    {0x0202, (uint16_t)((e >> 8) & 0xff)},       // EXP hi
    {0x0203, (uint16_t)(e & 0xff)},              // EXP lo
    {0x020E, 0x01},                              // digital gain hi = 1.0x
    {0x020F, 0x00},                              // digital gain lo
    {0x0B8E, (uint16_t)((gcode >> 8) & 0xff)},   // [op9] CUSTOM analog gain hi (gain*64)
    {0x0B8F, (uint16_t)(gcode & 0xff)},          // [op9] CUSTOM analog gain lo
    {0x0104, 0x00},                              // GPH off
  };
}

std::vector<i2c_random_wr_payload> IMX766::getFrameLengthRegisters(int frame_length) const {
  // [op9sync] Sony IMX FRM_LENGTH_LINES @ 0x0340/0x0341. NO group-hold bracket here:
  // these registers are spliced INSIDE the exposure packet's existing 0x0104 bracket
  // (integ+gain+FLL in ONE group hold = the standard Sony AE pattern the stock HAL
  // uses). A separate second GPH bracket in the same per-frame packet correlated with
  // hard SoC crash-dumps on this platform (see camera_qcom2.cc [op9sync] notes).
  // Never below the mode minimum (cuts readout); upper bound is a sanity cap (~ +24ms).
  uint32_t fll = (uint32_t)std::clamp(frame_length, (int)frame_length_lines, (int)frame_length_lines + 1200);
  return {
    {0x0340, (uint16_t)((fll >> 8) & 0xff)},
    {0x0341, (uint16_t)(fll & 0xff)},
  };
}

float IMX766::getExposureScore(float desired_ev, int exp_t, int exp_g_idx, float exp_gain, int gain_idx) const {
  float score = std::abs(desired_ev - (exp_t * exp_gain));
  score += std::abs(exp_g_idx - analog_gain_rec_idx) * analog_gain_cost_low;
  return score;
}

int IMX766::getSlaveAddress(int port) const {
  // OnePlus 9: IMX766 i2c slave (8-bit) = 0x34 (from dmesg). One entry per port.
  return 0x34;
}
