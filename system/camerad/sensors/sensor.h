#pragma once

#include <cassert>
#include <cstdint>
#include <map>
#include <utility>
#include <vector>

#include "media/cam_isp.h"
#include "media/cam_sensor.h"

// [op9] MIPI CSI-2 data types, needed by the Sony IMX sensor drivers (frame_data_type).
// Also defined identically in cameras/spectra.h; the guard makes the include order irrelevant.
#ifndef CSI_RAW10
#define CSI_RAW8   0x2A
#define CSI_RAW10  0x2B
#define CSI_RAW12  0x2C
#endif

#include "cereal/gen/cpp/log.capnp.h"
#include "system/camerad/sensors/ox03c10_registers.h"
#include "system/camerad/sensors/os04c10_registers.h"

#define ANALOG_GAIN_MAX_CNT 77  // [op9ae] extended for IMX766 custom-gain range to 64x (was 55)

class SensorInfo {
public:
  SensorInfo() = default;
  virtual std::vector<i2c_random_wr_payload> getExposureRegisters(int exposure_time, int new_exp_g, bool dc_gain_enabled) const { return {}; }
  virtual float getExposureScore(float desired_ev, int exp_t, int exp_g_idx, float exp_gain, int gain_idx) const {return 0; }
  virtual int getSlaveAddress(int port) const { assert(0); }
  // [op9sync] per-frame FRM_LENGTH_LINES write for SOF phase-locking; {} = unsupported
  virtual std::vector<i2c_random_wr_payload> getFrameLengthRegisters(int frame_length) const { return {}; }

  cereal::FrameData::ImageSensor image_sensor = cereal::FrameData::ImageSensor::UNKNOWN;
  float pixel_size_mm;
  uint32_t frame_width, frame_height;
  uint32_t frame_stride;
  uint32_t frame_offset = 0;
  uint32_t extra_height = 0;
  int out_scale = 1;
  int registers_offset = -1;
  int stats_offset = -1;
  int hdr_offset = -1;

  int exposure_time_min;
  int exposure_time_max;

  float dc_gain_factor;
  int dc_gain_min_weight;
  int dc_gain_max_weight;
  float dc_gain_on_grey;
  float dc_gain_off_grey;

  float ev_scale = 1.0;
  float sensor_analog_gains[ANALOG_GAIN_MAX_CNT];
  int analog_gain_min_idx;
  int analog_gain_max_idx;
  int analog_gain_rec_idx;
  int analog_gain_cost_delta;
  float analog_gain_cost_low;
  float analog_gain_cost_high;
  float target_grey_factor;
  float min_ev;
  float max_ev;

  bool data_word;
  uint32_t probe_reg_addr;
  uint32_t probe_expected_data;
  std::vector<i2c_random_wr_payload> start_reg_array;
  std::vector<i2c_random_wr_payload> init_reg_array;

  uint32_t bits_per_pixel;
  uint32_t bayer_pattern;
  uint32_t mipi_format;
  uint32_t mclk_frequency;
  uint32_t frame_data_type;

  // [op9] Sony IMX (imx766/imx689) additions
  uint64_t mipi_data_rate = 0;        // CSIPHY data rate (bps), from the HAL PHYDEV config
  uint64_t mipi_settle = 0;           // CSIPHY settle time as programmed
  bool mipi_cphy = false;             // C-PHY (3-trio) vs D-PHY
  bool apply_init_exposure = false;   // program an initial exposure before stream-on (Sony IMX)
  std::vector<int> init_group_sizes;  // split init into HAL CONFIG_DEV groups (empty -> spectra chunks it)

  uint32_t readout_time_ns;  // used to recover EOF from SOF

  // [op9sync] frame timing for SOF phase-locking (0 = sensor can't sync).
  // frame_length_lines is the sensor mode's FRM_LENGTH_LINES (the legal minimum:
  // shortening below it cuts into readout); line_time_ns is the line period.
  uint32_t frame_length_lines = 0;
  uint32_t line_time_ns = 0;

  // ISP image processing params
  uint32_t black_level;
  std::vector<uint32_t> color_correct_matrix;  // 3x3
  std::vector<uint32_t> gamma_lut_rgb;         // gamma LUTs are length 64 * sizeof(uint32_t); same for r/g/b here
  void prepare_gamma_lut() {
    for (int i = 0; i < 64; i++) {
      gamma_lut_rgb[i] |= ((uint32_t)(gamma_lut_rgb[i+1] - gamma_lut_rgb[i]) << 10);
    }
    gamma_lut_rgb.pop_back();
  }
  std::vector<uint32_t> linearization_lut;     // length 36
  std::vector<uint32_t> linearization_pts;     // length 4
  std::vector<uint32_t> vignetting_lut;        // length 221

  const int num() const {
    return static_cast<int>(image_sensor);
  };
};

class OX03C10 : public SensorInfo {
public:
  OX03C10();
  std::vector<i2c_random_wr_payload> getExposureRegisters(int exposure_time, int new_exp_g, bool dc_gain_enabled) const override;
  float getExposureScore(float desired_ev, int exp_t, int exp_g_idx, float exp_gain, int gain_idx) const override;
  int getSlaveAddress(int port) const override;
};

class OS04C10 : public SensorInfo {
public:
  OS04C10();
  std::vector<i2c_random_wr_payload> getExposureRegisters(int exposure_time, int new_exp_g, bool dc_gain_enabled) const override;
  float getExposureScore(float desired_ev, int exp_t, int exp_g_idx, float exp_gain, int gain_idx) const override;
  int getSlaveAddress(int port) const override;
};

// [op9] Sony IMX sensors for the OnePlus 9 (road = imx766, wide = imx689)
class IMX766 : public SensorInfo {
public:
  IMX766();
  std::vector<i2c_random_wr_payload> getExposureRegisters(int exposure_time, int new_exp_g, bool dc_gain_enabled) const override;
  float getExposureScore(float desired_ev, int exp_t, int exp_g_idx, float exp_gain, int gain_idx) const override;
  int getSlaveAddress(int port) const override;
  std::vector<i2c_random_wr_payload> getFrameLengthRegisters(int frame_length) const override;
};

class IMX689 : public SensorInfo {
public:
  IMX689();
  std::vector<i2c_random_wr_payload> getExposureRegisters(int exposure_time, int new_exp_g, bool dc_gain_enabled) const override;
  float getExposureScore(float desired_ev, int exp_t, int exp_g_idx, float exp_gain, int gain_idx) const override;
  int getSlaveAddress(int port) const override;
};
