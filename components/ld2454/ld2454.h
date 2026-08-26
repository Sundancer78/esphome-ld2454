#pragma once

#include <cmath>
#include <cstdint>

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"

namespace esphome {
namespace ld2454 {

enum class TargetDirection : uint8_t {
  NONE = 0,
  APPROACHING,
  MOVING_AWAY,
  STILL
};

struct TargetData {
  bool detected{false};

  int16_t x{0};
  int16_t y{0};
  int16_t speed_cm_s{0};
  int16_t speed_mm_s{0};

  uint16_t resolution{0};

  float distance{0.0f};
  float angle{0.0f};

  TargetDirection direction{TargetDirection::NONE};

  bool published_detected{false};

  int16_t published_x{0};
  int16_t published_y{0};
  int16_t published_speed_mm_s{0};

  uint16_t published_resolution{0};

  float published_distance{0.0f};
  float published_angle{0.0f};

  TargetDirection published_direction{TargetDirection::NONE};
};

class LD2454Component : public Component, public uart::UARTDevice {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  void set_target_x_sensor(uint8_t target, sensor::Sensor *sensor) {
    if (target < 3)
      this->target_x_sensors_[target] = sensor;
  }

  void set_target_y_sensor(uint8_t target, sensor::Sensor *sensor) {
    if (target < 3)
      this->target_y_sensors_[target] = sensor;
  }

  void set_target_distance_sensor(uint8_t target, sensor::Sensor *sensor) {
    if (target < 3)
      this->target_distance_sensors_[target] = sensor;
  }

  void set_target_angle_sensor(uint8_t target, sensor::Sensor *sensor) {
    if (target < 3)
      this->target_angle_sensors_[target] = sensor;
  }

  void set_target_speed_sensor(uint8_t target, sensor::Sensor *sensor) {
    if (target < 3)
      this->target_speed_sensors_[target] = sensor;
  }

  void set_target_resolution_sensor(uint8_t target, sensor::Sensor *sensor) {
    if (target < 3)
      this->target_resolution_sensors_[target] = sensor;
  }

  void set_target_direction_sensor(uint8_t target, text_sensor::TextSensor *sensor) {
    if (target < 3)
      this->target_direction_sensors_[target] = sensor;
  }

  void set_target_count_sensor(sensor::Sensor *sensor) {
    this->target_count_sensor_ = sensor;
  }

  void set_moving_target_count_sensor(sensor::Sensor *sensor) {
    this->moving_target_count_sensor_ = sensor;
  }

  void set_still_target_count_sensor(sensor::Sensor *sensor) {
    this->still_target_count_sensor_ = sensor;
  }

  void set_presence_binary_sensor(binary_sensor::BinarySensor *sensor) {
    this->presence_binary_sensor_ = sensor;
  }

 protected:
  static constexpr uint8_t FRAME_SIZE = 30;

  uint8_t frame_[FRAME_SIZE];
  uint8_t frame_pos_{0};

  TargetData targets_[3];

  sensor::Sensor *target_x_sensors_[3]{nullptr, nullptr, nullptr};
  sensor::Sensor *target_y_sensors_[3]{nullptr, nullptr, nullptr};
  sensor::Sensor *target_distance_sensors_[3]{nullptr, nullptr, nullptr};
  sensor::Sensor *target_angle_sensors_[3]{nullptr, nullptr, nullptr};
  sensor::Sensor *target_speed_sensors_[3]{nullptr, nullptr, nullptr};
  sensor::Sensor *target_resolution_sensors_[3]{nullptr, nullptr, nullptr};

  text_sensor::TextSensor *target_direction_sensors_[3]{nullptr, nullptr, nullptr};

  sensor::Sensor *target_count_sensor_{nullptr};
  sensor::Sensor *moving_target_count_sensor_{nullptr};
  sensor::Sensor *still_target_count_sensor_{nullptr};

  binary_sensor::BinarySensor *presence_binary_sensor_{nullptr};

  uint8_t last_target_count_{255};
  uint8_t last_moving_target_count_{255};
  uint8_t last_still_target_count_{255};

  bool last_presence_{false};
  bool presence_initialized_{false};

  void process_byte_(uint8_t byte);
  void process_frame_();
  void publish_target_(uint8_t target);

  int16_t decode_signed_value_(uint8_t low, uint8_t high);

  TargetDirection determine_direction_(int16_t speed_mm_s);
  const char *direction_to_string_(TargetDirection direction);
};

}  // namespace ld2454
}  // namespace esphome