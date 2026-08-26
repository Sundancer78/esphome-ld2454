#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace ld2454 {

struct TargetData {
  bool detected{false};

  int16_t x{0};
  int16_t y{0};
  int16_t speed{0};
  uint16_t resolution{0};

  float distance{0.0f};
  float angle{0.0f};

  bool published_detected{false};
  int16_t published_x{0};
  int16_t published_y{0};
  int16_t published_speed{0};
  uint16_t published_resolution{0};
  float published_distance{0.0f};
  float published_angle{0.0f};
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

  void set_target_count_sensor(sensor::Sensor *sensor) {
    this->target_count_sensor_ = sensor;
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

  sensor::Sensor *target_count_sensor_{nullptr};
  
  uint8_t last_target_count_{255};

  void process_byte_(uint8_t byte);
  void process_frame_();
  void publish_target_(uint8_t target);

  int16_t decode_signed_value_(uint8_t low, uint8_t high);
};

}  // namespace ld2454
}  // namespace esphome