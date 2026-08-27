#pragma once

#include <cmath>
#include <cstddef>
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

enum class CommandTestState : uint8_t {
  IDLE = 0,

  WAIT_ENTER_ACK,
  WAIT_SEND_FIRMWARE,
  WAIT_FIRMWARE_ACK,
  WAIT_SEND_EXIT,
  WAIT_EXIT_ACK,

  DONE,
  FAILED
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


  // ---------------------------------------------------------------------------
  // Target sensors
  // ---------------------------------------------------------------------------

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

  void set_target_direction_sensor(
      uint8_t target,
      text_sensor::TextSensor *sensor) {
    if (target < 3)
      this->target_direction_sensors_[target] = sensor;
  }


  // ---------------------------------------------------------------------------
  // Firmware
  // ---------------------------------------------------------------------------

  void set_firmware_version_sensor(text_sensor::TextSensor *sensor) {
    this->firmware_version_sensor_ = sensor;
  }


  // ---------------------------------------------------------------------------
  // Global entities
  // ---------------------------------------------------------------------------

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
  // ===========================================================================
  // Tracking frame
  //
  // AA FF 03 00
  // Target 1: 8 bytes
  // Target 2: 8 bytes
  // Target 3: 8 bytes
  // 55 CC
  // ===========================================================================

  static constexpr uint8_t TRACKING_FRAME_SIZE = 30;

  uint8_t frame_[TRACKING_FRAME_SIZE];
  uint8_t frame_pos_{0};

  TargetData targets_[3];


  // ===========================================================================
  // Command / ACK parser
  //
  // FD FC FB FA
  // LEN_L LEN_H
  // payload
  // 04 03 02 01
  // ===========================================================================

  static constexpr uint16_t COMMAND_MAX_PAYLOAD_SIZE = 128;

  uint8_t command_header_pos_{0};

  uint8_t command_length_bytes_[2]{0, 0};
  uint8_t command_length_pos_{0};

  uint16_t command_payload_length_{0};
  uint16_t command_payload_pos_{0};

  uint8_t command_payload_[COMMAND_MAX_PAYLOAD_SIZE];

  uint8_t command_footer_pos_{0};

  bool command_receiving_{false};


  // ===========================================================================
  // Non-blocking command test
  // ===========================================================================

  CommandTestState command_test_state_{CommandTestState::IDLE};

  uint32_t setup_time_{0};

  uint32_t command_deadline_{0};

  uint32_t next_command_time_{0};

  bool command_test_started_{false};


  // ===========================================================================
  // ESPHome entities
  // ===========================================================================

  sensor::Sensor *target_x_sensors_[3]{
      nullptr, nullptr, nullptr};

  sensor::Sensor *target_y_sensors_[3]{
      nullptr, nullptr, nullptr};

  sensor::Sensor *target_distance_sensors_[3]{
      nullptr, nullptr, nullptr};

  sensor::Sensor *target_angle_sensors_[3]{
      nullptr, nullptr, nullptr};

  sensor::Sensor *target_speed_sensors_[3]{
      nullptr, nullptr, nullptr};

  sensor::Sensor *target_resolution_sensors_[3]{
      nullptr, nullptr, nullptr};

  text_sensor::TextSensor *target_direction_sensors_[3]{
      nullptr, nullptr, nullptr};

  text_sensor::TextSensor *firmware_version_sensor_{nullptr};


  sensor::Sensor *target_count_sensor_{nullptr};

  sensor::Sensor *moving_target_count_sensor_{nullptr};

  sensor::Sensor *still_target_count_sensor_{nullptr};


  binary_sensor::BinarySensor *presence_binary_sensor_{nullptr};


  // ===========================================================================
  // Published state
  // ===========================================================================

  uint8_t last_target_count_{255};

  uint8_t last_moving_target_count_{255};

  uint8_t last_still_target_count_{255};

  bool last_presence_{false};

  bool presence_initialized_{false};


  // ===========================================================================
  // UART / tracking
  // ===========================================================================

  void process_byte_(uint8_t byte);

  void process_tracking_byte_(uint8_t byte);

  void process_frame_();

  void publish_target_(uint8_t target);


  // ===========================================================================
  // Command parser
  // ===========================================================================

  void process_command_byte_(uint8_t byte);

  void process_command_frame_();

  void reset_command_parser_();


  // ===========================================================================
  // Command transmitter
  // ===========================================================================

  void send_command_(
      uint8_t command,
      const uint8_t *data = nullptr,
      size_t data_length = 0);

  void enter_config_mode_();

  void request_firmware_version_();

  void exit_config_mode_();


  // ===========================================================================
  // Non-blocking command test
  // ===========================================================================

  void start_command_test_();

  void process_command_test_();

  void process_command_test_response_(
      uint8_t command,
      uint8_t response,
      uint16_t status);


  // ===========================================================================
  // Helpers
  // ===========================================================================

  int16_t decode_signed_value_(
      uint8_t low,
      uint8_t high);

  TargetDirection determine_direction_(
      int16_t speed_mm_s);

  const char *direction_to_string_(
      TargetDirection direction);
};

}  // namespace ld2454
}  // namespace esphome