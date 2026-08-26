#include "ld2454.h"

#include <cmath>

#include "esphome/core/log.h"

namespace esphome {
namespace ld2454 {

static const char *const TAG = "ld2454";

void LD2454Component::setup() {
  ESP_LOGI(TAG, "LD2454 component started");
}

void LD2454Component::loop() {
  while (this->available()) {
    uint8_t byte;

    if (this->read_byte(&byte)) {
      this->process_byte_(byte);
    }
  }
}

void LD2454Component::process_byte_(uint8_t byte) {
  static const uint8_t HEADER[4] = {0xAA, 0xFF, 0x03, 0x00};

  if (this->frame_pos_ < 4) {
    if (byte == HEADER[this->frame_pos_]) {
      this->frame_[this->frame_pos_++] = byte;
    } else {
      if (byte == HEADER[0]) {
        this->frame_[0] = byte;
        this->frame_pos_ = 1;
      } else {
        this->frame_pos_ = 0;
      }
    }

    return;
  }

  this->frame_[this->frame_pos_++] = byte;

  if (this->frame_pos_ == FRAME_SIZE) {
    if (this->frame_[28] == 0x55 &&
        this->frame_[29] == 0xCC) {
      this->process_frame_();
    } else {
      ESP_LOGW(TAG, "Invalid LD2454 frame footer");
    }

    this->frame_pos_ = 0;
  }
}

int16_t LD2454Component::decode_signed_value_(
    uint8_t low,
    uint8_t high) {

  uint16_t raw =
      static_cast<uint16_t>(low) |
      (static_cast<uint16_t>(high) << 8);

  bool positive = (raw & 0x8000) != 0;
  int16_t value = raw & 0x7FFF;

  return positive ? value : -value;
}

void LD2454Component::process_frame_() {
  uint8_t target_count = 0;

  for (uint8_t target = 0; target < 3; target++) {
    const uint8_t offset = 4 + target * 8;

    bool empty = true;

    for (uint8_t i = 0; i < 8; i++) {
      if (this->frame_[offset + i] != 0x00) {
        empty = false;
        break;
      }
    }

    auto &data = this->targets_[target];

    data.detected = !empty;

    if (empty) {
      data.x = 0;
      data.y = 0;
      data.speed = 0;
      data.resolution = 0;
      data.distance = 0.0f;
      data.angle = 0.0f;

      this->publish_target_(target);
      continue;
    }

    target_count++;

    data.x = this->decode_signed_value_(
        this->frame_[offset],
        this->frame_[offset + 1]);

    data.y = this->decode_signed_value_(
        this->frame_[offset + 2],
        this->frame_[offset + 3]);

    data.speed = this->decode_signed_value_(
        this->frame_[offset + 4],
        this->frame_[offset + 5]);

    data.resolution =
        static_cast<uint16_t>(
            this->frame_[offset + 6]) |
        (static_cast<uint16_t>(
            this->frame_[offset + 7]) << 8);

    data.distance = std::sqrt(
        static_cast<float>(data.x) *
            static_cast<float>(data.x) +
        static_cast<float>(data.y) *
            static_cast<float>(data.y));

    data.angle =
        std::atan2(
            static_cast<float>(data.x),
            static_cast<float>(data.y)) *
        180.0f / M_PI;

    this->publish_target_(target);
  }

  if (this->target_count_sensor_ != nullptr &&
      target_count != this->last_target_count_) {
    this->target_count_sensor_->publish_state(target_count);
    this->last_target_count_ = target_count;
  }
}

void LD2454Component::publish_target_(uint8_t target) {
  auto &data = this->targets_[target];

  if (!data.detected) {
    if (data.published_detected) {
      if (this->target_x_sensors_[target] != nullptr)
        this->target_x_sensors_[target]->publish_state(NAN);

      if (this->target_y_sensors_[target] != nullptr)
        this->target_y_sensors_[target]->publish_state(NAN);

      if (this->target_distance_sensors_[target] != nullptr)
        this->target_distance_sensors_[target]->publish_state(NAN);

      if (this->target_angle_sensors_[target] != nullptr)
        this->target_angle_sensors_[target]->publish_state(NAN);

      if (this->target_speed_sensors_[target] != nullptr)
        this->target_speed_sensors_[target]->publish_state(NAN);

      if (this->target_resolution_sensors_[target] != nullptr)
        this->target_resolution_sensors_[target]->publish_state(NAN);

      data.published_detected = false;
    }

    return;
  }

  const bool changed =
      !data.published_detected ||
      data.x != data.published_x ||
      data.y != data.published_y ||
      data.speed != data.published_speed ||
      data.resolution != data.published_resolution ||
      std::fabs(data.distance - data.published_distance) >= 1.0f ||
      std::fabs(data.angle - data.published_angle) >= 0.1f;

  if (!changed) {
    return;
  }

  if (this->target_x_sensors_[target] != nullptr)
    this->target_x_sensors_[target]->publish_state(data.x);

  if (this->target_y_sensors_[target] != nullptr)
    this->target_y_sensors_[target]->publish_state(data.y);

  if (this->target_distance_sensors_[target] != nullptr)
    this->target_distance_sensors_[target]->publish_state(data.distance);

  if (this->target_angle_sensors_[target] != nullptr)
    this->target_angle_sensors_[target]->publish_state(data.angle);

  if (this->target_speed_sensors_[target] != nullptr)
    this->target_speed_sensors_[target]->publish_state(data.speed);

  if (this->target_resolution_sensors_[target] != nullptr)
    this->target_resolution_sensors_[target]->publish_state(data.resolution);

  data.published_detected = true;
  data.published_x = data.x;
  data.published_y = data.y;
  data.published_speed = data.speed;
  data.published_resolution = data.resolution;
  data.published_distance = data.distance;
  data.published_angle = data.angle;
}

void LD2454Component::dump_config() {
  ESP_LOGCONFIG(TAG, "LD2454:");
  ESP_LOGCONFIG(TAG, "  Custom ESPHome component");
  ESP_LOGCONFIG(TAG, "  Tracking frame parser enabled");

  this->check_uart_settings(256000);
}

}  // namespace ld2454
}  // namespace esphome