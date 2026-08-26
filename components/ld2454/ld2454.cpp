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

  const bool positive = (raw & 0x8000) != 0;
  const int16_t value = raw & 0x7FFF;

  return positive ? value : -value;
}

TargetDirection LD2454Component::determine_direction_(
    int16_t speed_mm_s) {

  if (speed_mm_s > 0) {
    return TargetDirection::APPROACHING;
  }

  if (speed_mm_s < 0) {
    return TargetDirection::MOVING_AWAY;
  }

  return TargetDirection::STILL;
}

const char *LD2454Component::direction_to_string_(
    TargetDirection direction) {

  switch (direction) {
    case TargetDirection::APPROACHING:
      return "Approaching";

    case TargetDirection::MOVING_AWAY:
      return "Moving away";

    case TargetDirection::STILL:
      return "Still";

    case TargetDirection::NONE:
    default:
      return "None";
  }
}

void LD2454Component::process_frame_() {
  uint8_t target_count = 0;
  uint8_t moving_target_count = 0;
  uint8_t still_target_count = 0;

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
      data.speed_cm_s = 0;
      data.speed_mm_s = 0;
      data.resolution = 0;
      data.distance = 0.0f;
      data.angle = 0.0f;
      data.direction = TargetDirection::NONE;

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

    data.speed_cm_s = this->decode_signed_value_(
        this->frame_[offset + 4],
        this->frame_[offset + 5]);

    data.speed_mm_s =
        static_cast<int16_t>(data.speed_cm_s * 10);

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

    data.direction =
        this->determine_direction_(data.speed_mm_s);

    if (data.speed_mm_s == 0) {
      still_target_count++;
    } else {
      moving_target_count++;
    }

    this->publish_target_(target);
  }

  if (this->target_count_sensor_ != nullptr &&
      target_count != this->last_target_count_) {

    this->target_count_sensor_->publish_state(target_count);
    this->last_target_count_ = target_count;
  }

  if (this->moving_target_count_sensor_ != nullptr &&
      moving_target_count != this->last_moving_target_count_) {

    this->moving_target_count_sensor_->publish_state(
        moving_target_count);

    this->last_moving_target_count_ = moving_target_count;
  }

  if (this->still_target_count_sensor_ != nullptr &&
      still_target_count != this->last_still_target_count_) {

    this->still_target_count_sensor_->publish_state(
        still_target_count);

    this->last_still_target_count_ = still_target_count;
  }

  const bool presence = target_count > 0;

  if (this->presence_binary_sensor_ != nullptr &&
      (!this->presence_initialized_ ||
       presence != this->last_presence_)) {

    this->presence_binary_sensor_->publish_state(presence);

    this->last_presence_ = presence;
    this->presence_initialized_ = true;
  }
}

void LD2454Component::publish_target_(uint8_t target) {
  auto &data = this->targets_[target];

  // Target ist verschwunden:
  // Werte nur einmal auf NAN setzen.
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

      if (this->target_direction_sensors_[target] != nullptr)
        this->target_direction_sensors_[target]->publish_state("None");

      data.published_detected = false;
      data.published_direction = TargetDirection::NONE;
    }

    return;
  }

  // Ein vorhandenes Target wird bei jedem gültigen Radarframe publiziert.
  if (this->target_x_sensors_[target] != nullptr)
    this->target_x_sensors_[target]->publish_state(data.x);

  if (this->target_y_sensors_[target] != nullptr)
    this->target_y_sensors_[target]->publish_state(data.y);

  if (this->target_distance_sensors_[target] != nullptr)
    this->target_distance_sensors_[target]->publish_state(data.distance);

  if (this->target_angle_sensors_[target] != nullptr)
    this->target_angle_sensors_[target]->publish_state(data.angle);

  if (this->target_speed_sensors_[target] != nullptr)
    this->target_speed_sensors_[target]->publish_state(data.speed_mm_s);

  if (this->target_resolution_sensors_[target] != nullptr)
    this->target_resolution_sensors_[target]->publish_state(data.resolution);

  // Richtung nur veröffentlichen, wenn sie sich ändert.
  if (this->target_direction_sensors_[target] != nullptr &&
      (!data.published_detected ||
       data.direction != data.published_direction)) {

    this->target_direction_sensors_[target]->publish_state(
        this->direction_to_string_(data.direction));
  }

  data.published_detected = true;
  data.published_x = data.x;
  data.published_y = data.y;
  data.published_speed_mm_s = data.speed_mm_s;
  data.published_resolution = data.resolution;
  data.published_distance = data.distance;
  data.published_angle = data.angle;
  data.published_direction = data.direction;
}

void LD2454Component::dump_config() {
  ESP_LOGCONFIG(TAG, "LD2454:");
  ESP_LOGCONFIG(TAG, "  Tracking frame parser enabled");
  ESP_LOGCONFIG(TAG, "  Maximum targets: 3");

  this->check_uart_settings(256000);
}

}  // namespace ld2454
}  // namespace esphome