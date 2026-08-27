#include "ld2454.h"

#include <cmath>
#include <cstdio>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ld2454 {

static const char *const TAG = "ld2454";


// =============================================================================
// Setup
// =============================================================================

void LD2454Component::setup() {
  ESP_LOGI(TAG, "LD2454 component started");

  this->reset_command_parser_();

  this->setup_time_ = millis();
}


// =============================================================================
// Main loop
// =============================================================================

void LD2454Component::loop() {

  // ---------------------------------------------------------------------------
  // UART empfangen
  // ---------------------------------------------------------------------------

  while (this->available()) {
    uint8_t byte;

    if (this->read_byte(&byte)) {
      this->process_byte_(byte);
    }
  }


  // ---------------------------------------------------------------------------
  // Einmaliger Kommunikationstest nach 5 Sekunden
  // ---------------------------------------------------------------------------

  if (!this->command_test_started_ &&
      millis() - this->setup_time_ >= 5000) {

    this->command_test_started_ = true;

    this->start_command_test_();
  }


  // ---------------------------------------------------------------------------
  // Nichtblockierende Command-State-Machine
  // ---------------------------------------------------------------------------

  this->process_command_test_();
}


// =============================================================================
// UART dispatcher
// =============================================================================

void LD2454Component::process_byte_(uint8_t byte) {
  this->process_tracking_byte_(byte);

  this->process_command_byte_(byte);
}


// =============================================================================
// Tracking parser
// =============================================================================

void LD2454Component::process_tracking_byte_(uint8_t byte) {
  static const uint8_t HEADER[4] = {
      0xAA, 0xFF, 0x03, 0x00
  };


  // ---------------------------------------------------------------------------
  // Header suchen
  // ---------------------------------------------------------------------------

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


  // ---------------------------------------------------------------------------
  // Rest des Trackingframes sammeln
  // ---------------------------------------------------------------------------

  this->frame_[this->frame_pos_++] = byte;


  if (this->frame_pos_ == TRACKING_FRAME_SIZE) {

    if (this->frame_[28] == 0x55 &&
        this->frame_[29] == 0xCC) {

      this->process_frame_();

    } else {

      ESP_LOGW(
          TAG,
          "Invalid LD2454 tracking frame footer");
    }


    this->frame_pos_ = 0;
  }
}


// =============================================================================
// Command / ACK parser
// =============================================================================

void LD2454Component::process_command_byte_(uint8_t byte) {
  static const uint8_t HEADER[4] = {
      0xFD, 0xFC, 0xFB, 0xFA
  };

  static const uint8_t FOOTER[4] = {
      0x04, 0x03, 0x02, 0x01
  };


  // ---------------------------------------------------------------------------
  // Noch kein Commandframe aktiv:
  // Header FD FC FB FA suchen
  // ---------------------------------------------------------------------------

  if (!this->command_receiving_) {

    if (byte == HEADER[this->command_header_pos_]) {

      this->command_header_pos_++;


      if (this->command_header_pos_ == 4) {

        this->command_receiving_ = true;

        this->command_header_pos_ = 0;

        this->command_length_pos_ = 0;

        this->command_payload_length_ = 0;

        this->command_payload_pos_ = 0;

        this->command_footer_pos_ = 0;
      }

    } else {

      if (byte == HEADER[0]) {

        this->command_header_pos_ = 1;

      } else {

        this->command_header_pos_ = 0;
      }
    }


    return;
  }


  // ---------------------------------------------------------------------------
  // Payload-Länge lesen
  // ---------------------------------------------------------------------------

  if (this->command_length_pos_ < 2) {

    this->command_length_bytes_[
        this->command_length_pos_++] = byte;


    if (this->command_length_pos_ == 2) {

      this->command_payload_length_ =
          static_cast<uint16_t>(
              this->command_length_bytes_[0]) |
          (static_cast<uint16_t>(
              this->command_length_bytes_[1]) << 8);


      if (this->command_payload_length_ >
          COMMAND_MAX_PAYLOAD_SIZE) {

        ESP_LOGW(
            TAG,
            "Command payload too large: %u bytes",
            this->command_payload_length_);


        this->reset_command_parser_();
      }
    }


    return;
  }


  // ---------------------------------------------------------------------------
  // Payload lesen
  // ---------------------------------------------------------------------------

  if (this->command_payload_pos_ <
      this->command_payload_length_) {

    this->command_payload_[
        this->command_payload_pos_++] = byte;


    return;
  }


  // ---------------------------------------------------------------------------
  // Footer prüfen
  // ---------------------------------------------------------------------------

  if (byte == FOOTER[this->command_footer_pos_]) {

    this->command_footer_pos_++;


    if (this->command_footer_pos_ == 4) {

      this->process_command_frame_();

      this->reset_command_parser_();
    }


    return;
  }


  ESP_LOGW(
      TAG,
      "Invalid LD2454 command frame footer");


  this->reset_command_parser_();


  if (byte == HEADER[0]) {
    this->command_header_pos_ = 1;
  }
}


// =============================================================================
// Reset command parser
// =============================================================================

void LD2454Component::reset_command_parser_() {
  this->command_header_pos_ = 0;

  this->command_length_bytes_[0] = 0;
  this->command_length_bytes_[1] = 0;

  this->command_length_pos_ = 0;

  this->command_payload_length_ = 0;

  this->command_payload_pos_ = 0;

  this->command_footer_pos_ = 0;

  this->command_receiving_ = false;
}


// =============================================================================
// Complete command response
// =============================================================================

void LD2454Component::process_command_frame_() {

  ESP_LOGD(
      TAG,
      "LD2454 command response: payload=%u bytes",
      this->command_payload_length_);


  if (this->command_payload_length_ < 2) {

    ESP_LOGW(
        TAG,
        "Command response payload too short");

    return;
  }


  const uint8_t command =
      this->command_payload_[0];


  const uint8_t response =
      this->command_payload_[1];


  uint16_t status = 0xFFFF;


  if (this->command_payload_length_ >= 4) {

    status =
        static_cast<uint16_t>(
            this->command_payload_[2]) |
        (static_cast<uint16_t>(
            this->command_payload_[3]) << 8);
  }


  ESP_LOGD(
      TAG,
      "  Command: 0x%02X",
      command);


  ESP_LOGD(
      TAG,
      "  Response: 0x%02X",
      response);


  if (this->command_payload_length_ >= 4) {

    ESP_LOGD(
        TAG,
        "  Status: 0x%04X",
        status);
  }


  // ---------------------------------------------------------------------------
  // Kommandoabhängige Daten
  // ---------------------------------------------------------------------------

  if (this->command_payload_length_ > 4) {

    const uint16_t data_length =
        this->command_payload_length_ - 4;


    ESP_LOGD(
        TAG,
        "  Data length: %u bytes",
        data_length);


    // -------------------------------------------------------------------------
    // A0 = Firmwaredaten
    //
    // Noch bewusst RAW, bis das LD2454-Format sicher verstanden ist.
    // -------------------------------------------------------------------------

    if (command == 0xA0 &&
        response == 0x01 &&
        status == 0x0000) {

      char firmware_raw[64];

      size_t pos = 0;


      pos += snprintf(
          firmware_raw + pos,
          sizeof(firmware_raw) - pos,
          "RAW");


      for (uint16_t i = 0;
           i < data_length &&
           pos < sizeof(firmware_raw);
           i++) {

        const int written =
            snprintf(
                firmware_raw + pos,
                sizeof(firmware_raw) - pos,
                " %02X",
                this->command_payload_[4 + i]);


        if (written <= 0) {
          break;
        }


        if (static_cast<size_t>(written) >=
            sizeof(firmware_raw) - pos) {

          break;
        }


        pos += static_cast<size_t>(written);
      }


      ESP_LOGI(
          TAG,
          "LD2454 firmware raw data: %s",
          firmware_raw);


      if (this->firmware_version_sensor_ != nullptr) {

        this->firmware_version_sensor_->publish_state(
            firmware_raw);
      }
    }
  }


  // ---------------------------------------------------------------------------
  // Antwort an State-Machine weitergeben
  // ---------------------------------------------------------------------------

  this->process_command_test_response_(
      command,
      response,
      status);
}


// =============================================================================
// Generic command transmitter
// =============================================================================

void LD2454Component::send_command_(
    uint8_t command,
    const uint8_t *data,
    size_t data_length) {

  static const uint8_t HEADER[4] = {
      0xFD, 0xFC, 0xFB, 0xFA
  };


  static const uint8_t FOOTER[4] = {
      0x04, 0x03, 0x02, 0x01
  };


  if (data_length > 32) {

    ESP_LOGE(
        TAG,
        "TX command data too large");

    return;
  }


  const uint16_t payload_length =
      static_cast<uint16_t>(
          2 + data_length);


  uint8_t buffer[
      4 + 2 + 2 + 32 + 4];


  size_t pos = 0;


  // Header

  for (uint8_t value : HEADER) {
    buffer[pos++] = value;
  }


  // Payload length

  buffer[pos++] =
      payload_length & 0xFF;


  buffer[pos++] =
      (payload_length >> 8) & 0xFF;


  // Command

  buffer[pos++] = command;

  buffer[pos++] = 0x00;


  // Optional data

  for (size_t i = 0;
       i < data_length;
       i++) {

    buffer[pos++] = data[i];
  }


  // Footer

  for (uint8_t value : FOOTER) {
    buffer[pos++] = value;
  }


  ESP_LOGD(
      TAG,
      "TX command 0x%02X (%u byte payload)",
      command,
      payload_length);


  // ---------------------------------------------------------------------------
  // Wichtig:
  //
  // Kein flush() mehr.
  //
  // Hardware-UART übernimmt die Übertragung im Hintergrund.
  // ---------------------------------------------------------------------------

  this->write_array(
      buffer,
      pos);
}


// =============================================================================
// Enter configuration mode
// =============================================================================

void LD2454Component::enter_config_mode_() {

  static const uint8_t DATA[2] = {
      0x01, 0x00
  };


  ESP_LOGI(
      TAG,
      "Entering LD2454 configuration mode");


  this->send_command_(
      0xFF,
      DATA,
      sizeof(DATA));
}


// =============================================================================
// Firmware request
// =============================================================================

void LD2454Component::request_firmware_version_() {

  ESP_LOGI(
      TAG,
      "Requesting LD2454 firmware version");


  this->send_command_(
      0xA0);
}


// =============================================================================
// Exit configuration mode
// =============================================================================

void LD2454Component::exit_config_mode_() {

  ESP_LOGI(
      TAG,
      "Leaving LD2454 configuration mode");


  this->send_command_(
      0xFE);
}


// =============================================================================
// Start automatic communication test
// =============================================================================

void LD2454Component::start_command_test_() {

  ESP_LOGI(
      TAG,
      "Starting LD2454 command communication test");


  this->command_test_state_ =
      CommandTestState::WAIT_ENTER_ACK;


  this->command_deadline_ =
      millis() + 1500;


  this->enter_config_mode_();
}


// =============================================================================
// Non-blocking command state machine
// =============================================================================

void LD2454Component::process_command_test_() {

  const uint32_t now =
      millis();


  // ---------------------------------------------------------------------------
  // FF wurde bestätigt.
  // A0 zeitversetzt senden.
  // ---------------------------------------------------------------------------

  if (this->command_test_state_ ==
      CommandTestState::WAIT_SEND_FIRMWARE) {

    if (static_cast<int32_t>(
            now - this->next_command_time_) >= 0) {

      this->command_test_state_ =
          CommandTestState::WAIT_FIRMWARE_ACK;


      this->command_deadline_ =
          now + 1500;


      this->request_firmware_version_();
    }


    return;
  }


  // ---------------------------------------------------------------------------
  // A0 wurde bestätigt.
  // FE zeitversetzt senden.
  // ---------------------------------------------------------------------------

  if (this->command_test_state_ ==
      CommandTestState::WAIT_SEND_EXIT) {

    if (static_cast<int32_t>(
            now - this->next_command_time_) >= 0) {

      this->command_test_state_ =
          CommandTestState::WAIT_EXIT_ACK;


      this->command_deadline_ =
          now + 1500;


      this->exit_config_mode_();
    }


    return;
  }


  // ---------------------------------------------------------------------------
  // ACK timeout
  // ---------------------------------------------------------------------------

  if (this->command_test_state_ ==
          CommandTestState::WAIT_ENTER_ACK ||
      this->command_test_state_ ==
          CommandTestState::WAIT_FIRMWARE_ACK ||
      this->command_test_state_ ==
          CommandTestState::WAIT_EXIT_ACK) {

    if (static_cast<int32_t>(
            now - this->command_deadline_) >= 0) {

      ESP_LOGW(
          TAG,
          "LD2454 command test timeout");


      this->command_test_state_ =
          CommandTestState::FAILED;
    }
  }
}


// =============================================================================
// Command response state machine
// =============================================================================

void LD2454Component::process_command_test_response_(
    uint8_t command,
    uint8_t response,
    uint16_t status) {

  if (response != 0x01) {

    ESP_LOGW(
        TAG,
        "Unexpected command response byte: 0x%02X",
        response);

    return;
  }


  if (status != 0x0000) {

    ESP_LOGW(
        TAG,
        "LD2454 command 0x%02X returned status 0x%04X",
        command,
        status);


    this->command_test_state_ =
        CommandTestState::FAILED;


    return;
  }


  switch (this->command_test_state_) {


    // -------------------------------------------------------------------------
    // FF acknowledged
    // -------------------------------------------------------------------------

    case CommandTestState::WAIT_ENTER_ACK:

      if (command != 0xFF) {
        return;
      }


      ESP_LOGI(
          TAG,
          "LD2454 configuration mode acknowledged");


      // Nicht direkt A0 senden.
      // Erst 25 ms später in loop().

      this->command_test_state_ =
          CommandTestState::WAIT_SEND_FIRMWARE;


      this->next_command_time_ =
          millis() + 25;


      break;


    // -------------------------------------------------------------------------
    // A0 acknowledged
    // -------------------------------------------------------------------------

    case CommandTestState::WAIT_FIRMWARE_ACK:

      if (command != 0xA0) {
        return;
      }


      ESP_LOGI(
          TAG,
          "LD2454 firmware response received successfully");


      // Nicht direkt FE senden.
      // Erst 25 ms später in loop().

      this->command_test_state_ =
          CommandTestState::WAIT_SEND_EXIT;


      this->next_command_time_ =
          millis() + 25;


      break;


    // -------------------------------------------------------------------------
    // FE acknowledged
    // -------------------------------------------------------------------------

    case CommandTestState::WAIT_EXIT_ACK:

      if (command != 0xFE) {
        return;
      }


      ESP_LOGI(
          TAG,
          "LD2454 configuration mode closed");


      this->command_test_state_ =
          CommandTestState::DONE;


      ESP_LOGI(
          TAG,
          "LD2454 command communication test PASSED");


      break;


    default:
      break;
  }
}


// =============================================================================
// Signed LD2454 value decoder
// =============================================================================

int16_t LD2454Component::decode_signed_value_(
    uint8_t low,
    uint8_t high) {

  uint16_t raw =
      static_cast<uint16_t>(low) |
      (static_cast<uint16_t>(high) << 8);


  const bool positive =
      (raw & 0x8000) != 0;


  const int16_t value =
      raw & 0x7FFF;


  return positive ? value : -value;
}


// =============================================================================
// Direction
// =============================================================================

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


// =============================================================================
// Direction text
// =============================================================================

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


// =============================================================================
// Process tracking frame
// =============================================================================

void LD2454Component::process_frame_() {

  uint8_t target_count = 0;

  uint8_t moving_target_count = 0;

  uint8_t still_target_count = 0;


  for (uint8_t target = 0;
       target < 3;
       target++) {

    const uint8_t offset =
        4 + target * 8;


    bool empty = true;


    for (uint8_t i = 0;
         i < 8;
         i++) {

      if (this->frame_[offset + i] != 0x00) {

        empty = false;

        break;
      }
    }


    auto &data =
        this->targets_[target];


    data.detected =
        !empty;


    // -------------------------------------------------------------------------
    // Empty target
    // -------------------------------------------------------------------------

    if (empty) {

      data.x = 0;

      data.y = 0;

      data.speed_cm_s = 0;

      data.speed_mm_s = 0;

      data.resolution = 0;

      data.distance = 0.0f;

      data.angle = 0.0f;

      data.direction =
          TargetDirection::NONE;


      this->publish_target_(target);


      continue;
    }


    // -------------------------------------------------------------------------
    // Target present
    // -------------------------------------------------------------------------

    target_count++;


    data.x =
        this->decode_signed_value_(
            this->frame_[offset],
            this->frame_[offset + 1]);


    data.y =
        this->decode_signed_value_(
            this->frame_[offset + 2],
            this->frame_[offset + 3]);


    data.speed_cm_s =
        this->decode_signed_value_(
            this->frame_[offset + 4],
            this->frame_[offset + 5]);


    data.speed_mm_s =
        static_cast<int16_t>(
            data.speed_cm_s * 10);


    data.resolution =
        static_cast<uint16_t>(
            this->frame_[offset + 6]) |
        (static_cast<uint16_t>(
            this->frame_[offset + 7]) << 8);


    data.distance =
        std::sqrt(
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
        this->determine_direction_(
            data.speed_mm_s);


    if (data.speed_mm_s == 0) {

      still_target_count++;

    } else {

      moving_target_count++;
    }


    this->publish_target_(target);
  }


  // ---------------------------------------------------------------------------
  // Target count
  // ---------------------------------------------------------------------------

  if (this->target_count_sensor_ != nullptr &&
      target_count !=
          this->last_target_count_) {

    this->target_count_sensor_->
        publish_state(target_count);


    this->last_target_count_ =
        target_count;
  }


  // ---------------------------------------------------------------------------
  // Moving targets
  // ---------------------------------------------------------------------------

  if (this->moving_target_count_sensor_ != nullptr &&
      moving_target_count !=
          this->last_moving_target_count_) {

    this->moving_target_count_sensor_->
        publish_state(
            moving_target_count);


    this->last_moving_target_count_ =
        moving_target_count;
  }


  // ---------------------------------------------------------------------------
  // Still targets
  // ---------------------------------------------------------------------------

  if (this->still_target_count_sensor_ != nullptr &&
      still_target_count !=
          this->last_still_target_count_) {

    this->still_target_count_sensor_->
        publish_state(
            still_target_count);


    this->last_still_target_count_ =
        still_target_count;
  }


  // ---------------------------------------------------------------------------
  // Presence
  // ---------------------------------------------------------------------------

  const bool presence =
      target_count > 0;


  if (this->presence_binary_sensor_ != nullptr &&
      (!this->presence_initialized_ ||
       presence != this->last_presence_)) {

    this->presence_binary_sensor_->
        publish_state(
            presence);


    this->last_presence_ =
        presence;


    this->presence_initialized_ =
        true;
  }
}


// =============================================================================
// Publish target
// =============================================================================

void LD2454Component::publish_target_(
    uint8_t target) {

  auto &data =
      this->targets_[target];


  // ---------------------------------------------------------------------------
  // Target disappeared
  // ---------------------------------------------------------------------------

  if (!data.detected) {

    if (data.published_detected) {

      if (this->target_x_sensors_[target] != nullptr) {

        this->target_x_sensors_[target]->
            publish_state(NAN);
      }


      if (this->target_y_sensors_[target] != nullptr) {

        this->target_y_sensors_[target]->
            publish_state(NAN);
      }


      if (this->target_distance_sensors_[target] != nullptr) {

        this->target_distance_sensors_[target]->
            publish_state(NAN);
      }


      if (this->target_angle_sensors_[target] != nullptr) {

        this->target_angle_sensors_[target]->
            publish_state(NAN);
      }


      if (this->target_speed_sensors_[target] != nullptr) {

        this->target_speed_sensors_[target]->
            publish_state(NAN);
      }


      if (this->target_resolution_sensors_[target] != nullptr) {

        this->target_resolution_sensors_[target]->
            publish_state(NAN);
      }


      if (this->target_direction_sensors_[target] != nullptr) {

        this->target_direction_sensors_[target]->
            publish_state("None");
      }


      data.published_detected =
          false;


      data.published_direction =
          TargetDirection::NONE;
    }


    return;
  }


  // ---------------------------------------------------------------------------
  // Active target
  //
  // Wichtig:
  // weiterhin JEDEN gültigen Radarframe veröffentlichen.
  // ---------------------------------------------------------------------------

  if (this->target_x_sensors_[target] != nullptr) {

    this->target_x_sensors_[target]->
        publish_state(data.x);
  }


  if (this->target_y_sensors_[target] != nullptr) {

    this->target_y_sensors_[target]->
        publish_state(data.y);
  }


  if (this->target_distance_sensors_[target] != nullptr) {

    this->target_distance_sensors_[target]->
        publish_state(
            data.distance);
  }


  if (this->target_angle_sensors_[target] != nullptr) {

    this->target_angle_sensors_[target]->
        publish_state(
            data.angle);
  }


  if (this->target_speed_sensors_[target] != nullptr) {

    this->target_speed_sensors_[target]->
        publish_state(
            data.speed_mm_s);
  }


  if (this->target_resolution_sensors_[target] != nullptr) {

    this->target_resolution_sensors_[target]->
        publish_state(
            data.resolution);
  }


  // ---------------------------------------------------------------------------
  // Direction nur bei Änderung
  // ---------------------------------------------------------------------------

  if (this->target_direction_sensors_[target] != nullptr &&
      (!data.published_detected ||
       data.direction !=
           data.published_direction)) {

    this->target_direction_sensors_[target]->
        publish_state(
            this->direction_to_string_(
                data.direction));
  }


  data.published_detected =
      true;


  data.published_x =
      data.x;


  data.published_y =
      data.y;


  data.published_speed_mm_s =
      data.speed_mm_s;


  data.published_resolution =
      data.resolution;


  data.published_distance =
      data.distance;


  data.published_angle =
      data.angle;


  data.published_direction =
      data.direction;
}


// =============================================================================
// Dump configuration
// =============================================================================

void LD2454Component::dump_config() {

  ESP_LOGCONFIG(
      TAG,
      "LD2454:");


  ESP_LOGCONFIG(
      TAG,
      "  Tracking frame parser enabled");


  ESP_LOGCONFIG(
      TAG,
      "  Command response parser enabled");


  ESP_LOGCONFIG(
      TAG,
      "  Command transmitter enabled");


  ESP_LOGCONFIG(
      TAG,
      "  Non-blocking command state machine enabled");


  ESP_LOGCONFIG(
      TAG,
      "  Automatic firmware test enabled");


  ESP_LOGCONFIG(
      TAG,
      "  Maximum targets: 3");


  this->check_uart_settings(
      256000);
}

}  // namespace ld2454
}  // namespace esphome