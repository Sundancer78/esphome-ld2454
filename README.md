# ESPHome LD2454

Experimental ESPHome external component for the **Hi-Link LD2454 24 GHz multi-target tracking radar**.

> ⚠️ **Work in progress**
>
> This component is currently under active development and real-world testing.
> Configuration options, entities and behavior may still change.

## Current Features

- Presence detection
- Radar online status
- Multi-target tracking
- Target count
- Moving / still target count
- Target X / Y position
- Target distance
- Target angle
- Target speed
- Target resolution
- Target direction
- Single / Multi Target mode
- Target mode readback and verification
- Firmware version readout
- Restart
- Factory reset
- Command communication test

## Tested Hardware

The component is currently being tested with:

- ESP32-S3-DevKitC-1 N16R8
- Seeed Studio XIAO ESP32-C6
- ESP32-WROOM-32
- ESP32-WROOM-32D
- Hi-Link LD2454

Other ESP32 variants may also work, but have not yet been tested.

## Installation

The component can be loaded directly from GitHub using ESPHome `external_components`.

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/Sundancer78/esphome-ld2454
      ref: main
    components: [ld2454]
```

Then configure a UART interface for the radar:

```yaml
uart:
  id: ld2454_uart
  tx_pin: GPIO17
  rx_pin: GPIO16
  baud_rate: 256000
  data_bits: 8
  parity: NONE
  stop_bits: 1

ld2454:
  id: radar
  uart_id: ld2454_uart
```

> ⚠️ **Important: Do not use UART0 for the LD2454.**
>
> UART0 may be used for programming, logging or other system functions depending
> on the ESP32 board and configuration. Use another available hardware UART.

The GPIO numbers shown above are only an example. Select suitable UART pins for
your ESP32 board.

## Wiring

| LD2454 | ESP32 |
|--------|-------|
| 5V     | 5V    |
| GND    | GND   |
| TX     | RX    |
| RX     | TX    |

UART TX and RX must be crossed:

- **LD2454 TX → ESP32 RX**
- **LD2454 RX → ESP32 TX**

A common ground between the ESP32 and LD2454 is required.

## Example Configuration

```yaml
esphome:
  name: ld2454-test
  friendly_name: LD2454 Test

external_components:
  - source:
      type: git
      url: https://github.com/Sundancer78/esphome-ld2454
      ref: main
    components: [ld2454]

esp32:
  board: esp32dev
  framework:
    type: esp-idf

logger:
  level: DEBUG

wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password

api:

ota:
  - platform: esphome

uart:
  id: ld2454_uart
  tx_pin: GPIO17
  rx_pin: GPIO16
  baud_rate: 256000
  data_bits: 8
  parity: NONE
  stop_bits: 1

ld2454:
  id: radar
  uart_id: ld2454_uart

sensor:
  - platform: ld2454
    ld2454_id: radar

    target_count:
      name: "LD2454 Target Count"

    moving_target_count:
      name: "LD2454 Moving Targets"

    still_target_count:
      name: "LD2454 Still Targets"

    target_1_x:
      name: "LD2454 Target 1 X"

    target_1_y:
      name: "LD2454 Target 1 Y"

    target_1_distance:
      name: "LD2454 Target 1 Distance"

    target_1_angle:
      name: "LD2454 Target 1 Angle"

    target_1_speed:
      name: "LD2454 Target 1 Speed"

    target_1_resolution:
      name: "LD2454 Target 1 Resolution"

    target_2_x:
      name: "LD2454 Target 2 X"

    target_2_y:
      name: "LD2454 Target 2 Y"

    target_2_distance:
      name: "LD2454 Target 2 Distance"

    target_2_angle:
      name: "LD2454 Target 2 Angle"

    target_2_speed:
      name: "LD2454 Target 2 Speed"

    target_2_resolution:
      name: "LD2454 Target 2 Resolution"

    target_3_x:
      name: "LD2454 Target 3 X"

    target_3_y:
      name: "LD2454 Target 3 Y"

    target_3_distance:
      name: "LD2454 Target 3 Distance"

    target_3_angle:
      name: "LD2454 Target 3 Angle"

    target_3_speed:
      name: "LD2454 Target 3 Speed"

    target_3_resolution:
      name: "LD2454 Target 3 Resolution"

binary_sensor:
  - platform: ld2454
    ld2454_id: radar

    presence:
      name: "LD2454 Presence"

    online:
      name: "LD2454 Status"

text_sensor:
  - platform: ld2454
    ld2454_id: radar

    target_1_direction:
      name: "LD2454 Target 1 Direction"

    target_2_direction:
      name: "LD2454 Target 2 Direction"

    target_3_direction:
      name: "LD2454 Target 3 Direction"

    firmware_version:
      name: "LD2454 Firmware"

button:
  - platform: ld2454
    ld2454_id: radar

    restart:
      name: "LD2454 Restart"
      icon: mdi:power-cycle

    factory_reset:
      name: "LD2454 Factory Reset"

switch:
  - platform: ld2454
    ld2454_id: radar

    multi_target:
      name: "LD2454 Multi Target"
```

## UART Settings

The LD2454 uses:

- **256000 baud**
- **8 data bits**
- **No parity**
- **1 stop bit**

The default radar baud rate is 256000.

Baud-rate changes are supported internally by the component, but are not
exposed as a normal Home Assistant entity.

## Current Status

The component is currently being tested with multiple LD2454 sensors and
different ESP32 boards.

Current focus:

- Stable continuous tracking
- Long-term UART stability
- Reliable command communication
- Single / Multi Target behavior
- Real-world comparison between multiple LD2454 modules

Target mode after restart: The LD2454 may return to Single Target mode after a radar restart or factory reset. The component re-reads the actual target mode after the radar comes back online and updates the switch accordingly.

This project should currently be considered **experimental**.

Bug reports, test results and observations are welcome.