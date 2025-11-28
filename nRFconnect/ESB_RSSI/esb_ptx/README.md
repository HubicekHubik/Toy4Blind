# Enhanced ShockBurst TX Power & RSSI Test Sample

This project is based on Nordic's standard `esb_ptx` and `esb_prx` Zephyr samples and extends them to evaluate the impact of transmit power on RSSI values across various development boards.

## Overview

The application allows testing the full range of supported TX power levels on the PTX side and observes the received signal strength (RSSI) on the PRX side. Each transmitted packet includes metadata such as:

- **Board type**
- **Device ID**
- **TX power setting**
- **Flag to indicate end of test**

On the receiver side (PRX), each incoming packet logs:

- Sender device ID
- TX power setting
- RSSI in dBm
- Board name
- Link quality interpretation (e.g. *Weak*, *Average*, *Very Poor*)

Example output:
```
ID:3170380583 | Board:nRF54L15DK            | Counter:  5 | TX:  5 dBm | RSSI:-63 dBm (Average)
```

## Build Setup

Multiple builds can be created per board, such as:
- `build_54L15`
- `build_52840_FEM`
- `build_5340`
- etc.

### FEM Support (nRF21540)

To enable Front-End Module (FEM) support using the MPSL driver:

1. Add an additional Kconfig fragment:
   ```
   Extra Kconfig fragments: fem.conf
   ```

2. For boards using the **nRF21540-EK** shield, add:
   ```
   Extra CMake arguments: -DSHIELD=nrf21540ek
   ```

3. For **nRF21540-DK**, FEM support is already enabled via its default devicetree overlay.

This approach allows all builds to use a default  `prj.conf` and `fem.conf` as additional Kconfig fragment while customizing FEM settings per board.

## Default TX Power Settings

The PTX side cycles through the TX power settings from:

```c
min_tx_power = 0;   // -40 dBm
max_tx_power = 8;  // Up to 8 dBm supported directly
```

Unsupported values (e.g., 9 and 10) will be skipped with a console warning.

## Console Output Examples

### PTX (Transmitter)
```
Set TX power to: 4 dBm
Sending: TX SUCCESS
TX-Power = 4 dBm, FEM TX-Power = 0 dBm, Totaal TX-Power = 4 dBm
```

### PRX (Receiver)
```
ID:3170380583 | Board:nRF54L15DK | Counter:  4 | TX:  4 dBm | RSSI:-64 dBm (Average)
```

## Board Identification

Board types are determined via compile-time `#define` macros in `device_info.h`. This file maps `CONFIG_BOARD_*` and FEM configuration to a unique board ID:

```c
#elif defined(CONFIG_BOARD_NRF52840DK)
    #define BOARD_ID 0x02
#elif defined(CONFIG_BOARD_NRF54L15DK) && defined(CONFIG_MPSL_FEM)
    #define BOARD_ID 0x16
```

## Additional Features

### Repeat Mode

A new **Repeat Mode** is available to continuously send ESB packets at fixed TX power levels, intended for practical range testing. This mode can be enabled by setting:

```c
repeatModes = true;
```

In this mode:

- A packet is transmitted every second using the configured `repeat_tx_power`.
- Ideal for testing maximum communication range at a fixed power level.
- By default, the TX power is set to **8 dBm**, which is supported on **nRF52** and **nRF54** series.
- For **nRF53**, the maximum supported TX power is **0 dBm**, and `repeat_tx_power` should be adjusted accordingly.

### Bitrate Configuration

You can configure the ESB bitrate in `main(void)` by modifying the `bitrate` variable:

```c
bitrate = ESB_BITRATE_1MBPS;  // Default
bitrate = ESB_BITRATE_2MBPS;  // Optional
```

Only 1 Mbps and 2 Mbps are supported. Any invalid setting will default back to 1 Mbps with a console message.

### FEM TX Power Override

When **FEM support** is enabled (e.g., for nRF21540), the TX power is automatically overridden using the configured gain from `fem.conf`:

```c
esb_set_tx_power(CONFIG_MPSL_FEM_NRF21540_TX_GAIN_DB);
```

> ⚠️ **Note:** Although not documented by Nordic, this is currently the only working method to apply the FEM TX gain correctly. Without this override, your FEM will not achieve full transmission power.

## Requirements

- **nRF Connect SDK** v3.0.0 or newer
- One Nordic board as PTX, another as PRX
- Optional: nRF21540-EK or DK for FEM testing

## Reference

This project is derived from the official [nRF Connect SDK ESB samples](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/zephyr/samples/bluetooth/esb/README.html) and extended for TX power and RSSI analysis.

## Supported Boards

The following board types are currently supported and automatically identified in the receiver (PRX) firmware:

| Board ID | Name                      |
|----------|---------------------------|
| 0x01     | nRF21540DK                |
| 0x02     | nRF52840DK                |
| 0x03     | nRF5340DK                 |
| 0x04     | nRF5340_AUDIO_DK          |
| 0x05     | nRF7002DK                 |
| 0x06     | nRF54L15DK                |
| 0x11     | nRF21540DK FEM Active     |
| 0x12     | nRF52840DK FEM Active     |
| 0x13     | nRF5340DK FEM Active      |
| 0x14     | nRF5340AUDIO_DK FEM Active|
| 0x15     | nRF7002DK FEM Active      |
| 0x16     | nRF54L15DK FEM Active     |
| 0x21     | Raytac52DK                |
| 0x22     | Raytac54DK                |
| 0xFF     | Unknown                   |

The board ID is determined by macros in `device_info.h` (PTX) or by parsing the `board_id` field in the RX payload (PRX).
