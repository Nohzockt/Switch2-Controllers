# NS2 Bluetooth Gamepad (Virtual Xbox 360 Mapper)

Current version of the ns2-ble-cgamepad.py is not working the "fix" doesn't work right now use older version(uploaded as ns2-bleak-022.2-vgamepad) before bleak 1.0 fix commit with bleak version 22.2

A Python tool to scan, connect, and monitor Nintendo Switch-compatible controllers (Pro Controller, Joy-Con, GameCube Controller) via Bluetooth, and map their input to a virtual Xbox 360 gamepad (using [vgamepad](https://github.com/yannbouteiller/vgamepad) and ViGEmBus on Windows). Designed for real-time controller-to-Xbox mapping, stick and button monitoring, and feature exploration using the [bleak](https://github.com/hbldh/bleak) Bluetooth library.<br>
**Platform:** Windows only (for virtual gamepad support).

Some functions noted here may not be working right now so sorry 

> **Note:** This repository is intended as a universal base for all future tools and programs related to Nintendo Switch and compatible Bluetooth controllers. Contributions and extensions are welcome.

> **This repository contains portions of code or information derived from sources licensed as follows:**  
> **Copyright (c) 2025, Jacques Gagnon**  
> **SPDX-License-Identifier: Apache-2.0**

## Features

- **Bluetooth Device Scanning**: Discover Nintendo Switch-compatible controllers.
- **Controller-to-Xbox Mapping**: Button presses and analog sticks are mapped live to a virtual Xbox 360 controller (via ViGEmBus).
- **Real-time Monitoring**: See button presses and stick movement as Xbox360 input.
- **Simple Usage**: No configuration needed for supported controllers.
- **Debug Output**: Optional debug/verbose output for troubleshooting.
- **Open for Extensions**: Designed as a base for further Python tools for Nintendo and compatible Bluetooth controllers.

## Supported Controllers

- Nintendo Switch Pro Controller(not tested)
- Nintendo Switch Joy-Con (L/R, only separately, not tested)
- Nintendo GameCube Controller

## Quick Start

### Requirements

- Windows 10/11
- Python 3.7+ (tested in 3.12)
- [bleak](https://pypi.org/project/bleak/) (`pip install bleak` tested in 0.22.2)
- [vgamepad](https://pypi.org/project/vgamepad/) (`pip install vgamepad` tested in 0.1.0)
- [ViGEmBus](https://vigembusdriver.com) driver installed

### Usage

```bash
python3 ns2-ble-cgamepad.py [options]
```

**Options:**

- `-d`, `--debug` Enable debug output
- `-v`, `--verbose` Enable verbose output

### Controls

- `Ctrl+C` Exit

> **Note:** Interactive controls (rumble, LED, raw data, etc.) are not available in this version. This tool focuses on gamepad mapping and live Xbox 360 input.

## Pairing Instructions

1. **Put controller in pairing mode:**
   - Pro Controller: Hold the small pairing button on the top
   - Joy-Con: Hold the pairing button on the side
   - GameCube Controller: Hold the pairing button on the adapter
2. Ensure the controller is not already connected to another device.
3. Run the program and follow the on-screen instructions.

## Roadmap & Contribution

- Add support for combined Joy-Con mode
- Optional: More interactive features (LED)
- Cross-platform support if virtual driver alternatives exist

Contributions, issues, and feature requests are welcome!

## License

MIT License  
Parts of this repository are derived from sources licensed under Apache-2.0 (see notices above).

---

**This repository is for experimentation, learning, and controller interfacing. It is not affiliated with or endorsed by Nintendo.**
