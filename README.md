This simulator is built for the rapid development of [Flexisender](https://github.com/bisonactual/Flexisender2). You can use it alongside flexisender to run a simulated env. stdin is available, so you can simulate physical triggers which show up in Flexisender. You can also use it to test any other sender. 

Connection is only over websocket. Web stack is not simulated, it's just existent to connect.

Currently has a hard build of the dev branch atc plugin from expatria included. You can rapidly add and test your small plugins by pushing them to the rapid plugin builder - it recompiles to add your plugin on the fly. I use this on WSL in windows, but it should work on windows/linux. Give it to a clanker and it'll sort out minor issues. 

SD Card is mounted in /build/sdcard and provides 1gb for various shenanigans. EEPROM is where the 'hardware memory' lives. 

Clanker output below :)

# grblHAL FlexiHAL Simulator

A cross-platform simulator for CNC controllers running [grblHAL](https://github.com/grblHAL) firmware. Compiles the real grblHAL core with a virtual HAL driver that emulates the [Expatria FlexiHAL](https://github.com/Expatria-Technologies/Flexi-HAL) controller.

Connect any G-code sender via WebSocket. Upload firmware via drag-and-drop UF2. Add grblHAL plugins through a browser UI.

## What it does

- Runs the **real grblHAL C code** on your PC — not an emulation, the actual motion planner, G-code parser, and state machine
- Emulates a FlexiHAL board: 5 axes (X/Y/Z + 2 ABC), PWM spindle, flood/mist coolant, probe, safety door, E-stop
- **WebSocket server** so any sender (CNCjs, UGS, gSender, etc.) can connect
- **UF2 firmware upload** via browser drag-and-drop
- **Plugin manager** — drop in grblHAL plugin `.c` files, rebuild from the browser
- Board-specific guards are automatically stripped so plugins work in simulation
- Persistent settings via EEPROM file
- Keyboard-driven hardware event simulation (limits, probe, e-stop, etc.)

## Quick start

### Prerequisites

**Linux / WSL (Ubuntu):**
```bash
sudo apt-get install build-essential cmake git
```

**Windows (native):**
- [Git](https://git-scm.com/download/win)
- [CMake](https://cmake.org/download/) (add to PATH during install)
- [Visual Studio Build Tools 2022](https://visualstudio.microsoft.com/downloads/) — select "Desktop development with C++"

### Clone & build

```bash
git clone --recurse-submodules https://github.com/Expatria-Technologies/grblhal-sim.git
cd grblhal-sim
mkdir build && cd build
cmake ..
make        # Linux/WSL
# or: cmake --build . --config Release    # Windows
```

### Run

```bash
./grblHAL_flexihal_sim
```

You'll see:
```
=== grblHAL FlexiHAL Simulator ===
Emulating: Expatria Flexi-HAL (STM32F446)
Axes: 5 (X, Y, Z + 2 ABC motors)

[WS] Listening on ws://0.0.0.0:8080
[HTTP] Management page at http://localhost:8081

Ready. Connect your sender to ws://localhost:8080
Manage firmware & plugins at http://localhost:8081
```

### Connect your sender

Point your sender's connection to:
```
ws://localhost:8080
```

### Upload firmware

Open `http://localhost:8081` in your browser. Drag a `.uf2` file onto the Firmware tab.

FlexiHAL firmware can be built from: https://github.com/Expatria-Technologies/STM32F4xx (branch `F446_Flexi_HAL`)

Or download pre-built `.uf2` files from the [FlexiHAL releases](https://github.com/Expatria-Technologies/Flexi-HAL/releases).

### Add plugins

1. Open `http://localhost:8081` and click the **Plugins** tab
2. Drag in a grblHAL plugin `.c` file
3. Click **Rebuild & Restart Simulator**
4. Restart the simulator to load the new plugin

Board-specific `#if defined(BOARD_XXX)` guards are automatically removed so plugins compile for the simulator. Hardware I/O calls (`DIGITAL_IN`, `AUXINPUT` ports, etc.) are stubbed to safe defaults.

## Command-line options

```
-w <port>   WebSocket port (default: 8080)
-u <port>   HTTP management port (default: 8081)
-t <speed>  Realtime speed multiplier (default: 1.0, 0 = max speed)
-e <file>   EEPROM settings file (default: EEPROM.DAT)
-r <time>   Step report interval in seconds (0 = off)
-n          No comment prefix on serial output
```

## Keyboard controls

When a sender is connected via WebSocket, stdin is free for hardware simulation:

| Key | Action |
|-----|--------|
| `e` | Toggle E-Stop |
| `r` | Toggle Reset |
| `h` | Toggle Feed Hold |
| `s` | Toggle Cycle Start |
| `d` | Toggle Safety Door |
| `p` | Toggle Probe |
| `o` | Toggle Probe Connected |
| `x` `y` `z` | Toggle axis limit switches |
| `?` | Request status report |
| Ctrl+C | Stop simulator |

## Building with plugins via cmake

Instead of the web UI, you can enable plugins at build time:

```bash
cmake .. -DPLUGIN_EXCLUSION_ZONES=ON
make
```

## Project structure

```
grblhal-sim/
├── CMakeLists.txt
├── src/
│   ├── grbl/               # grblHAL core (git submodule)
│   ├── driver.c/h          # FlexiHAL virtual HAL driver
│   ├── main.c              # Entry point, WebSocket + HTTP setup
│   ├── simulator.c/h       # Simulation tick engine
│   ├── mcu.c/h             # Virtual MCU (timers, GPIO, UART)
│   ├── serial.c/h          # UART ring buffer emulation
│   ├── eeprom.c/h          # NVS persistence to file
│   ├── websocket.c/h       # RFC 6455 WebSocket server
│   ├── uf2.c/h             # UF2 parser + plugin manager + HTTP UI
│   ├── sim_stubs.h         # Hardware macro stubs for plugins
│   ├── my_plugin.c         # Auto-generated plugin loader
│   └── platform_*.c/h      # OS abstraction (Linux/Windows)
└── plugins/                 # Drop plugin .c files here (auto-discovered)
```

## License

GPLv3 — same as grblHAL.

Based on the [grblHAL Simulator](https://github.com/grblHAL/Simulator) by Terje Io, Jens Geisler, and Adam Shelly.
