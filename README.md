# grblHAL FlexiHAL Simulator

A cross-platform simulator for the FlexiHAL grblHAL controller profile used by Expatria. It compiles the real grblHAL core with a virtual HAL driver and exposes the controller to G-code senders over WebSocket.

This is a fixed simulator build, not a runtime firmware uploader. The bundled profile currently targets an Expatria Flexi-HAL style STM32F446 controller with 5 axes, PWM spindle, flood/mist coolant, probe, safety door, E-stop, simulated EEPROM, simulated SD card, and the active bundled simulation plugins: Sienci ATCi and Exclusion Zones.

It is used for Flexisender development, but it can also be used with other WebSocket-capable G-code senders.

## What It Does

- Runs the real grblHAL parser, planner, state machine, settings, alarms, and reports on a PC.
- Emulates the FlexiHAL hardware surface used by the current simulator profile.
- Lets senders such as gSender, CNCjs, and UGS connect to `ws://localhost:8080`.
- Keeps simulator controls local through stdin, so the sender WebSocket remains sender-only.
- Persists settings in an EEPROM file.
- Provides a host-backed `sdcard/` folder for grblHAL VFS and tool table style workflows.
- Includes a native Windows launcher for start/stop, port settings, persisted options, controller-state display, log viewing, and simulator input buttons.

## Quick Start

### Linux

Install dependencies:

```bash
sudo apt-get install build-essential cmake git
```

Build and run:

```bash
git clone --recurse-submodules https://github.com/bisonactual/GRBLHAL-Sim.git
cd GRBLHAL-Sim
cmake -S . -B build-release/linux -DCMAKE_BUILD_TYPE=Release
cmake --build build-release/linux -j
./build-release/linux/grblHAL_flexihal_sim
```

Create a Linux release tarball:

```bash
./packaging/linux/package-linux.sh
```

The archive is written to `dist/grblhal-flexihal-sim-linux-x64.tar.gz`.

### Windows

Install:

- Git
- CMake
- Visual Studio Build Tools 2022 with "Desktop development with C++"

From an x64 Native Tools Command Prompt:

```bat
git clone --recurse-submodules https://github.com/bisonactual/GRBLHAL-Sim.git
cd GRBLHAL-Sim
cmake -S . -B C:\Users\Owl\git\grblhal-sim-windows-build -G "Visual Studio 17 2022" -A x64 -DBUILD_WINDOWS_LAUNCHER=ON
cmake --build C:\Users\Owl\git\grblhal-sim-windows-build --config Release
```

Create a Windows release zip:

```powershell
powershell -ExecutionPolicy Bypass -File .\packaging\windows\package-windows.ps1
```

The archive is written to `dist/grblhal-flexihal-sim-windows-x64.zip`.

### macOS

Install:

- Xcode Command Line Tools
- CMake
- Git

Build and run:

```bash
git clone --recurse-submodules https://github.com/bisonactual/GRBLHAL-Sim.git
cd GRBLHAL-Sim
cmake -S . -B build-release/macos -DCMAKE_BUILD_TYPE=Release
cmake --build build-release/macos -j
./build-release/macos/grblHAL_flexihal_sim
```

Create a macOS release tarball:

```bash
./packaging/macos/package-macos.sh
```

The archive is written to `dist/grblhal-flexihal-sim-macos-<arch>.tar.gz`.

## Running

CLI:

```bash
./grblHAL_flexihal_sim
```

Windows launcher:

```text
grblHAL_flexihal_launcher.exe
```

The launcher starts and stops the simulator process, stores its settings in `%LOCALAPPDATA%\grblHAL FlexiHAL Simulator\launcher.ini`, shows simulator output, and sends simulator input commands to the child process over stdin.
It also polls the simulator over stdin for a stdout `[SIMSTATE]` line and displays the current grblHAL realtime state without using the sender WebSocket.

Connect your sender to:

```text
ws://localhost:8080
```

## Command-Line Options

```text
-w <port>   WebSocket port for sender connection (default: 8080)
-t <speed>  Realtime speed multiplier (default: 1.0, 0 = max speed)
-e <file>   EEPROM/settings file (default: EEPROM.DAT)
-r <time>   Step report interval in seconds (0 = off)
-n          No comment prefix on serial output
-h          Show help
```

## Simulator Inputs

When running from a terminal, press keys in the simulator console. When using the Windows launcher, use either the buttons or the same keys while the launcher window has focus.

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
| `1` | Inject hard limit alarm (`ALARM:1`) |
| `2` | Inject soft limit alarm (`ALARM:2`) |
| `3` | Inject abort during cycle alarm (`ALARM:3`) |
| `4` | Inject probe initial-state alarm (`ALARM:4`) |
| `5` | Inject probe contact alarm (`ALARM:5`) |
| `6` | Inject homing reset alarm (`ALARM:6`) |
| `7` | Inject homing door alarm (`ALARM:7`) |
| `8` | Inject homing pull-off alarm (`ALARM:8`) |
| `9` | Inject homing approach alarm (`ALARM:9`) |
| `0` | Inject E-stop asserted alarm (`ALARM:10`) |
| `m` | Inject motor fault alarm (`ALARM:17`) |
| `i` | Force the simulator controller state back to Idle |
| `n` | Toggle no-response mode: keep WebSocket connected but drop controller replies |
| `k` | Kick the active WebSocket client |
| `?` | Request status report |
| Ctrl+C | Stop simulator in a terminal |

The Windows launcher also uses simulator-private stdin byte `0x12` to request a stdout state line:

```text
[SIMSTATE]<Idle|MPos:0.000,0.000,0.000,0.000,0.000|...>
```

That line is for local UI/status tooling. It is not sent over the sender WebSocket.
Forcing Idle prints a normal simulator log line and then emits a fresh `[SIMSTATE]` line so local UI can update immediately.

## Project Structure

```text
grblhal-sim/
├── CMakeLists.txt
├── packaging/
│   ├── linux/package-linux.sh
│   ├── macos/package-macos.sh
│   └── windows/package-windows.ps1
├── plugins/                 # Bundled simulator profile plugins
├── src/
│   ├── grbl/                # grblHAL core submodule
│   ├── driver.c/h           # FlexiHAL virtual HAL driver
│   ├── main.c               # Simulator entry point
│   ├── websocket.c/h        # Sender WebSocket server
│   ├── fs_sim.c/h           # Host-backed simulated SD card
│   ├── eeprom.c/h           # NVS persistence to file
│   ├── my_plugin.c          # Bundled plugin entry points
│   └── platform_*.c/h       # OS abstraction
└── ui/windows-launcher/      # Native Windows launcher
```

## Notes

- The sender WebSocket is only for sender/controller traffic.
- Simulator hardware events are injected through stdin.
- The upstream grblHAL core is kept as a clean submodule; simulator compatibility changes are applied from `packaging/grbl-core-flexihal-sim.patch` into the build tree.
- GitHub Actions builds Linux, Windows, and macOS packages on pushes, pull requests, and tags.
- Runtime UF2 firmware upload and browser plugin rebuilds were removed because this distributable targets one known FlexiHAL profile.
- Future firmware-profile loading should be added as a separate explicit simulator feature, not as source upload and rebuild from the browser.

## License

GPLv3, same as grblHAL.

Based on the grblHAL Simulator by Terje Io, Jens Geisler, and Adam Shelly.
