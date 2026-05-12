# Low-Cost SAR Rover with Thermal and Air Quality-Based Human Detection

**Author:** Octavian Russu  
**Degree:** B.Sc. Robotics and Intelligent Devices  
**Supervisor:** JahanZeb Gul  

## Project Overview
This project implements a low-cost search and rescue (SAR) rover capable of detecting human presence in confined or hazardous indoor environments. It combines a thermal imaging camera (MLX90640) with CO₂ and humidity sensing (SCD30) in a rule-based sensor fusion algorithm, running on a Raspberry Pi 5. The rover is manually driven via keyboard commands, streams live sensor data over Wi-Fi, and displays a real-time terminal dashboard.

## File Descriptions

### `final_version.py` — Raspberry Pi Main Program

This is the core program that runs on the Raspberry Pi 5. It manages all rover functions concurrently using four background threads:

- **Sensor loop** - Reads thermal frames from the MLX90640 every 0.2 s and CO₂/humidity from the SCD30 every 2 s, then runs the person detection algorithm
- **Serial loop** - Sends drive and servo commands to the Arduino every 50 ms via UART
- **Flask loop** - Hosts a local web server on port 5000, serving all sensor and detection data as JSON at `/data`
- **UI loop** - Renders a live terminal dashboard using Python's `curses` library

**Detection algorithm (inside `detect_person()`):**
1. Computes an adaptive temperature threshold per frame based on ambient temperature
2. Masks pixels in the human thermal range (threshold → 37 °C)
3. Groups hot pixels into connected blobs using SciPy's `label()`
4. Filters blobs by minimum size (5 px) and minimum contrast (3 °C above ambient)
5. Learns CO₂ and humidity baselines from the first 10 readings after startup
6. Fuses all signals into one of three confidence levels:
   - `CONFIRMED` — Strong thermal signal + CO₂ or humidity spike
   - `POSSIBLE` — Thermal blob + environmental spike, or strong thermal alone
   - `NONE` — Insufficient evidence

**Keyboard controls (in the terminal dashboard):**

| Key | Action |
|---|---|
| W / A / S / D | Drive Forward / Left / Back / Right |
| X | Force stop |
| I / K / M | Camera tilt Up / Down / Centre |
| R | Reset CO₂ and humidity baselines |
| Q | Quit |

### `final_version.ino` — Arduino Sketch
This sketch runs on the Elegoo Uno R3 and handles all low-level motor and servo control. It receives single ASCII character commands from the Raspberry Pi over serial (9600 baud) and responds accordingly.

### `laptop_live.py` — Laptop Thermal Viewer
This script runs on a remote laptop connected to the same Wi-Fi network as the rover. It polls the Raspberry Pi's Flask endpoint every 50 ms and renders a live thermal heatmap with detection overlays using Matplotlib.

**Features:**
- Upscales the 24×32 thermal array by 10× for a clear 240×320 display
- Dynamically adjusts the colour scale to the current frame's temperature range
- Draws a **cyan bounding box** around detected persons
- Shows detection status (`CONFIRMED` / `POSSIBLE` / `NO PERSON DETECTED`) and signal flags
- Displays CO₂, humidity, air temperature, and thermal stats in the title bar

## Known Limitations
- Detection range is limited to ~1m due to the MLX90640's 24×32 resolution
- CO₂ and humidity changes delay 5–15 seconds behind physical presence due to gas diffusion
- Non-human heat sources (warm drinks) in the human temperature range (30–35.5 °C) may trigger a `POSSIBLE` alert if environmental signals do not suppress it
- The system is designed for stationary or slow-moving subjects. A person walking quickly through the field of view may not trigger a detection event
