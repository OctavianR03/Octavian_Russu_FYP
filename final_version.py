# Rover Control System
# This program controls a rover that can:
# 1. Drive using keyboard input (W/A/S/D keys)
# 2. Control a camera servo (tilt up/down/centre)
# 3. Detect people using a thermal camera (MLX90640)
# 4. Confirm detection using CO2 and humidity sensors (SCD30)
# 5. Stream sensor data over a local web server (Flask)
# 6. Display a live terminal dashboard (curses UI)

import time
import threading
import curses
import traceback

import serial
import numpy as np
import board
import busio
import adafruit_mlx90640
import adafruit_scd30

from scipy.ndimage import label, find_objects
from flask import Flask, jsonify

# Configuration constants:
# These values control how the system behaves

# Serial port settings for communicating with the Arduino
PORT = "/dev/ttyACM0"   # The port the Arduino is connected to
BAUD = 9600             # Communication speed matching the Arduino sketch

# Flask web server port
FLASK_PORT = 5000       # The local port for the data API

# Timing intervals (in seconds)
SEND_INTERVAL = 0.05    # How often to send drive commands to the Arduino
KEY_TIMEOUT = 0.20      # How long after a keypress before the rover stops
THERMAL_INTERVAL = 0.20 # How often to read the thermal camera
SCD30_INTERVAL = 2.0    # How often to read the CO2/humidity sensor

# Thermal Detection Thresholds
MIN_HUMAN_PIXELS = 5        # Minimum blob size pixels to count as a detection
MIN_TEMP_FLOOR = 29.0       # Minimum temperature (°C) a pixel must be to be hot
TEMP_ABOVE_BACKGROUND = 2.0 # How many °C above ambient a pixel must be
MIN_CONTRAST = 3.0          # Minimum difference between peak and ambient temp

# CO2 and Humidity Thresholds
CO2_BASELINE_SAMPLES = 10       # Number of readings to average for the CO2 baseline
CO2_SPIKE_PPM = 35.0            # A rise above baseline by this much triggers a CO2 spike
HUMIDITY_SPIKE_PERCENT = 0.6    # A rise above baseline by this much triggers humidity spike

# Strong Thermal override
# If the thermal signal alone is very strong, the system allows a POSSIBLE detection
# even before CO2/humidity has risen
STRONG_THERMAL_EXTRA_CONTRAST = 1.5  # Extra contrast needed to be counted as strong thermal

# Global State:
# Shared data between threads.

app  = Flask(__name__)     # The Flask web server instance
lock = threading.Lock()    # Thread lock, only one thread writes at a time

running = True   # Set to False to shut everything down gracefully
ser = None       # The serial connection object (set up in setup_serial)

# Drive command tracking
manual_cmd = "S"    # The command the user is currently pressing (S = Stop)
sent_cmd = "S"      # The last command actually sent to the Arduino
last_key_time = 0.0 # Timestamp of the last keypress

# Servo tracking
servo_cmd = None        # A one-shot servo command waiting to be sent
last_servo_cmd = "-"    # The last servo command that was sent

# System status flags
serial_ok = False   # True if serial connection is working
sensor_ok = False   # True if sensors are working
serial_error = ""   # Holds error message if serial fails
sensor_error = ""   # Holds error message if sensor fails

# CO2 and humidity baseline learning
co2_samples = []        # Stores initial CO2 readings to calculate baseline
humidity_samples = []   # Stores initial humidity readings to calculate baseline
co2_baseline = None     # Average CO2 level at startup (used as "normal")
humidity_baseline = None # Average humidity level at startup (used as "normal")

# `latest` is the central shared data dictionary.
# The Flask server serves this to the browser.
# The UI reads from it to display the dashboard.
latest = {
    # Raw thermal camera data (24x32 grid of temperatures)
    "thermal": [[0 for _ in range(32)] for _ in range(24)],
    "min": 0,       # Minimum temperature in the frame
    "max": 40,      # Maximum temperature in the frame
    "avg": 0,       # Average temperature in the frame
    "threshold": 29, # Calculated detection threshold temperature

    # Overall detection result
    "detected": False,              # True if a person is detected
    "detection_confidence": "NONE", # "CONFIRMED", "POSSIBLE", or "NONE"

    # Individual detection signals
    "thermal_detected": False,  # True if a warm blob was found in the thermal image
    "co2_spike": False,         # True if CO2 has risen above baseline
    "humidity_spike": False,    # True if humidity has risen above baseline
    "strong_thermal": False,    # True if the thermal signal alone is very strong

    # Bounding box and position of the detected person
    "box": None,       # Dictionary with x, y, w, h of the detection box
    "area": 0,         # Size of the detected blob in pixels
    "peak": None,      # Hottest pixel temperature in the detected blob
    "center_x": None,  # Horizontal centre of the detected blob
    "center_y": None,  # Vertical centre of the detected blob

    # Environmental sensor readings
    "co2": None,               # Current CO2 reading (ppm)
    "humidity": None,          # Current relative humidity (%)
    "air_temp": None,          # Current air temperature from SCD30 (°C)
    "co2_baseline": None,      # Learned normal CO2 level
    "humidity_baseline": None, # Learned normal humidity level
    "co2_threshold": None,     # CO2 level that triggers a spike alert
    "humidity_threshold": None,# Humidity level that triggers a spike alert

    # Drive and servo control state
    "manual_cmd": "S",
    "sent_cmd": "S",
    "last_servo_cmd": "-",

    # Connection status
    "serial_ok": False,
    "sensor_ok": False,
    "serial_error": "",
    "sensor_error": "",
}

# Person Detection Algorithm:
def detect_person(thermal, co2=None, humidity=None):

    global co2_baseline, humidity_baseline

    # Step 1: Calculate the ambient (background) temperature
    # using the 10th percentile rather than the minimum, so that
    # cold corners or sensor noise don't skew our reference point.
    ambient = float(np.percentile(thermal, 10))

    # The detection threshold is at least MIN_TEMP_FLOOR, or ambient + offset,
    # whichever is higher. This adapts to warm rooms automatically.
    threshold = max(MIN_TEMP_FLOOR, ambient + TEMP_ABOVE_BACKGROUND)

    # Step 2: Create a hot mask
    # Mark every pixel that is above the detection threshold and below 37°C
    hot_mask = (thermal >= threshold) & (thermal <= 37.0)

    # Step 3: Find connected blobs of hot pixels
    # `label` assigns a unique integer ID to each connected group of True pixels.
    labeled_array, _ = label(hot_mask)

    # Track the largest valid blob found
    best_area  = 0
    best_box   = None
    best_peak  = None

    # Loop through each blob and check if it qualifies as a person
    for i, obj_slice in enumerate(find_objects(labeled_array), start=1):
        if obj_slice is None:
            continue

        # create a mask for just this blob
        region = labeled_array[obj_slice] == i
        area   = int(np.sum(region))  # number of pixels in this blob

        # Reject blobs that are too small to be a person
        if area < MIN_HUMAN_PIXELS:
            continue

        # Get the temperatures of just the pixels in this blob
        temps = thermal[obj_slice][region]
        peak  = float(np.max(temps))  # hottest pixel in the blob

        # Reject blobs where the peak temperature is too close to ambient (not enough contrast)
        if peak - ambient < MIN_CONTRAST:
            continue

        # Keep track of the largest valid blob
        if area > best_area:
            best_area = area
            best_box  = obj_slice
            best_peak = peak

    # Whether any qualifying thermal blob was found
    thermal_detected = best_box is not None

    # Step 4: Check CO2 and humidity against learned baselines
    co2_threshold      = None
    humidity_threshold = None
    co2_spike          = False
    humidity_spike     = False

    # Only check for a CO2 spike once we have a baseline to compare against
    if co2_baseline is not None:
        co2_threshold = co2_baseline + CO2_SPIKE_PPM
        if co2 is not None:
            co2_spike = co2 >= co2_threshold  # true if CO2 has risen above threshold

    # Only check for a humidity spike once we have a baseline to compare against
    if humidity_baseline is not None:
        humidity_threshold = humidity_baseline + HUMIDITY_SPIKE_PERCENT
        if humidity is not None:
            humidity_spike = humidity >= humidity_threshold  # True if humidity risen above threshold

    # Step 5: Check for strong thermal signal 
    # A strong thermal detection means the blob is large, hot, and clearly above background
    strong_thermal = (
        best_peak is not None
        and best_area >= 10             # must be a reasonably large blob
        and 30.0 <= best_peak <= 35.5   # temperature in expected human skin range
        and (best_peak - ambient) >= (MIN_CONTRAST + STRONG_THERMAL_EXTRA_CONTRAST)
    )

    # Step 6: Fused decision (three-tier confidence)
    if strong_thermal and (co2_spike or humidity_spike):
        # Best case: strong thermal signal confirmed by air quality change
        detection_confidence = "CONFIRMED"

    elif thermal_detected and (co2_spike or humidity_spike):
        # Thermal blob seen, and air quality supports it
        detection_confidence = "POSSIBLE"

    elif strong_thermal:
        # Only thermal evidence
        detection_confidence = "POSSIBLE"

    else:
        # Not enough evidence
        detection_confidence = "NONE"

    # True if any level of detection was found
    fused_detected = detection_confidence != "NONE"

    # Step 7: Return results
    # If no blob was found, return with empty position data
    if best_box is None:
        return {
            "detected": fused_detected,
            "detection_confidence": detection_confidence,
            "thermal_detected": thermal_detected,
            "co2_spike": co2_spike,
            "humidity_spike": humidity_spike,
            "strong_thermal": strong_thermal,
            "box": None,
            "area": 0,
            "peak": None,
            "center_x": None,
            "center_y": None,
            "threshold": threshold,
            "co2_threshold": co2_threshold,
            "humidity_threshold": humidity_threshold,
        }

    # Calculate bounding box and centre point of the best blob
    y_slice, x_slice = best_box

    box = {
        "x": int(x_slice.start),                # Left edge of the box
        "y": int(y_slice.start),                # Top edge of the box
        "w": int(x_slice.stop - x_slice.start), # Width of the box
        "h": int(y_slice.stop - y_slice.start), # Height of the box
    }

    center_x = (x_slice.start + x_slice.stop) / 2  # Horizontal midpoint
    center_y = (y_slice.start + y_slice.stop) / 2  # Vertical midpoint

    return {
        "detected": fused_detected,
        "detection_confidence": detection_confidence,
        "thermal_detected": thermal_detected,
        "co2_spike": co2_spike,
        "humidity_spike": humidity_spike,
        "strong_thermal": strong_thermal,
        "box": box,
        "area": best_area,
        "peak": best_peak,
        "center_x": center_x,
        "center_y": center_y,
        "threshold": threshold,
        "co2_threshold": co2_threshold,
        "humidity_threshold": humidity_threshold,
    }

# Serial Setup:

def setup_serial():

    # Sets `serial_ok` to True on success, or records the error on failure.
    # Called once at startup before the threads begin.
    global ser, serial_ok, serial_error

    try:
        # Open the serial port (timeout=0 means non-blocking reads)
        ser = serial.Serial(PORT, BAUD, timeout=0)

        # Wait 2 seconds for the Arduino to reset after the connection opens
        time.sleep(2)

        # Clear any leftover data in the buffers
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        serial_ok    = True
        serial_error = ""

    except Exception as e:
        # If the port can't be opened, record the error and continue
        ser          = None
        serial_ok    = False
        serial_error = f"{type(e).__name__}: {e}"

# Sensor thread:

def sensor_loop():
    
    # Background thread that continuously reads the thermal camera (MLX90640)
    # and the CO2/humidity sensor (SCD30), runs person detection, and updates
    # the shared `latest` dictionary.

    global latest, sensor_ok, sensor_error, running
    global co2_baseline, humidity_baseline, co2_samples, humidity_samples

    try:
        # Give sensors time to power up before trying to read them
        print("Waiting for sensors to power up...")
        time.sleep(8)

        # Set up the I2C bus that both sensors share
        i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)

        # Initialise the thermal camera
        print("Initializing MLX90640...")
        mlx = adafruit_mlx90640.MLX90640(i2c)
        mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_4_HZ  # 4 frames/sec
        print("MLX90640 detected")

        # Initialise the CO2 and humidity sensor
        print("Initializing SCD30...")
        scd = adafruit_scd30.SCD30(i2c)
        scd.measurement_interval = 2  # Take a reading every 2 seconds
        print("SCD30 detected")

        # Buffer for the raw thermal frame (24 rows × 32 cols = 768 values)
        frame = [0] * 768

        # Track when last read each sensor
        last_thermal = 0
        last_scd = 0

        # Main sensor reading loop
        while running:
            now = time.time()

            # Read Thermal Camera
            if now - last_thermal >= THERMAL_INTERVAL:
                try:
                    # Read a full frame from the camera into the buffer
                    mlx.getFrame(frame)

                    # Reshape the flat list into a 24x32 numpy array
                    thermal = np.array(frame, dtype=float).reshape((24, 32))

                    # Get the latest CO2/humidity readings to pass into detection
                    with lock:
                        current_co2 = latest["co2"]
                        current_humidity = latest["humidity"]

                    # Run the person detection algorithm
                    detection = detect_person(
                        thermal,
                        co2 = current_co2,
                        humidity = current_humidity
                    )

                    # Write results into the shared dictionary
                    with lock:
                        # Raw thermal stats
                        latest["thermal"] = thermal.tolist()
                        latest["min"] = float(np.min(thermal))
                        latest["max"] = float(np.max(thermal))
                        latest["avg"] = float(np.mean(thermal))

                        # Detection results
                        latest["threshold"] = float(detection["threshold"])
                        latest["detected"] = detection["detected"]
                        latest["detection_confidence"] = detection["detection_confidence"]
                        latest["thermal_detected"] = detection["thermal_detected"]
                        latest["co2_spike"] = detection["co2_spike"]
                        latest["humidity_spike"] = detection["humidity_spike"]
                        latest["strong_thermal"] = detection["strong_thermal"]

                        # Position of detected person
                        latest["box"] = detection["box"]
                        latest["area"] = detection["area"]
                        latest["peak"] = detection["peak"]
                        latest["center_x"] = detection["center_x"]
                        latest["center_y"] = detection["center_y"]

                        # Detection thresholds for display
                        latest["co2_threshold"] = detection["co2_threshold"]
                        latest["humidity_threshold"] = detection["humidity_threshold"]

                        latest["sensor_ok"] = True
                        latest["sensor_error"] = ""

                    sensor_ok = True
                    sensor_error = ""

                except ValueError:
                    # Skip corrupt frames sent by the MLX90640
                    pass

                except Exception as e:
                    sensor_ok = False
                    sensor_error = f"MLX90640 error: {type(e).__name__}: {e}"

                    with lock:
                        latest["sensor_ok"] = False
                        latest["sensor_error"] = sensor_error

                last_thermal = now

            # Read CO2 and Humidity Sensor (SCD30)
            if now - last_scd >= SCD30_INTERVAL:
                try:
                    if scd.data_available:
                        # Read the three values from the SCD30
                        co2_value = round(float(scd.CO2), 1)
                        humidity_value = round(float(scd.relative_humidity), 1)
                        air_temp_value = round(float(scd.temperature), 1)

                        # Build CO2 baseline from the first N readings
                        if co2_baseline is None:
                            co2_samples.append(co2_value)

                            if len(co2_samples) >= CO2_BASELINE_SAMPLES:
                                # Once enough samples collected, average them
                                co2_baseline = sum(co2_samples) / len(co2_samples)

                        # Build humidity baseline from the first N readings
                        if humidity_baseline is None:
                            humidity_samples.append(humidity_value)

                            if len(humidity_samples) >= CO2_BASELINE_SAMPLES:
                                humidity_baseline = sum(humidity_samples) / len(humidity_samples)

                        # Calculate current spike thresholds from baselines
                        co2_threshold = None
                        humidity_threshold = None

                        if co2_baseline is not None:
                            co2_threshold = co2_baseline + CO2_SPIKE_PPM

                        if humidity_baseline is not None:
                            humidity_threshold = humidity_baseline + HUMIDITY_SPIKE_PERCENT

                        # Update the shared dictionary with environmental data
                        with lock:
                            latest["co2"] = co2_value
                            latest["humidity"] = humidity_value
                            latest["air_temp"] = air_temp_value

                            latest["co2_baseline"] = co2_baseline
                            latest["humidity_baseline"] = humidity_baseline
                            latest["co2_threshold"] = co2_threshold
                            latest["humidity_threshold"] = humidity_threshold

                            latest["sensor_ok"] = True
                            latest["sensor_error"] = ""

                    sensor_ok = True
                    sensor_error = ""

                except Exception as e:
                    sensor_ok = False
                    sensor_error = f"SCD30 error: {type(e).__name__}: {e}"

                    with lock:
                        latest["sensor_ok"] = False
                        latest["sensor_error"] = sensor_error

                last_scd = now

            # Short sleep to prevent this thread from hogging the CPU
            time.sleep(0.02)

    except Exception as e:
        # If sensor setup fails entirely, record the error
        sensor_ok = False
        sensor_error = f"Sensor setup error: {type(e).__name__}: {e}"

        with lock:
            latest["sensor_ok"] = False
            latest["sensor_error"] = sensor_error


# Serial Control Thread:
def serial_loop():
    
    # Background thread that sends drive commands to the Arduino over serial.
    global sent_cmd, serial_ok, serial_error, running
    global servo_cmd, last_servo_cmd

    while running:
        # Read the current state under the lock to avoid race conditions
        with lock:
            cmd = manual_cmd            # The user's current key input
            last_key = last_key_time    # When was the last keypress?
            pending_servo = servo_cmd   # One-shot servo command (if any)
            servo_cmd = None            # Clear it so it only sends once

        # If no key has been pressed recently, default to Stop
        if time.time() - last_key > KEY_TIMEOUT:
            cmd_to_send = "S"
        else:
            cmd_to_send = cmd

        sent_cmd = cmd_to_send

        # Send the commands over serial if the port is open
        if ser is not None:
            try:
                # Send servo command first (if one is pending)
                if pending_servo is not None:
                    ser.write(pending_servo.encode("ascii"))
                    last_servo_cmd = pending_servo
                    time.sleep(0.01)  # Short gap between commands

                # Send the drive command
                ser.write(cmd_to_send.encode("ascii"))

                serial_ok = True
                serial_error = ""

            except Exception as e:
                serial_ok = False
                serial_error = f"{type(e).__name__}: {e}"

        # Write updated control state back to the shared dictionary
        with lock:
            latest["manual_cmd"] = manual_cmd
            latest["sent_cmd"] = sent_cmd
            latest["last_servo_cmd"] = last_servo_cmd
            latest["serial_ok"] = serial_ok
            latest["serial_error"] = serial_error

        # Wait before sending the next command
        time.sleep(SEND_INTERVAL)

# Flask Web Server:
@app.route("/data")
def data():

    # HTTP endpoint that returns all sensor and detection data as JSON.
    with lock:
        return jsonify(latest)


def flask_loop():
    
    # Starts the Flask web server.
    # Runs in its own background thread so it doesn't block the UI.
    
    app.run(host="0.0.0.0", port=FLASK_PORT, debug=False, use_reloader=False)

# Helper Functions for the Terminal UI:
def fmt(value, spec):

    # Safely formats a value for display.
    # Returns "---" if the value is None (sensor not yet ready)

    if value is None:
        return "---"
    return format(value, spec)


def reset_environment_baselines():

    # Resets the CO2 and humidity baselines back to None.
    # This causes the system to re-learn "normal" levels from the next readings.
    
    global co2_baseline, humidity_baseline, co2_samples, humidity_samples

    co2_baseline      = None
    humidity_baseline = None
    co2_samples       = []
    humidity_samples  = []

    # Also clear the displayed values in the shared dictionary
    latest["co2_baseline"]      = None
    latest["humidity_baseline"] = None
    latest["co2_threshold"]     = None
    latest["humidity_threshold"] = None
    latest["co2_spike"]         = False
    latest["humidity_spike"]    = False


# Terminal User Interface (curses):
def ui_main(stdscr):

    # The main terminal UI, powered by the curses library.
    # Displays a live dashboard

    global running, manual_cmd, last_key_time, servo_cmd

    # Configure the terminal
    curses.curs_set(0)    # Hide the blinking cursor
    stdscr.nodelay(True)  # Don't block waiting for a keypress
    stdscr.keypad(True)   # Enable special key support

    last_key_time = time.time()

    while running:
        key = stdscr.getch()  # Read a key (-1 if no key was pressed)
        now = time.time()

        # Handle Key Input
        if key != -1:
            with lock:
                # Drive controls (WASD)
                if key == ord("w"):
                    manual_cmd = "F"    # Forward
                    last_key_time = now

                elif key == ord("s"):
                    manual_cmd = "B"    # Back
                    last_key_time = now

                elif key == ord("a"):
                    manual_cmd = "L"    # Left
                    last_key_time = now

                elif key == ord("d"):
                    manual_cmd = "R"    # Right
                    last_key_time = now

                elif key == ord("x"):
                    manual_cmd = "S"    # Stop
                    last_key_time = now

                # Reset baselines (R or r)
                elif key == ord("r") or key == ord("R"):
                    reset_environment_baselines()

                # Servo controls (camera tilt)
                elif key == ord("i") or key == ord("I"):
                    servo_cmd = "I"   # tilt up

                elif key == ord("k") or key == ord("K"):
                    servo_cmd = "K"   # tilt down

                elif key == ord("m") or key == ord("M"):
                    servo_cmd = "M"   # centre

                # Quit (Q or q)
                elif key == ord("q") or key == ord("Q"):
                    manual_cmd = "S"    # stop the rover before quitting
                    running = False

        # Take a snapshot of the shared data for display
        with lock:
            d = latest.copy()

        # Draw the UI
        stdscr.erase()   # Clear the screen before redrawing
        W = 62           # Width of separator lines

        # Title bar
        stdscr.addstr(0, 0, " Rover Control System")
        stdscr.addstr(1, 0, "-" * 40)

        # Drive control diagram and current command
        row = 3
        stdscr.addstr(row,     0, "  Drive Controls:")
        stdscr.addstr(row + 1, 0, "-" * W)
        stdscr.addstr(row + 2, 4, "        [W] Forward")
        stdscr.addstr(row + 3, 4, "[A] Left           [D] Right")
        stdscr.addstr(row + 4, 4, "         [S] Back")

        # Map single-letter codes to readable names for display
        wheel_names = {"F": "Forward", "B": "Back", "L": "Left", "R": "Right"}
        w_input = wheel_names.get(d["manual_cmd"], d["manual_cmd"])
        w_sent  = wheel_names.get(d["sent_cmd"],   d["sent_cmd"])
        stdscr.addstr(row + 6, 4, f"Input: {w_input:<10} Sent: {w_sent}")

        # Servo (camera) control section
        row = 11
        stdscr.addstr(row,     0, "  Camera Position:")
        stdscr.addstr(row + 1, 0, "-" * W)
        stdscr.addstr(row + 2, 4, "[I] Tilt Up    [K] Tilt Down    [M] Centre")
        stdscr.addstr(row + 3, 4, f"Last command: {d['last_servo_cmd']}")

        # System status (are serial and sensors connected?)
        row = 17
        stdscr.addstr(row,     0, "  System Status:")
        stdscr.addstr(row + 1, 0, "-" * W)

        serial_status = "OK" if d["serial_ok"] else "ERROR"
        sensor_status = "OK" if d["sensor_ok"] else "ERROR"

        stdscr.addstr(row + 2, 4, f"Serial: {serial_status:<6} Sensors: {sensor_status}")

        # Environmental sensor readings (CO2, humidity, air temperature)
        row = 21
        stdscr.addstr(row,     0, "  Environment (SCD30):")
        stdscr.addstr(row + 1, 0, "-" * W)
        stdscr.addstr(row + 2, 4,
            f"CO2:      {fmt(d['co2'], '.1f'):>7} ppm   "
            f"(base: {fmt(d['co2_baseline'], '.1f')}, threshold: {fmt(d['co2_threshold'], '.1f')})")
        stdscr.addstr(row + 3, 4,
            f"Humidity: {fmt(d['humidity'], '.1f'):>7} %     "
            f"(base: {fmt(d['humidity_baseline'], '.1f')}, threshold: {fmt(d['humidity_threshold'], '.1f')})")
        stdscr.addstr(row + 4, 4, f"Air Temp: {fmt(d['air_temp'], '.1f'):>7} C")
        stdscr.addstr(row + 5, 4, "[R] Reset baselines.")

        # Thermal camera stats
        row = 28
        stdscr.addstr(row,     0, "  Thermal Camera (MLX90640):")
        stdscr.addstr(row + 1, 0, "-" * W)
        stdscr.addstr(row + 2, 4,
            f"Min: {fmt(d['min'], '.1f'):>6} C    "
            f"Max: {fmt(d['max'], '.1f'):>6} C    "
            f"Avg: {fmt(d['avg'], '.1f'):>6} C")
        stdscr.addstr(row + 3, 4, f"Detection threshold: {fmt(d['threshold'], '.1f')} C")

        # Person detection summary
        row = 33
        stdscr.addstr(row,     0, "  Person Detection:")
        stdscr.addstr(row + 1, 0, "-" * W)

        # Choose a display string based on confidence level
        if d["detection_confidence"] == "CONFIRMED":
            detected_str = "PERSON DETECTED"
        elif d["detection_confidence"] == "POSSIBLE":
            detected_str = "POSSIBLE PERSON DETECTED"
        else:
            detected_str = "NO PERSON DETECTED"

        stdscr.addstr(row + 2, 4, f"Status: {detected_str}")
        stdscr.addstr(row + 3, 4,
            f"Thermal blob: {d['thermal_detected']}             Strong thermal: {d['strong_thermal']}")
        stdscr.addstr(row + 4, 4,
            f"CO2 spike: {d['co2_spike']}               Humidity spike: {d['humidity_spike']}")
        stdscr.addstr(row + 5, 4,
            f"Area: {d['area']:>4} px       "
            f"Peak: {fmt(d['peak'], '.1f'):>6} C      "
            f"Target: ({fmt(d['center_x'], '.1f')}, {fmt(d['center_y'], '.1f')})")

        # Error messages (only shown if there are errors)
        row = 40
        if d["serial_error"] or d["sensor_error"]:
            stdscr.addstr(row,     0, "  Errors")
            stdscr.addstr(row + 1, 0, "-" * W)
            row += 2

            if d["serial_error"]:
                # Truncate long error messages to fit the screen width
                stdscr.addstr(row, 4, f"Serial: {d['serial_error']}"[:W - 4])
                row += 1

            if d["sensor_error"]:
                stdscr.addstr(row, 4, f"Sensor: {d['sensor_error']}"[:W - 4])
                row += 1

            stdscr.addstr(row, 0, "-" * W)
            row += 1

        # Footer with quit instruction
        stdscr.addstr(row,     0, "-" * W)
        stdscr.addstr(row + 1, 4, "Q - Quit")
        stdscr.addstr(row + 2, 0, "-" * W)

        # Push everything to the screen
        stdscr.refresh()

        # Short sleep to avoid redrawing too fast
        time.sleep(0.03)

# Main Entry Point:
def main():
  
    # Program entry point.
    # Sets up the serial connection, starts all background threads, then launches the terminal UI.

    global running

    # Attempt to connect to the Arduino over serial
    setup_serial()

    # Create background threads
    threads = [
        threading.Thread(target=sensor_loop, daemon=True),  # Reads sensors
        threading.Thread(target=serial_loop, daemon=True),  # Sends commands
        threading.Thread(target=flask_loop,  daemon=True),  # Hosts web API
    ]

    # Start all threads (daemon=True means they stop automatically when main exits)
    for t in threads:
        t.start()

    try:
        # Launch the curses terminal UI (blocks until UI exits)
        curses.wrapper(ui_main)

    except KeyboardInterrupt:
        # User pressed Ctrl+C — exit gracefully
        pass

    except Exception:
        # Print any unexpected errors to the terminal
        traceback.print_exc()

    finally:
        # Signal all threads to stop
        running = False

        # Give threads a moment to finish their current iteration
        time.sleep(0.2)

        # Send final stop commands to the Arduino and close the port
        if ser is not None:
            try:
                ser.write(b"S")    # Stop the wheels
                time.sleep(0.05)
                ser.write(b"M")    # Centre the servo
                time.sleep(0.05)
                ser.close()        # Close the serial port cleanly
            except Exception:
                pass

        print("System stopped safely.")

# Run the program:
if __name__ == "__main__":
    main()