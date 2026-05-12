# Laptop Live Thermal Viewer
# This protgram displays:
# 1. A colour heatmap of the thermal camera image
# 2. A detection box around any detected person
# 3. CO2, humidity, and temperature readings in the title bar
# 4. A detection status message (CONFIRMED / POSSIBLE / NONE)
 
import requests
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from scipy.ndimage import zoom
import time
 
# Configuration:
 
# Change this to the IP address shown on your Raspberry Pi
PI_IP = "10.99.113.180"
 
# The full URL of the Flask data endpoint on the Pi
URL = f"http://{PI_IP}:5000/data"
 
# UPSCALE=10 makes the camera 320x240 pixels, which is much easier to see.
UPSCALE = 10
 
# Helper Function:

def nice(value, digits=1, unit=""):
    
    if value is None:
        return "---"   # Shown when the sensor hasn't sent data yet
    try:
        return f"{float(value):.{digits}f}{unit}"
    except Exception:
        return f"{value}{unit}"  # Fallback if value can't be converted to float
 
# Plot Setup:

# Turn on interactive mode so the plot updates in a loop
plt.ion()
 
# Create the figure and axes (9 inches wide, 6 inches tall)
fig, ax = plt.subplots(figsize=(9, 6))
 
# Start with a blank 24x32 thermal grid (all zeros)
thermal = np.zeros((24, 32))
 
# Scale up the image using scipy's zoom for display
thermal_resized = zoom(thermal, UPSCALE)
 
# Display the thermal image using the inferno colourmap
# Black = cold, yellow/white = hot
img = ax.imshow(thermal_resized, cmap="inferno", interpolation="nearest")
 
# Colour bar on the right showing the temperature scale
plt.colorbar(img, ax=ax, label="Temperature (°C)")
 
# Invisible rectangle that will be drawn around a detected person
# Invisible at first and show it only when a person is detected
bbox_patch = patches.Rectangle(
    (0, 0),          # Starting position (top-left corner)
    1,               # Initial width 
    1,               # Initial height
    linewidth=2,
    edgecolor="cyan",
    facecolor="none",  # Transparent fill outline only
    visible=False      # Hidden until a detection occurs
)
ax.add_patch(bbox_patch)
 
# Text label shown inside the plot with detection status and sensor info
# Positioned near the bottom-left of the image
status_text = ax.text(
    5,     # X position in pixel coordinates
    230,   # Y position in pixel coordinates 
    "",    # Start with empty text
    color="cyan",
    fontsize=11,
    bbox=dict(facecolor="black", alpha=0.65),  # Dark background for readability
)
 
# Label the axes
ax.set_xlabel("X")
ax.set_ylabel("Y")
 
print("Starting live thermal viewer...")

# Main Loop

# This loop runs continuously, fetching new data from the Pi and updating the plot

while True:
    try:
        # Step 1: Fetch data from the Pi
        response = requests.get(URL, timeout=2)  # 2-second timeout
        data = response.json()                   # Parse the JSON response
 
        # Step 2: Update the thermal heatmap
        thermal = np.array(data["thermal"])          # 24x32 temperature grid
        thermal_resized = zoom(thermal, UPSCALE)     # Scale up for display
 
        min_temp = data["min"]   # Coldest pixel in the frame
        max_temp = data["max"]   # Hottest pixel in the frame
        avg_temp = data["avg"]   # Average temperature
 
        img.set_data(thermal_resized)             # Update the image pixels
        img.set_clim(vmin=min_temp, vmax=max_temp) # Rescale the colour range
 
        # Step 3: Read detection flags from the Pi
        thermal_detected = data.get("thermal_detected", False)  # Warm blob found?
        co2_spike = data.get("co2_spike",        False)         # CO2 risen above baseline?
        humidity_spike = data.get("humidity_spike",   False)    # Humidity risen above baseline?
        strong_thermal = data.get("strong_thermal",   False)    # Very strong thermal signal?
 
        # Step 4: Calculate detection confidence level
        # This mirrors the three-tier logic used on the Pi.
        if strong_thermal and (co2_spike or humidity_spike):
            # Best case: strong thermal signal confirmed by air quality change
            confidence = "CONFIRMED"
 
        elif thermal_detected and (co2_spike or humidity_spike):
            # Thermal blob plus environmental signal
            confidence = "POSSIBLE"
 
        elif strong_thermal and not (co2_spike or humidity_spike):
            # Strong thermal alone
            confidence = "POSSIBLE"
 
        else:
            # Not enough evidence
            confidence = "NONE"
 
        # Step 5: Draw detection box and status text
        box = data.get("box")  # Bounding box dict with x, y, w, h (or None)
 
        if confidence in ("CONFIRMED", "POSSIBLE"):
            # Show and position the detection box if we have position data
            if box is not None:
                # Scale the box coordinates up to match the upscaled image
                x = box["x"] * UPSCALE
                y = box["y"] * UPSCALE
                w = box["w"] * UPSCALE
                h = box["h"] * UPSCALE
 
                bbox_patch.set_xy((x, y))    # Move box to correct position
                bbox_patch.set_width(w)      # Set box width
                bbox_patch.set_height(h)     # Set box height
                bbox_patch.set_visible(True) # Make box visible
            else:
                bbox_patch.set_visible(False)  # No position data, hide the box
 
            # Style the box and status text in cyan for a detection
            bbox_patch.set_edgecolor("cyan")
            status_text.set_color("cyan")
 
            # Choose heading based on confidence level
            if confidence == "CONFIRMED":
                heading = "PERSON DETECTED"
            else:
                heading = "POSSIBLE PERSON DETECTED"
 
            # Display detection details in the status text box
            status_text.set_text(
                f"{heading}\n"
                f"Thermal: {thermal_detected}\n"
                f"CO2 spike: {co2_spike}\n"
                f"Humidity spike: {humidity_spike}\n"
                f"Strong thermal: {strong_thermal}\n"
                f"Area: {data.get('area')} px\n"
                f"Peak: {nice(data.get('peak'), 1, '°C')}"
            )
 
        else:
            # While no detection hide the box and show a plain white status message
            bbox_patch.set_visible(False)
            status_text.set_color("white")
            status_text.set_text(
                "NO PERSON DETECTED\n"
                f"Thermal: {thermal_detected}\n"
                f"CO2 spike: {co2_spike}\n"
                f"Humidity spike: {humidity_spike}\n"
                f"Strong thermal: {strong_thermal}"
            )
 
        # Step 6: Update the plot title with live sensor readings
        ax.set_title(
            f"MLX90640 Thermal Viewer\n"
            f"Min: {nice(min_temp, 1, '°C')}   "
            f"Max: {nice(max_temp, 1, '°C')}   "
            f"Avg: {nice(avg_temp, 1, '°C')}\n"
            f"CO2: {nice(data.get('co2'), 1, ' ppm')}   "
            f"CO2 base: {nice(data.get('co2_baseline'), 1, ' ppm')}   "
            f"CO2 threshold: {nice(data.get('co2_threshold'), 1, ' ppm')}\n"
            f"Humidity: {nice(data.get('humidity'), 1, '%')}   "
            f"Humidity base: {nice(data.get('humidity_baseline'), 1, '%')}   "
            f"Humidity threshold: {nice(data.get('humidity_threshold'), 1, '%')}   "
            f"Air Temp: {nice(data.get('air_temp'), 1, '°C')}"
        )
 
        # Step 7: Refresh the plot
        fig.canvas.draw()          # Redraw the canvas
        fig.canvas.flush_events()  # Process any pending window events
        plt.pause(0.05)            # Short pause (~50ms) before next update
 
    except KeyboardInterrupt:
        # Pressing Ctrl+C exits the loop cleanly
        print("Stopping viewer...")
        break
 
    except Exception as e:
        # If the Pi is unreachable or returns bad data, wait and retry
        print("Viewer error:", e)
        time.sleep(1)
        
# Turn off interactive mode and show the final frozen frame
plt.ioff()
plt.show()