#!/usr/bin/env python3
"""
 ===========================================================================
   COLOR DETECTION TEST - Standalone Windows Tool
 ===========================================================================

 Tests color detection and object dimension measurement WITHOUT ROS.
 Useful for calibrating HSV ranges and testing box size detection.

 Usage:
   python color_detection_test.py

 Requirements:
   pip install opencv-python numpy

 Controls:
   - SPACE: Capture frame and print debug info
   - Q: Quit

 ===========================================================================
"""

import cv2
import numpy as np

# ─────────────────────────────────────────────────────────────────────────────
# CONFIGURATION
# ─────────────────────────────────────────────────────────────────────────────

# HSV Color ranges for detection
# Red has TWO ranges because Hue wraps around at 0/180
COLOR_RANGES = {
    'red': {
        'lower1': np.array([0, 100, 100]),      # Lower red (Hue 0-10)
        'upper1': np.array([10, 255, 255]),
        'lower2': np.array([170, 100, 100]),     # Upper red (Hue 170-180)
        'upper2': np.array([180, 255, 255])
    },
    'green': {
        'lower': np.array([40, 50, 50]),
        'upper': np.array([80, 255, 255])
    },
    'blue': {
        'lower': np.array([100, 80, 50]),       # S:80 to reject skin
        'upper': np.array([130, 255, 255])
    }
}

# Camera calibration (from calibration)
FOCAL_LENGTH = 828.4  # pixels from camera matrix [0,0]
BOX_WIDTH_MM = 100    # actual box width in mm

# Detection thresholds
MIN_AREA = 500        # minimum contour area
ASPECT_HORIZONTAL = 1.5  # width/height > this = horizontal
ASPECT_VERTICAL = 0.67    # width/height < this = vertical


# ─────────────────────────────────────────────────────────────────────────────
# DETECTION FUNCTIONS
# ─────────────────────────────────────────────────────────────────────────────

def detect_color_and_dimensions(frame, focal_length=FOCAL_LENGTH, ref_width_mm=BOX_WIDTH_MM):
    """
    Detect color box and calculate real-world dimensions.
    Detects ALL colors simultaneously (red, green, blue).

    Returns dict with:
    - color: 'red', 'green', 'blue', 'none', or 'multiple'
    - detected: list of all detected colors
    - boxes: dict with info for each detected color
    - pixel_width, pixel_height: bounding box in pixels (largest detection)
    - width_mm, height_mm: real-world dimensions in mm
    - distance_m: estimated distance in meters
    - orientation: 'horizontal', 'vertical', 'unknown'
    """
    h, w = frame.shape[:2]

    # Full frame HSV (no ROI restriction for better detection)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    all_detections = {}

    # Process each color
    for color_name in ['red', 'green', 'blue']:
        if color_name == 'red':
            # Red has two Hue ranges
            mask1 = cv2.inRange(hsv, COLOR_RANGES['red']['lower1'], COLOR_RANGES['red']['upper1'])
            mask2 = cv2.inRange(hsv, COLOR_RANGES['red']['lower2'], COLOR_RANGES['red']['upper2'])
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            mask = cv2.inRange(hsv, COLOR_RANGES[color_name]['lower'], COLOR_RANGES[color_name]['upper'])

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)

            if area > MIN_AREA:
                x, y, pw, ph = cv2.boundingRect(largest)

                # Calculate dimensions
                if pw > 10:
                    distance_m = (ref_width_mm * focal_length) / (pw * 1000)
                    width_mm = (pw * distance_m) / focal_length * 1000
                    height_mm = (ph * distance_m) / focal_length * 1000
                else:
                    distance_m = width_mm = height_mm = 0.0

                # Orientation
                aspect = pw / ph if ph > 0 else 1.0
                if aspect > ASPECT_HORIZONTAL:
                    orientation = 'horizontal'
                elif aspect < ASPECT_VERTICAL:
                    orientation = 'vertical'
                else:
                    orientation = 'unknown'

                all_detections[color_name] = {
                    'pixel_width': pw,
                    'pixel_height': ph,
                    'width_mm': round(width_mm, 1),
                    'height_mm': round(height_mm, 1),
                    'distance_m': round(distance_m, 3),
                    'orientation': orientation,
                    'area': area,
                    'center_x': x + pw // 2,
                    'center_y': y + ph // 2
                }

    # Determine primary color (largest area)
    if not all_detections:
        return {
            'color': 'none',
            'detected': [],
            'boxes': {},
            'pixel_width': 0, 'pixel_height': 0,
            'width_mm': 0.0, 'height_mm': 0.0,
            'distance_m': 0.0,
            'orientation': 'unknown'
        }

    # Primary color is the one with largest area
    primary = max(all_detections.items(), key=lambda x: x[1]['area'])
    primary_color = primary[0]
    primary_info = primary[1]

    return {
        'color': primary_color if len(all_detections) == 1 else 'multiple',
        'detected': list(all_detections.keys()),
        'boxes': all_detections,
        'pixel_width': primary_info['pixel_width'],
        'pixel_height': primary_info['pixel_height'],
        'width_mm': primary_info['width_mm'],
        'height_mm': primary_info['height_mm'],
        'distance_m': primary_info['distance_m'],
        'orientation': primary_info['orientation']
    }


def draw_debug_overlay(frame, detection):
    """Draw detection info on frame - shows ALL detected colors with dimensions."""
    h, w = frame.shape[:2]

    # Draw each detected color's bounding box and info
    color_bgr = {'red': (0, 0, 255), 'green': (0, 255, 0), 'blue': (255, 0, 0)}

    for color_name, box in detection['boxes'].items():
        color = color_bgr.get(color_name, (255, 255, 255))

        # Draw bounding box
        bx = box['center_x'] - box['pixel_width'] // 2
        by = box['center_y'] - box['pixel_height'] // 2
        cv2.rectangle(frame, (bx, by),
                     (bx + box['pixel_width'], by + box['pixel_height']),
                     color, 2)

        # Draw info text above box
        text = f"{color_name.upper()}: {box['width_mm']:.0f}x{box['height_mm']:.0f}mm"
        cv2.putText(frame, text, (bx, by - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        # Draw distance below box
        dist_text = f"{box['distance_m']:.2f}m"
        cv2.putText(frame, dist_text, (bx, by + box['pixel_height'] + 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

    # Draw status bar at top
    detected = detection['detected']
    if detected:
        if len(detected) == 1:
            status = f"DETECTED: {detected[0].upper()}"
        else:
            status = f"DETECTED: {' + '.join([c.upper() for c in detected])}"
        status_color = (0, 255, 0)
    else:
        status = "NO DETECTION"
        status_color = (128, 128, 128)

    cv2.rectangle(frame, (5, 5), (280, 28), (0, 0, 0), -1)
    cv2.putText(frame, status, (10, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)

    # Draw dimensions at bottom
    if not detection['boxes']:
        return

    # Get primary detection for main info
    primary_box = list(detection['boxes'].values())[0]
    info = (f"Size: {primary_box['width_mm']:.0f}x{primary_box['height_mm']:.0f}mm | "
            f"Dist: {primary_box['distance_m']:.2f}m | "
            f"{primary_box['orientation']}")

    cv2.rectangle(frame, (5, h - 35), (w - 5, h - 5), (0, 0, 0), -1)
    cv2.putText(frame, info, (10, h - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

    # Draw pixel dimensions
    pixel_info = f"Pixels: {primary_box['pixel_width']}x{primary_box['pixel_height']}"
    cv2.putText(frame, pixel_info, (w - 180, h - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)


# ─────────────────────────────────────────────────────────────────────────────
# MAIN LOOP
# ─────────────────────────────────────────────────────────────────────────────

def main():
    print("=" * 60)
    print("  COLOR DETECTION TEST - Object Dimension Measurement")
    print("=" * 60)
    print("Controls:")
    print("  SPACE - Capture and print debug info")
    print("  Q     - Quit")
    print("=" * 60)

    # Open camera
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print("ERROR: Cannot open camera")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    print("Camera opened successfully")

    last_detection = None

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Frame capture failed")
            break

        # Detect
        detection = detect_color_and_dimensions(frame)
        last_detection = detection

        # Draw overlay
        draw_debug_overlay(frame, detection)

        # Show main window
        cv2.imshow('Color Detection Test', frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord(' '):
            # Print detailed info
            print("\n" + "=" * 60)
            print("  CAPTURED DETECTION")
            print("=" * 60)
            detected = detection['detected']
            print(f"Detected colors: {detected if detected else 'NONE'}")
            print("-" * 60)

            for color_name, box in detection['boxes'].items():
                print(f"{color_name.upper()}:")
                print(f"  Size: {box['width_mm']:.1f} x {box['height_mm']:.1f} mm")
                print(f"  Distance: {box['distance_m']:.3f} m")
                print(f"  Orientation: {box['orientation']}")
                print(f"  Pixels: {box['pixel_width']} x {box['pixel_height']}")
                print()

            if not detection['boxes']:
                print("No boxes detected")
            print("=" * 60)

    cap.release()
    cv2.destroyAllWindows()
    print("\nDone")


if __name__ == '__main__':
    main()