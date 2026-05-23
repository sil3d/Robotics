# Camera Calibration Guide
## Micro-ROS Autonomous Mobile Robot

**Version:** 1.0.0
**Date:** May 2026

---

## Table of Contents

1. [Overview](#1-overview)
2. [Why Camera Calibration Matters](#2-why-camera-calibration-matters)
3. [Camera Model](#3-camera-model)
4. [Calibration Pattern Types](#4-calibration-pattern-types)
5. [Calibration Procedure](#5-calibration-procedure)
6. [Understanding Calibration Results](#6-understanding-calibration-results)
7. [Using Calibration Data](#7-using-calibration-data)
8. [Troubleshooting](#8-troubleshooting)
9. [Advanced Topics](#9-advanced-topics)

---

## 1. Overview

Camera calibration is essential for accurate 3D pose estimation from 2D images.
For our AprilTag detection system, precise calibration enables:

- Accurate distance measurement to tags
- Correct orientation estimation
- Reliable navigation

This project includes a complete calibration tool (`calibrate_camera.py`) that uses
either chessboard patterns or AprilTag markers for calibration.

### Quick Start

```bash
# Run the calibration tool
python3 calibrate_camera.py

# Steps:
# 1. Print chessboard pattern
# 2. Capture 15+ images from different angles (SPACE key)
# 3. Press 'C' to compute calibration
# 4. Results saved to data/camera_calibration/camera_calibration.json
```

---

## 2. Why Camera Calibration Matters

### 2.1 Without Calibration

```
Real World                    Camera Image
────────────────              ──────────────
         Tag                              Tag
           \                                |
            \  50cm                       | 50px (wrong!)
             \                           |
              Camera ──────────────► Camera
```

The camera sees a distorted world. Without calibration:
- Distances will be wrong (50cm looks like 30cm)
- Angles will be incorrect
- Pose estimation will be unreliable

### 2.2 With Calibration

```
Real World                    Camera Image
────────────────              ──────────────
         Tag                              Tag
           \                                |
            \  50cm                       | 52px (correct!)
             \                    (accounting for lens distortion)
              Camera ──────────────► Camera
```

With proper calibration:
- Pinhole model + distortion coefficients applied
- Accurate 3D reconstruction
- Reliable pose estimation

### 2.3 Impact on Robot Navigation

| Calibration Quality | Distance Error (1m target) | Navigation Impact |
|--------------------|---------------------------|------------------|
| None | ±30% | Robot misses waypoints |
| Poor (>2px error) | ±15% | Occasional waypoint miss |
| Good (1-2px error) | ±5% | Reliable navigation |
| Excellent (<1px) | ±1% | Precise maneuvering |

---

## 3. Camera Model

### 3.1 Pinhole Camera Model

Our camera follows the pinhole model:

```
                           image plane
                         ┌──────────────┐
                         │              │
                         │    ● (cx,cy)│◄── principal point
                         │              │
                         └──────────────┘
                              ▲
                              │ focal length (f)
                              │
    World Point (X,Y,Z) ─────●───────────────────────► camera
         (Xw,Yw,Zw)         /          /
         │                 /          /
         │                /          /
         │               /          /
         └──────────────┐/──────────┐
                        │camera    │
                        │center   │
                        │(0,0,0) │
                        └─────────┘
```

### 3.2 Intrinsic Parameters

The camera matrix (K) contains internal parameters:

```python
# 3x3 camera matrix
K = [[fx,  0, cx],
     [ 0, fy, cy],
     [ 0,  0,  1]]

# Example values from our calibration:
K = [[828.4,   0.0, 337.5],    # fx,  0,  cx
     [  0.0, 812.7, 213.6],    #  0, fy,  cy
     [  0.0,   0.0,   1.0]]    #  0,  0,   1
```

| Parameter | Symbol | Description |
|-----------|--------|-------------|
| Focal length x | fx | Horizontal focal length (pixels) |
| Focal length y | fy | Vertical focal length (pixels) |
| Principal point x | cx | Image center x (typically width/2) |
| Principal point y | cy | Image center y (typically height/2) |

### 3.3 Distortion Coefficients

Real lenses have distortion. We model:

```
Point in image
     │
     │ (with distortion)
     ▼
    ●────────────► distorts outward (barrel) or inward (pincushion)
     │
     │
     │ (radial distortion)

Also: tangential distortion from lens tilt
```

```python
# Distortion coefficients (OpenCV order)
# [k1, k2, p1, p2, k3]
D = [[-1.44, 14.76, -0.006, 0.054, -37.11]]
     # k1,   k2,   p1,    p2,    k3
     # radial       tangential  radial
```

| Coefficient | Type | Effect |
|-------------|------|--------|
| k1, k2 | Radial | Barrel (-ve) or pincushion (+ve) distortion |
| p1, p2 | Tangential | Asymmetric distortion from lens tilt |
| k3 | Radial | High-order radial distortion |

### 3.4 Complete Camera Model

```
s * [u]   [fx  0 cx] [R|t] [X]
  [v] =  [ 0 fy cy]      [Y]
  [1]    [ 0  0  1]      [Z]
                           [1]

Where:
  (u,v) = pixel coordinates
  (X,Y,Z) = world point
  s = scale factor
  R,t = extrinsic parameters (rotation, translation from world to camera)
```

---

## 4. Calibration Pattern Types

### 4.1 Chessboard Pattern (Recommended)

**Advantages:**
- Very accurate (<0.5px RMS error possible)
- Well-understood mathematical properties
- Works with standard OpenCV functions

**Specification:**
- Inner corners: 9 columns × 6 rows
- Square size: 2.0 cm (physical)
- Color: Black/white alternating

```
┌───┬───┬───┬───┬───┬───┬───┬───┬───┐
│   │ ● │   │ ● │   │ ● │   │ ● │   │ ← 9 inner corners per row
├───┼───┼───┼───┼───┼───┼───┼───┼───┤
│   │ ● │   │ ● │   │ ● │   │ ● │   │
├───┼───┼───┼───┼───┼───┼───┼───┼───┤
│   │ ● │   │ ● │   │ ● │   │ ● │   │
├───┼───┼───┼───┼───┼───┼───┼───┼───┤
│   │ ● │   │ ● │   │ ● │   │ ● │   │
├───┼───┼───┼───┼───┼───┼───┼───┼───┤
│   │ ● │   │ ● │   │ ● │   │ ● │   │ ← 6 inner corners per column
└───┴───┴───┴───┴───┴───┴───┴───┴───┘
```

**Generating a chessboard:**
```bash
# Using OpenCV
python3 -c "import cv2; cv2.imwrite('chessboard.png', cv2.imread('path/to/opencv/data/chessboard.png'))"

# Or download from OpenCV samples
# https://github.com/opencv/opencv/blob/master/doc/pattern_tools/chessboard.png
```

### 4.2 AprilTag Marker

**Advantages:**
- Can be used for both calibration and robot localization
- Automatic detection and identification
- Robust to partial occlusion

**Specification:**
- Dictionary: APRILTAG_36H11
- Tag size: 10cm (use same as for localization)
- Detection: Fully automatic

```
┌─────────────────────────┐
│ ██████████████████████ │
│ ██                  ███ │
│ ██  ████  ████     ███ │
│ ██  ████  ████     ███ │
│ ██                  ███ │
│ ██████████████████████ │
│ ██  ████████        ███ │
│ ██  ██  ██  ████████ ██ │
│ ██  ██  ██  ████████ ██ │
│ ██        ████        ██ │
│ ██  ██████████  ████ ██ │
│ ██  ████  ████  ████ ██ │
│ ██                  ███ │
│ ██████████████████████ │
└─────────────────────────┘
```

### 4.3 Comparison

| Feature | Chessboard | AprilTag |
|---------|-------------|----------|
| Accuracy | ★★★★★ | ★★★★ |
| Automation | Manual corner click | Automatic |
| Dual use | No | Yes (calibration + localization) |
| Setup time | Medium | Fast |
| Robustness | ★★★★ | ★★★★★ |

---

## 5. Calibration Procedure

### 5.1 Equipment Setup

1. **Camera Setup:**
   - Mount camera at typical robot height (~20cm above ground)
   - Focus manually (lock lens)
   - Set fixed exposure, brightness, contrast

2. **Pattern Setup:**
   - Print chessboard on rigid backing (cardboard/foam)
   - Ensure pattern is flat (no warping)

3. **Environment:**
   - Good, even lighting (no harsh shadows)
   - Static scene (no movement)
   - Background contrast (avoid white-on-white)

### 5.2 Capturing Images

**Key principle:** Cover the parameter space uniformly

For accurate calibration, capture images that vary:

| Variation | Range | Why |
|-----------|-------|-----|
| Distance | 0.5m to 2.0m | Affects focal length accuracy |
| Tilt (pitch) | ±30° | Affects distortion coefficients |
| Tilt (roll) | ±30° | Affects aspect ratio |
| Position | All corners + center | Full field of view |

**Recommended poses (capture 15-20 images):**

```
Image 1: Far, centered
Image 2: Close, centered
Image 3: Left side, level
Image 4: Right side, level
Image 5: Top, tilted down
Image 6: Bottom, tilted up
Image 7: Near-left corner
Image 8: Near-right corner
Image 9: Far-left corner
Image 10: Far-right corner
Image 11: Tilted left
Image 12: Tilted right
Image 13: Very close, centered
Image 14: Very far, centered
Image 15: Roll variations
```

### 5.3 Using the Calibration Tool

```bash
# Start calibration tool
python3 calibrate_camera.py

# Controls:
# SPACE   - Capture current frame
# C       - Compute calibration
# V       - Visualize reprojection errors
# R       - Reset (clear all frames)
# Q       - Quit
```

### 5.4 Step-by-Step Instructions

1. **Start the tool:**
   ```bash
   python3 calibrate_camera.py
   ```

2. **You should see:**
   - Camera preview window
   - FPS counter
   - Frame count
   - Pattern detection status (CHESSBOARD DETECTED / No pattern)

3. **Capture frames:**
   - Hold chessboard in front of camera
   - Move to different positions
   - Press SPACE to capture
   - Watch console for "Captured (N/15)" confirmation

4. **Get 15+ valid captures:**
   - Tool requires minimum 15 frames
   - Quality indicator shows pattern detection confidence

5. **Compute calibration:**
   - Press 'C' key
   - Tool runs OpenCV calibrateCamera()
   - Results printed to console

6. **Review results:**
   - Mean reprojection error (lower is better)
   - Camera matrix (fx, fy, cx, cy)
   - Distortion coefficients

7. **Save:**
   - Results auto-saved to `data/camera_calibration/camera_calibration.json`

### 5.5 Expected Output

```
[messages] Calibration done! Mean error: 0.85 px

  Camera matrix (fx, fy, cx, cy):
  [[828.4, 0, 337.5],
   [0, 812.7, 213.6],
   [0, 0, 1]]

  Distortion coeffs:
  [[-1.44, 14.76, -0.006, 0.054, -37.11]]

[SAVE] Saved to data/camera_calibration/camera_calibration.json
```

---

## 6. Understanding Calibration Results

### 6.1 Camera Matrix

```json
{
  "camera_matrix": [
    [828.4, 0.0, 337.5],
    [0.0, 812.7, 213.6],
    [0.0, 0.0, 1.0]
  ]
}
```

**Interpreting values:**

| Parameter | Our Value | Expected Range | Interpretation |
|-----------|-----------|----------------|----------------|
| fx | 828.4 | 600-1000 | Normal (depends on resolution) |
| fy | 812.7 | 600-1000 | Close to fx (good - low distortion) |
| cx | 337.5 | ~320 | Centered (320 = width/2) |
| cy | 213.6 | ~240 | Slightly above center (240 = height/2) |

### 6.2 Distortion Coefficients

```json
{
  "dist_coeffs": [
    [-1.44, 14.76, -0.006, 0.054, -37.11]
  ]
}
```

**Interpreting values:**

| Coefficient | Value | Effect | Acceptable Range |
|-------------|-------|--------|------------------|
| k1 | -1.44 | Strong barrel distortion | -2 to +1 |
| k2 | 14.76 | High-order radial | -2 to +2 |
| p1 | -0.006 | Negligible tangential | -0.5 to +0.5 |
| p2 | 0.054 | Negligible tangential | -0.5 to +0.5 |
| k3 | -37.11 | Very high (may indicate calibration issue) | -2 to +2 |

**Note:** Large k3 value may indicate:
1. Pattern not covering full field of view
2. Insufficient tilt variation in images
3. Camera lens with significant distortion

### 6.3 Reprojection Error

```json
{
  "mean_reprojection_error_px": 0.85
}
```

| Error (px) | Quality | Notes |
|-------------|--------|-------|
| < 0.5 | Excellent | Research-grade calibration |
| 0.5 - 1.0 | Good | Production-ready |
| 1.0 - 2.0 | Acceptable | Works for navigation |
| > 2.0 | Poor | Recalibrate with more varied poses |

### 6.4 Good vs Bad Calibration

**Good calibration characteristics:**
```
✓ Mean error < 1.0 px
✓ fx and fy similar (aspect ratio ~1.0)
✓ cx ≈ image_width/2
✓ cy ≈ image_height/2
✓ k1, k2, k3 all within reasonable ranges
```

**Bad calibration indicators:**
```
✗ Mean error > 2.0 px
✗ fx/fy ratio significantly different from 1.0
✗ cx, cy far from image center
✗ Very large k3 value
✗ Calibration fails to converge
```

---

## 7. Using Calibration Data

### 7.1 JSON Format

```json
{
  "calibration_date": "2026-05-14 12:30:00",
  "frame_width": 640,
  "frame_height": 480,
  "camera_matrix": [
    [828.4, 0.0, 337.5],
    [0.0, 812.7, 213.6],
    [0.0, 0.0, 1.0]
  ],
  "dist_coeffs": [
    [-1.44, 14.76, -0.006, 0.054, -37.11]
  ],
  "mean_reprojection_error_px": 0.85,
  "frames_used": 25,
  "apriltag_size_cm": 10.0
}
```

### 7.2 Loading in Python

```python
import numpy as np
import json

def load_calibration(filepath):
    with open(filepath, 'r') as f:
        data = json.load(f)

    K = np.array(data['camera_matrix'], dtype=np.float32)
    D = np.array(data['dist_coeffs'], dtype=np.float32)
    return K, D

# Usage
K, D = load_calibration('data/camera_calibration/camera_calibration.json')
print(f"Camera matrix:\n{K}")
print(f"Distortion:\n{D}")
```

### 7.3 Loading in C++ (Arduino)

For ESP32, use hardcoded values or load from SPIFFS:

```cpp
// Hardcoded (from calibration)
float cam_matrix[9] = {
    828.4f, 0.0f, 337.5f,
    0.0f, 812.7f, 213.6f,
    0.0f, 0.0f, 1.0f
};

float dist_coeffs[5] = {
    -1.44f, 14.76f, -0.006f, 0.054f, -37.11f
};
```

### 7.4 Undistorting Images

```python
import cv2

def undistort(frame, K, D):
    h, w = frame.shape[:2]
    new_K, roi = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 1, (w, h))
    undistorted = cv2.undistort(frame, K, D, None, new_K)
    return undistorted

# Or use remapping for better performance
def undistort_remap(frame, K, D):
    h, w = frame.shape[:2]
    new_K, roi = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 1, (w, h))
    mapx, mapy = cv2.initUndistortRectifyMap(K, D, None, new_K, (w, h), 5)
    return cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)
```

### 7.5 Pose Estimation with Calibration

```python
# AprilTag pose estimation using calibrated camera
def estimate_tag_pose(corners, K, D, tag_size):
    # 3D tag object points
    half = tag_size / 2.0
    obj_points = np.array([
        [-half,  half, 0],
        [ half,  half, 0],
        [ half, -half, 0],
        [-half, -half, 0]
    ], dtype=np.float32)

    # Solve PnP
    success, rvec, tvec = cv2.solvePnP(
        obj_points, corners,
        K, D,
        flags=cv2.SOLVEPNP_ITERATIVE
    )

    return rvec, tvec

# Convert rotation vector to rotation matrix
rot_matrix, _ = cv2.Rodrigues(rvec)
```

---

## 8. Troubleshooting

### 8.1 Calibration Fails to Converge

**Symptom:** "calibrateCamera failed" or very high error

**Causes:**
1. Insufficient view variations
2. Blurry images
3. Pattern not fully visible

**Solutions:**
- Capture more images with greater variation
- Ensure pattern is in focus
- Use better lighting
- Check pattern printing (corners must be sharp)

### 8.2 Very High Distortion Coefficients

**Symptom:** k3 > 10 or similar extreme values

**Causes:**
- Not enough tilt variation
- Images all taken from similar angle

**Solutions:**
- Add images with significant tilt (±45°)
- Cover corners of image more
- Try different pattern (AprilTag instead of chessboard)

### 8.3 Mean Error > 2.0 Pixels

**Symptom:** Calibration completes but accuracy is poor

**Causes:**
1. Camera not focus-locked (zoom changes)
2. Varying lighting causing exposure changes
3. Pattern not flat (curved or bent)

**Solutions:**
1. Lock camera focus manually
2. Use fixed exposure
3. Mount pattern on rigid flat backing

### 8.4 Pattern Not Detected

**Symptom:** "No pattern detected" when pattern is visible

**Causes:**
1. Lighting too dark or too bright
2. Pattern too small or too large
3. Resolution mismatch

**Solutions:**
1. Adjust lighting
2. Ensure good contrast
3. Check pattern size matches configuration (CHESSBOARD_COLS=9, CHESSBOARD_ROWS=6)

---

## 9. Advanced Topics

### 9.1 Stereo Calibration

For depth sensing with two cameras:

```python
# Stereo calibration for dual-camera setup
ret, K1, D1, K2, D2, R, T, E, F = cv2.stereoCalibrate(
    object_points,  # 3D points for each view
    image_points1,  # 2D points in image 1
    image_points2,  # 2D points in image 2
    imageSize,
    K1, D1,  # Initial camera matrices
    K2, D2,
    flags=cv2.CALIB_FIX_INTRINSIC
)
```

### 9.2 Rolling Shutter Correction

For global shutter cameras (recommended for robotics):
- No correction needed

For rolling shutter cameras (most USB webcams):
- May introduce distortion in motion
- Consider using global shutter camera for better results

### 9.3 Temperature Compensation

Lens properties change with temperature:
- For extreme environments, calibrate at operating temperature
- Consider recalibrating seasonally

### 9.4 Auto-Exposure Issues

Most webcams adjust exposure automatically:
- **Disable auto-exposure** before calibration
- Lock exposure to fixed value
- This ensures consistent image properties

```python
# Disable auto-exposure (OpenCV)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # 0.25 = disable auto
cap.set(cv2.CAP_PROP_EXPOSURE, 0.01)       # Set fixed value
```

### 9.5 Checking Calibration Validity

```python
def validate_calibration(K, D, image):
    """Check if calibration is reasonable for this image"""
    h, w = image.shape[:2]

    # Check if principal point is near center
    cx, cy = K[0, 2], K[1, 2]
    if abs(cx - w/2) > w * 0.1 or abs(cy - h/2) > h * 0.1:
        print("WARNING: Principal point far from center")

    # Check if focal lengths are reasonable
    fx, fy = K[0, 0], K[1, 1]
    if fx < 100 or fx > 2000:
        print("WARNING: Focal length out of expected range")

    return True
```

---

## Appendix: Calibration File Format

### A.1 JSON Schema

```json
{
  "type": "object",
  "required": ["camera_matrix", "dist_coeffs"],
  "properties": {
    "calibration_date": {
      "type": "string",
      "description": "ISO timestamp of calibration"
    },
    "frame_width": {
      "type": "integer",
      "description": "Camera resolution width (pixels)"
    },
    "frame_height": {
      "type": "integer",
      "description": "Camera resolution height (pixels)"
    },
    "camera_matrix": {
      "type": "array",
      "items": {"type": "array", "items": {"type": "number"}},
      "description": "3x3 camera intrinsic matrix"
    },
    "dist_coeffs": {
      "type": "array",
      "items": {"type": "array"},
      "description": "Distortion coefficients [k1,k2,p1,p2,k3]"
    },
    "mean_reprojection_error_px": {
      "type": "number",
      "description": "Average reprojection error in pixels"
    },
    "frames_used": {
      "type": "integer",
      "description": "Number of calibration frames used"
    }
  }
}
```

---

**Document Version:** 1.0.0
**Last Updated:** May 2026
**Author:** Robotics Team