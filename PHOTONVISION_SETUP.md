# PhotonVision 4-Camera Setup Guide
## Control Freaks FRC Team - Raspberry Pi 5 with 4x Arducam OV9281

> **Branch**: `photonvision-4camera-setup`
> **Status**: In Development - PhotonVision 2025 library pending release
> **Hardware**: Raspberry Pi 5 (8GB) + 4x Arducam OV9281 Global Shutter Cameras

---

## Table of Contents
1. [Hardware Overview](#hardware-overview)
2. [Power System](#power-system)
3. [Software Installation](#software-installation)
4. [Camera Configuration](#camera-configuration)
5. [Robot Code Integration](#robot-code-integration)
6. [Calibration Process](#calibration-process)
7. [Testing & Validation](#testing--validation)
8. [Troubleshooting](#troubleshooting)

---

## Hardware Overview

### Bill of Materials
- ✅ **Raspberry Pi 5 (8GB RAM)** - Main compute unit
- ⏳ **4x Arducam OV9281 Cameras** ($49 each)
  - 1MP Monochrome Global Shutter
  - 1280x800 @ 100fps max
  - 70° FOV
  - Low distortion lens
  - USB 2.0/3.0 interface
- ⏳ **Powered USB Hub** - USB-C input for power delivery
- ⏳ **MicroSD Card** (32GB+ recommended, Class 10/UHS-I)
- ⏳ **Active Cooling** - Fan/heatsink for Pi 5
- ✅ **Custom 12V-to-USB-C PD Buck Converters** (from REV PDH)
- ⏳ **USB Cables** - High-quality USB-C and USB-A cables

### Camera Layout Plan
```
          FRONT
    [Camera-Front]
         ↑
LEFT ←  ROBOT  → RIGHT
[Cam-L]         [Cam-R]
         ↓
    [Camera-Back]
          BACK
```

**Recommended Mounting**:
- **Front**: Centered on front bumper, ~10-12" high, pitched down 10-15°
- **Back**: Centered on back bumper, ~10-12" high, pitched down 10-15°, rotated 180°
- **Left**: Centered on left side, ~10-12" high, pitched down 10-15°, rotated 90°
- **Right**: Centered on right side, ~10-12" high, pitched down 10-15°, rotated -90°

**Why This Config**:
- 360° coverage for AprilTag detection
- Overlapping fields of view at corners
- Optimal height/angle for 2025 Reefscape field tags
- Maximizes multi-tag opportunities for better pose estimation

---

## Power System

### Power Distribution
1. **REV PDH** → Custom 12V-to-USB-C PD Buck Converter
2. **Buck Converter** → USB-C input on Powered USB Hub
3. **USB Hub** → Powers Raspberry Pi 5 + 4 cameras

### Power Requirements
- **Raspberry Pi 5**: 5V @ 5A (25W max) via USB-C PD
- **Each Arducam OV9281**: ~500mA @ 5V (2.5W)
- **Total**: ~35W peak (plan for 40-45W converter)

### Wiring Checklist
- [ ] Buck converter secured to robot frame
- [ ] PDH connection uses appropriate gauge wire and breaker
- [ ] USB hub mounted securely near Pi
- [ ] All USB cables have strain relief
- [ ] Power wiring away from signal cables (CAN, Ethernet)

---

## Software Installation

### Step 1: Flash PhotonVision Image to SD Card

1. **Download PhotonVision Image for Pi 5**:
   - URL: https://photonvision.org/download
   - Select: "Raspberry Pi 5" image (2025 season)

2. **Flash with Balena Etcher or Raspberry Pi Imager**:
   ```bash
   # Windows: Use Balena Etcher GUI
   # Linux/Mac:
   sudo dd if=photonvision-raspi5.img of=/dev/sdX bs=4M status=progress
   sync
   ```

3. **Initial Boot**:
   - Insert SD card into Pi 5
   - Connect to powered USB hub
   - Wait 1-2 minutes for first boot
   - Look for solid green LED

### Step 2: Network Configuration

**Option A: Ethernet (Recommended for Competition)**
1. Connect Pi 5 Ethernet to robot radio/switch
2. Pi will get DHCP address (usually `10.TE.AM.11` where TEAM=your team #)
3. Access PhotonVision at: `http://photonvision.local:5800`

**Option B: WiFi (For Testing)**
1. Connect to Pi's WiFi hotspot: `photonvision`
2. Password: `photonvision`
3. Access UI at: `http://10.0.0.1:5800`

### Step 3: Initial PhotonVision Setup

1. **Access Web UI**: http://photonvision.local:5800
2. **Settings Tab**:
   - Set Team Number: `XXXX`
   - Set Hostname: `photonvision-pi5`
   - Enable NetworkTables: ✓
   - NT Server: `10.TE.AM.2` (RoboRIO address)
3. **Update PhotonVision** (if prompted)

---

## Camera Configuration

### Step 1: Connect Cameras One at a Time

**Start with ONE camera to avoid confusion**:
1. Connect front camera to USB hub
2. Refresh PhotonVision web UI
3. Camera should appear in "Cameras" tab
4. Name it: `camera-front`

### Step 2: Configure Each Camera

For **each camera** (`camera-front`, `camera-back`, `camera-left`, `camera-right`):

#### General Settings
- **Resolution**: 1280x800 (native)
- **FPS**: 30-60 (start with 30, increase if Pi can handle it)
- **Exposure**: Auto (initially), then tune manually
- **Brightness**: 50 (adjust for field lighting)

#### AprilTag Pipeline Settings
1. **Create New Pipeline**: "AprilTags"
2. **Detector**: AprilTag (36h11 family - FRC standard)
3. **Decimate**: 2.0 (balance speed vs accuracy)
4. **Blur**: 0.0 (global shutter doesn't need motion blur reduction)
5. **Threads**: 2 per camera (Pi 5 has 4 cores)
6. **Pose Strategy**: MULTI_TAG_PNP_ON_COPROCESSOR

#### 3D Settings (CRITICAL)
1. **Select Field Layout**: "2025 Reefscape"
2. **Camera Position** (relative to robot center):
   - Use values from `Constants.PhotonVisionConstants`
   - Example for front camera:
     - X: 0.3048m (12 inches forward)
     - Y: 0.0m (centered)
     - Z: 0.254m (10 inches up)
     - Roll: 0°
     - Pitch: -15° (pitched down)
     - Yaw: 0° (facing forward)

### Step 3: Repeat for All 4 Cameras

Add cameras one at a time:
1. Connect `camera-back` → Configure → Test
2. Connect `camera-left` → Configure → Test
3. Connect `camera-right` → Configure → Test

**Naming Convention** (must match Constants.java):
- `camera-front`
- `camera-back`
- `camera-left`
- `camera-right`

---

## Robot Code Integration

### Step 1: Install PhotonVision Vendor Dependency

**IMPORTANT**: As of December 2024, PhotonVision 2025 library is not yet released.

**When available**, install via WPILib:
1. Open WPILib Command Palette (Ctrl+Shift+P)
2. Type: "Manage Vendor Libraries"
3. Select: "Install new libraries (online)"
4. Enter URL: `https://maven.photonvision.org/repository/internal/org/photonvision/photonlib-json/1.0/photonlib-json-1.0.json`
5. Build project: `./gradlew build`

**Alternative**: Download vendordep from https://docs.photonvision.org

### Step 2: Camera Constants (Already Configured!)

The camera positions are already configured in `Constants.PhotonVisionConstants`:
- `kRobotToFrontCamera`
- `kRobotToBackCamera`
- `kRobotToLeftCamera`
- `kRobotToRightCamera`

**TODO**: Measure your actual robot and update these values!

### Step 3: PhotonVisionSubsystem (To Be Created)

Will be created in next steps with:
- Multi-camera pose estimation
- Automatic camera switching
- Quality filtering
- Integration with SwerveDrive pose estimator

### Step 4: RobotContainer Integration

You'll be able to switch between:
- **Limelight MegaTag2** (on `main` branch)
- **PhotonVision 4-camera** (on `photonvision-4camera-setup` branch)

---

## Calibration Process

### Why Calibrate?
Camera calibration corrects for:
- Lens distortion
- Sensor imperfections
- Mounting angle errors

**Result**: More accurate pose estimation!

### Calibration Tools

**Option 1: PhotonVision Built-in Calibration**
1. Print calibration board from PhotonVision docs
2. Use "Calibration" tab in PhotonVision UI
3. Follow on-screen instructions (capture 15-20 images at different angles)
4. Save calibration to camera

**Option 2: mrcal (Advanced)**
- More accurate but more complex
- See: https://docs.photonvision.org/en/latest/docs/calibration/calibration.html

### Calibration Checklist
- [ ] All 4 cameras calibrated individually
- [ ] Calibration data saved to PhotonVision
- [ ] Test with AprilTags at various distances
- [ ] Verify pose accuracy in Glass

---

## Testing & Validation

### Phase 1: Single Camera Test
1. Enable only `camera-front` in PhotonVision
2. Place robot 2-3 meters from AprilTag
3. Check NetworkTables in Glass:
   - `photonvision/camera-front/hasTarget` should be true
   - `photonvision/camera-front/targetPose` should show reasonable values
4. Move robot around, verify tracking

### Phase 2: Multi-Camera Test
1. Enable all 4 cameras
2. Position robot where multiple tags visible
3. Verify in Glass:
   - Each camera detecting tags independently
   - Combined pose estimation updating smoothly
   - No wild jumps in position

### Phase 3: Performance Validation
Monitor in PhotonVision UI:
- **FPS**: Should maintain 30fps minimum per camera
- **Latency**: < 50ms total pipeline
- **CPU Usage**: < 80% on Pi 5
- **Temperature**: < 70°C with active cooling

### Phase 4: Accuracy Testing
1. Place robot at known field positions
2. Compare PhotonVision pose vs actual position
3. Acceptable error: < 5cm translation, < 2° rotation

---

## Troubleshooting

### Cameras Not Detected
- Check USB cables (try different ports)
- Verify USB hub is powered
- Check Pi 5 power (needs full 5V 5A)
- Run `lsusb` in PhotonVision shell to see USB devices

### Low FPS / High Latency
- Reduce resolution (try 960x600 or 800x600)
- Reduce FPS target
- Increase decimation (2.0 → 3.0)
- Reduce thread count per camera
- Check CPU temperature (thermal throttling?)

### Inaccurate Pose Estimation
- Re-calibrate cameras
- Verify camera positions in Constants.java match physical mounts
- Check for lens obstructions (dirt, glare)
- Ensure tags are well-lit
- Increase minimum tag area threshold

### NetworkTables Not Connecting
- Verify team number in PhotonVision settings
- Check RoboRIO IP address (should be 10.TE.AM.2)
- Ensure robot radio/network switch is working
- Try pinging Pi from Driver Station: `ping 10.TE.AM.11`

### Pi 5 Overheating
- Install active cooling (fan + heatsink)
- Reduce camera count (test with 2-3 cameras)
- Lower resolution/FPS
- Ensure adequate airflow around Pi

---

## Performance Targets

### Minimum Acceptable
- **FPS**: 20fps per camera (80fps total across 4 cameras)
- **Latency**: < 80ms
- **Pose Update Rate**: 10Hz
- **Accuracy**: < 10cm translation

### Good Performance
- **FPS**: 30fps per camera (120fps total)
- **Latency**: < 50ms
- **Pose Update Rate**: 20Hz
- **Accuracy**: < 5cm translation

### Excellent Performance
- **FPS**: 60fps per camera (240fps total)
- **Latency**: < 30ms
- **Pose Update Rate**: 30Hz
- **Accuracy**: < 3cm translation

---

## Next Steps

1. **Order remaining hardware** (cameras, hub, SD card, cooling)
2. **Flash PhotonVision image** when hardware arrives
3. **Test single camera** setup first
4. **Create PhotonVisionSubsystem.java** (waiting for 2025 library)
5. **Calibrate all cameras**
6. **Integrate with robot code**
7. **Compare with Limelight MegaTag2** performance

---

## Resources

- **PhotonVision Docs**: https://docs.photonvision.org
- **PhotonLib Java API**: https://docs.photonvision.org/en/latest/docs/programming/photonlib/index.html
- **Arducam OV9281 Specs**: https://www.arducam.com/product/arducam-1mp-*-ov9281-global-shutter-mono-usb-camera-board/
- **Pi 5 Cooling**: https://www.raspberrypi.com/products/active-cooler/
- **FRC 2025 Game Manual**: https://firstfrc.blob.core.windows.net/frc2025/Manual/2025GameManual.pdf

---

## Branch Management

**Main Branch** (`main`):
- Limelight 4 with MegaTag2
- Production-ready
- Use for competitions

**PhotonVision Branch** (`photonvision-4camera-setup`):
- PhotonVision with 4x Arducam OV9281
- Experimental
- For testing and comparison

**Switch Between Branches**:
```bash
# Switch to Limelight setup
git checkout main

# Switch to PhotonVision setup
git checkout photonvision-4camera-setup

# Compare performance results
git diff main photonvision-4camera-setup
```

---

## Contact & Support

- **PhotonVision Discord**: https://discord.gg/wYxTwym
- **Chief Delphi PhotonVision Forum**: https://www.chiefdelphi.com/tag/photonvision
- **WPILib Forums**: https://www.chiefdelphi.com/c/technical-discussion/software/30

---

*Last Updated: December 26, 2024*
*Team: Control Freaks*
*Season: 2025 Reefscape*
