# Swerve Drive Setup and Calibration Guide

## Overview
This swerve drive implementation includes:
- **Field-oriented drive** with gyro integration
- **Odometry tracking** for autonomous positioning
- **Vision pose estimation** using Limelight 4 with MegaTag2
- **PathPlanner integration** for autonomous paths
- **REV MAXSwerve modules** with SparkMax controllers and NEO Vortex motors
- **ADIS16470 IMU** for gyroscope (Analog Devices IMU via SPI)
- **LimelightHelpers** library for advanced vision integration

## Hardware Configuration

### REV MAXSwerve Module Configuration
- **Drive Motor**: NEO Vortex Brushless (SparkMax)
- **Steer Motor**: NEO 550 Brushless (SparkMax)
- **Steer Encoder**: SparkMax integrated absolute encoder
- **Pinion Gear**: 14T (configurable: 12T, 13T, or 14T)
- **Wheel Size**: 3 inches (0.0762m)
- **Gear Reduction**: Calculated from MAXSwerve internal gearing

### CAN IDs (Update in Constants.java)
Current configuration in `Constants.SwerveConstants`:
```
Front Left:  Drive=1, Steer=2
Front Right: Drive=3, Steer=4
Back Left:   Drive=5, Steer=6
Back Right:  Drive=7, Steer=8
ADIS16470 IMU: Connected via SPI (no CAN ID)
```

## Initial Setup Steps

### 1. Update Constants
Edit `src/main/java/frc/robot/Constants.java`:

#### Measure Your Robot
- **kTrackWidthMeters**: Distance from left to right wheels (center to center)
- **kWheelBaseMeters**: Distance from front to back wheels (center to center)
- **kWheelDiameterMeters**: Already set to 3 inches

#### REV MAXSwerve Gear Ratios (Configured in ModuleConstants)
- **kDrivingMotorPinionTeeth**: 14T (adjust if using 12T or 13T pinion)
- **kDrivingMotorReduction**: Automatically calculated from MAXSwerve internal gearing
  - Formula: `(45.0 * 22) / (kDrivingMotorPinionTeeth * 15)`
  - 14T pinion = ~4.71:1 reduction
- **kDrivingMotorFreeSpeedRps**: Uses NEO Vortex free speed (6784 RPM)

#### Motor Inversions
Test and adjust these if modules spin backward:
- **kDriveMotorInverted**: false (flip if wheels go backward)
- **kSteerMotorInverted**: true (flip if modules turn wrong direction)
- **kSteerEncoderInverted**: true (flip if encoder reads backward)

### 2. Initial Dependencies Check
After creating the project, run:
```bash
./gradlew build
```

This will download:
- REVLib (SparkMax support)
- PathPlanner (autonomous paths)
- ADIS16470 IMU support is included in WPILib

### 3. Module Offset Calibration (REV MAXSwerve with SparkMax)

**IMPORTANT**: For REV MAXSwerve modules with SparkMax controllers, offset calibration is done **directly on the SparkMax** using REV Hardware Client, NOT in software!

#### Calibration Process:

1. **Install REV Hardware Client**:
   - Download from: https://docs.revrobotics.com/rev-hardware-client/
   - Connect to robot via USB or WiFi

2. **Manually align all modules**:
   - Turn all wheels to point straight forward (parallel to robot)
   - Make sure bevel gears face the same direction (typically right)

3. **Zero each module using REV Hardware Client**:
   - Open REV Hardware Client
   - Select each SparkMax steer controller one at a time
   - Go to the "Absolute Encoder" tab
   - With the wheel aligned straight forward, click **"Set Position to Absolute"** or **"Burn Flash"**
   - Repeat for all four modules

4. **Verify**:
   - Deploy code to robot
   - Enable robot
   - Modules should stay pointing forward when enabled
   - When you command forward movement, all modules should point forward

**Note**: No offset constants needed in Constants.java! The calibration is stored directly on the SparkMax controllers and persists across power cycles and code deploys.

### 4. PID Tuning

#### Drive PID (Velocity Control)
Start with these values and tune:
```java
kDriveP = 0.1;
kDriveI = 0.0;
kDriveD = 0.0;
kDriveFF = 0.0;
```

Tuning process:
1. Set robot on blocks (wheels off ground)
2. Command slow forward movement
3. If wheels oscillate: decrease P
4. If wheels don't reach target speed: increase P or add FF
5. Feed-forward (FF) should be around 0.2-0.3 for NEOs

#### Steer PID (Position Control)
Start with these values and tune:
```java
kSteerP = 0.5;
kSteerI = 0.0;
kSteerD = 0.1;
```

Tuning process:
1. Command the robot to rotate in place
2. Observe modules turning to angles
3. If modules overshoot: decrease P, increase D
4. If modules are slow to reach angle: increase P
5. If modules oscillate: decrease P, increase D

## Controller Layout

### Default Controls (Xbox Controller)

**Driving:**
- Left Stick Y: Forward/Backward
- Left Stick X: Strafe Left/Right
- Right Stick X: Rotate

**Speed Modes:**
- Right Trigger: Turbo (100% speed)
- Left Trigger: Precision (30% speed)
- No Trigger: Normal (80% speed)

**Functions:**
- Start Button: Reset gyro heading to 0°
- Back Button: Toggle field-oriented drive
- X Button: Lock wheels in X formation (defense)
- Y Button: Reset odometry to origin

## Features Explained

### Field-Oriented Drive
When enabled (default), the robot moves relative to the field:
- Push stick forward → robot moves forward (regardless of robot orientation)
- Push stick left → robot moves left (regardless of robot orientation)

When disabled (robot-oriented):
- Push stick forward → robot moves in direction it's facing

### Vision Pose Estimation with MegaTag2
The Limelight 4 with MegaTag2 provides advanced AprilTag-based localization:
- **MegaTag2**: Uses multiple AprilTags simultaneously for improved accuracy
- **Automatic outlier rejection**: Filters bad measurements internally
- **Dynamic trust scaling**: Trust increases with more tags and closer distances
- **Robust geometry**: Works even with partial tag views and extreme angles

#### MegaTag2 Configuration:
1. Access Limelight web interface at http://limelight.local:5801
2. Set pipeline to **AprilTag mode**
3. Enable **MegaTag2** in AprilTag settings
4. Select **2025 Reefscape** field layout
5. Configure camera mount position and angle
6. Configure camera name in `VisionConstants.kLimelightName`

#### Using Glass for MegaTag2 Visualization:
Glass provides superior visualization for pose estimation and AprilTag tracking:

1. **Launch Glass**:
   - Open WPILib Command Palette (Ctrl+Shift+P / Cmd+Shift+P)
   - Type "Start Tool" and select "Glass"
   - Or run `gradlew glass` from terminal

2. **Connect to Robot**:
   - Glass automatically connects via NetworkTables
   - Ensure robot code is running and NetworkTables is active

3. **Key Glass Features for MegaTag2**:
   - **Field2d Widget**: Shows robot pose, vision estimates, and AprilTag locations
   - **NetworkTables Viewer**: Monitor vision data in real-time:
     - `Vision/Has Target` - Whether tags are detected
     - `Vision/Tag Count` - Number of tags being tracked by MegaTag2
     - `Vision/Avg Distance` - Average distance to visible tags
     - `Vision/Tag Span` - Geometry quality indicator
     - `Vision/Bot Pose` - Current vision-estimated pose
   - **Pose History**: Track how vision updates correct odometry drift
   - **3D Field View**: Visualize robot and tag positions in 3D

4. **Tuning with Glass**:
   - Watch pose jumps to identify bad vision measurements
   - Monitor tag count to optimize camera positioning
   - Observe how multi-tag vs single-tag affects pose quality
   - Adjust standard deviations based on observed accuracy

### PathPlanner Integration
To use PathPlanner for autonomous:

1. **Install PathPlanner GUI**:
   - Download from: https://pathplanner.dev/
   - Create paths in the GUI

2. **Save paths**:
   - Paths auto-save to: `src/main/deploy/pathplanner/paths/`
   - Autos save to: `src/main/deploy/pathplanner/autos/`

3. **Load autonomous in code**:
   ```java
   Command autoCommand = AutoBuilder.buildAuto("YourAutoName");
   ```

## Troubleshooting

### Modules Won't Turn Correctly
- Check steer motor and encoder inversions
- Verify absolute encoder offsets are calibrated
- Check CAN IDs match configuration

### Robot Drives in Wrong Direction
- Field-oriented mode might be on when you expect robot-oriented
- Check drive motor inversions
- Verify gyro is reading correctly (check SmartDashboard)

### Modules Turn More Than 90 Degrees
- This is normal and handled by optimization
- If excessive, check encoder offsets

### Vision Not Working
- Verify Limelight is on network (http://limelight.local:5801)
- Check NetworkTables connection in Driver Station
- Ensure **MegaTag2 is enabled** in AprilTag pipeline settings
- Verify **2025 Reefscape** field layout is selected
- Check camera mount position and angle are configured correctly
- Ensure team number matches in code and Limelight config
- Look for "Vision/Has Target" on SmartDashboard to verify tag detection
- Check "Vision/Tag Count" to see how many tags are being tracked

### Gyro Drifting
- ADIS16470 IMU calibrates on startup (robot MUST be stationary)
- Do not move the robot during the first 1-2 seconds after power-on
- If drift occurs, power cycle the robot while keeping it stationary
- Vision pose estimation will correct drift automatically

## Testing Checklist

- [ ] All motors respond on SmartDashboard/Shuffleboard
- [ ] Module offsets calibrated in REV Hardware Client (wheels point forward when enabled)
- [ ] Drive motors spin in correct direction
- [ ] Steer motors turn to commanded angles
- [ ] Gyro reads 0° when robot facing forward
- [ ] Field-oriented drive works correctly
- [ ] **Glass is running and connected to robot**
- [ ] **Field2d widget shows robot pose in Glass**
- [ ] Vision shows "Has Target" when AprilTag visible (check Glass NetworkTables)
- [ ] **Tag Count increases when multiple tags visible (MegaTag2 working)**
- [ ] **Pose updates smoothly in Glass when tags are visible**
- [ ] Controller inputs match expected movement

## Next Steps

1. **Tune PID values** for smooth, responsive control
2. **Test autonomous** with simple PathPlanner paths
3. **Adjust vision trust** based on field testing
4. **Create autonomous routines** in PathPlanner
5. **Practice driving** in both field and robot-oriented modes

## Support Resources

- WPILib Documentation: https://docs.wpilib.org/
- Glass Documentation: https://docs.wpilib.org/en/stable/docs/software/dashboards/glass/index.html
- ADIS16470 IMU Docs: https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/gyros-hardware.html
- PathPlanner Docs: https://pathplanner.dev/home.html
- REVLib Documentation: https://docs.revrobotics.com/
- REV Hardware Client: https://docs.revrobotics.com/rev-hardware-client/
- Limelight Docs: https://docs.limelightvision.io/
- Limelight MegaTag2 Docs: https://docs.limelightvision.io/docs/docs-limelight/apis/tracking-data-fiducials#megatag2

## Advanced Configuration

### Adjusting Max Speeds
In `SwerveConstants`:
- **kMaxSpeedMetersPerSecond**: Limit top speed (default 4.5 m/s)
- **kMaxAngularSpeedRadiansPerSecond**: Limit rotation speed (default 2π rad/s)

### Current Limits
Adjust if motors are:
- Browning out: Decrease current limits
- Underperforming: Increase current limits (within breaker ratings)

### MegaTag2 Vision Standard Deviations
The code uses dynamic standard deviations that automatically adjust based on measurement quality:

**Single Tag Mode** (`VisionConstants.kSingleTagStdDevs`):
- Default: `{1.0, 1.0, 2.0}` - Less confident when seeing only one tag
- Increase: Trust single tags less
- Decrease: Trust single tags more (only if very confident in setup)

**Multi-Tag Mode** (`VisionConstants.kMultiTagStdDevs`):
- Default: `{0.5, 0.5, 1.0}` - More confident with multiple tags
- Increase: Trust MegaTag2 less
- Decrease: Trust MegaTag2 more (typical range: 0.3-0.7)

**Additional Filtering**:
- `kMaxDistanceMeters`: Maximum distance to trust tags (default: 4.0m)
- `kMinTagArea`: Minimum tag size in frame (default: 0.01 = 1%)
- `kMinTagSpan`: Minimum spacing between tags for multi-tag (default: 10 pixels)

The system automatically scales confidence based on:
- Number of tags visible
- Average distance to tags
- Tag spread in frame (geometry quality)
- Tag size in image
