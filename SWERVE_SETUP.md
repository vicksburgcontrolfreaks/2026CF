# Swerve Drive Setup and Calibration Guide

## Overview
This swerve drive implementation includes:
- **Field-oriented drive** with gyro integration
- **Odometry tracking** for autonomous positioning
- **Vision pose estimation** using Limelight v4
- **PathPlanner integration** for autonomous paths
- **REV SparkMax** motor controllers with NEO motors
- **ADIS16470 IMU** for gyroscope (Analog Devices IMU via SPI)

## Hardware Configuration

### Motors per Module
- **Drive Motor**: NEO Brushless (SparkMax)
- **Steer Motor**: NEO 550 Brushless (SparkMax)
- **Steer Encoder**: SparkMax integrated absolute encoder

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

#### Determine Gear Ratios
- **kDriveGearRatio**: Currently set to 6.75:1 (SDS Mk4i L2)
  - Measure: Drive motor rotations per wheel rotation
  - Common values: L1=8.14, L2=6.75, L3=6.12
- **kSteerGearRatio**: Currently set to 150/7 = 21.43:1
  - Measure: Steer motor rotations per module rotation

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

### 3. Module Offset Calibration

**IMPORTANT**: This step is critical for proper swerve operation!

#### Calibration Process:

1. **Manually align all modules**:
   - Turn all wheels to point straight forward (parallel to robot)
   - Make sure bevel gears face the same direction (typically right)

2. **Read absolute encoder positions**:
   - Deploy code to robot
   - Open SmartDashboard or Shuffleboard
   - Look for values like "FL Absolute Encoder", "FR Absolute Encoder", etc.
   - Record these values (they will be in rotations, 0.0 to 1.0)

3. **Update offsets in Constants.java**:
   ```java
   public static final double kFrontLeftOffset = 0.234;  // Your recorded value
   public static final double kFrontRightOffset = 0.567;
   public static final double kBackLeftOffset = 0.891;
   public static final double kBackRightOffset = 0.123;
   ```

4. **Verify**:
   - Redeploy code
   - Enable robot
   - Modules should stay pointing forward when enabled
   - When you command forward movement, all modules should point forward

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

### Vision Pose Estimation
The Limelight v4 provides AprilTag-based pose correction:
- Automatically updates robot position when AprilTags are visible
- Trust is adjusted based on distance and number of tags
- Configure camera name in `VisionConstants.kLimelightName`

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
- Verify Limelight is on network (http://limelight.local)
- Check NetworkTables connection
- Ensure AprilTag pipeline is selected
- Verify team number matches in code and Limelight config

### Gyro Drifting
- ADIS16470 IMU calibrates on startup (robot MUST be stationary)
- Do not move the robot during the first 1-2 seconds after power-on
- If drift occurs, power cycle the robot while keeping it stationary
- Vision pose estimation will correct drift automatically

## Testing Checklist

- [ ] All motors respond on SmartDashboard/Shuffleboard
- [ ] Absolute encoders show values between 0.0 and 1.0
- [ ] Module offsets calibrated (wheels point forward when enabled)
- [ ] Drive motors spin in correct direction
- [ ] Steer motors turn to commanded angles
- [ ] Gyro reads 0° when robot facing forward
- [ ] Field-oriented drive works correctly
- [ ] Vision shows "Has Target" when AprilTag visible
- [ ] Robot pose updates on Field2d widget
- [ ] Controller inputs match expected movement

## Next Steps

1. **Tune PID values** for smooth, responsive control
2. **Test autonomous** with simple PathPlanner paths
3. **Adjust vision trust** based on field testing
4. **Create autonomous routines** in PathPlanner
5. **Practice driving** in both field and robot-oriented modes

## Support Resources

- WPILib Documentation: https://docs.wpilib.org/
- ADIS16470 IMU Docs: https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/gyros-hardware.html
- PathPlanner Docs: https://pathplanner.dev/home.html
- REVLib Documentation: https://docs.revrobotics.com/
- Limelight Docs: https://docs.limelightvision.io/

## Advanced Configuration

### Adjusting Max Speeds
In `SwerveConstants`:
- **kMaxSpeedMetersPerSecond**: Limit top speed (default 4.5 m/s)
- **kMaxAngularSpeedRadiansPerSecond**: Limit rotation speed (default 2π rad/s)

### Current Limits
Adjust if motors are:
- Browning out: Decrease current limits
- Underperforming: Increase current limits (within breaker ratings)

### Vision Standard Deviations
In `VisionConstants.kVisionStdDevs`:
- Increase values: Trust vision less (useful if seeing bad tags)
- Decrease values: Trust vision more (useful with good field setup)

Default: `{0.7, 0.7, 999999}` (don't trust rotation from vision)
