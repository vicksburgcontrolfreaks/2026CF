# FRC 2026 Testbot - Swerve Drive with Vision

A comprehensive FRC robot codebase featuring swerve drive, PathPlanner autonomous, and vision-based localization using Limelight MegaTag2.

## Features

### ✨ Core Systems

- **Swerve Drive** - REV MAXSwerve modules with field-oriented control
- **Vision Localization** - Limelight 4 with MegaTag2 for precise pose estimation
- **PathPlanner Integration** - GUI-based autonomous path creation and following
- **Pose Estimation Fusion** - Combines odometry with vision for drift correction
- **Elastic Telemetry** - Modern web-based dashboard with NetworkTables publishers

### 🎮 Driver Controls

**Xbox Controller Layout:**

| Control | Function |
|---------|----------|
| Left Stick Y | Forward/Backward |
| Left Stick X | Strafe Left/Right |
| Right Stick X | Rotate |
| Right Trigger | Turbo Mode (100% speed) |
| Left Trigger | Precision Mode (30% speed) |
| Start Button | Reset Gyro to 0° |
| Back Button | Toggle Field-Oriented Drive |
| X Button | Lock Wheels (X Formation) |
| Y Button | Reset Odometry to Origin |

### 🤖 Hardware

- **Swerve Modules**: REV MAXSwerve (NEO Vortex drive, NEO 550 steer)
- **IMU**: ADIS16470 (via SPI)
- **Vision**: Limelight 4 with MegaTag2
- **Controller**: REV Robotics roboRIO 2.0
- **Motor Controllers**: SparkMax (with absolute encoder calibration)

## Quick Start

### Prerequisites

- WPILib 2025+ installed
- REV Hardware Client
- PathPlanner GUI ([download](https://pathplanner.dev/))

### 1. Clone and Build

```bash
git clone <your-repo-url>
cd 2026CF
./gradlew build
```

### 2. How to Set Up on Your Robot

**📖 Complete setup instructions: [SETUP.md](SETUP.md)**

This guide walks you through:
- Updating CAN IDs and robot dimensions
- Calibrating swerve modules with REV Hardware Client
- Configuring PathPlanner robot settings
- Testing and tuning PID constants
- Setting up Limelight vision (optional)
- Troubleshooting common issues

**Quick setup checklist:**
1. Update CAN IDs in [Constants.java](src/main/java/frc/robot/Constants.java)
2. Calibrate swerve modules using REV Hardware Client
3. Configure robot dimensions in PathPlanner GUI
4. Test motor inversions and tune PID values
5. Configure Limelight (optional)

### 3. Deploy to Robot

```bash
./gradlew deploy
```

### 4. Access Dashboard

Open Elastic dashboard:
- **Simulation**: http://localhost:5800
- **Robot**: http://10.TE.AM.2:5800 (replace TEAM with your team number)

## Project Structure

```
src/main/
├── java/frc/robot/
│   ├── Robot.java                    # Main robot class
│   ├── RobotContainer.java           # Subsystems, commands, bindings
│   ├── Constants.java                # All configuration constants
│   ├── LimelightHelpers.java         # Vision helper library
│   ├── commands/
│   │   └── drive/
│   │       └── SwerveDriveCommand.java  # Teleop swerve control
│   └── subsystems/
│       ├── swerve/
│       │   ├── SwerveDrive.java      # Main swerve subsystem
│       │   ├── SwerveModule.java     # Individual module control
│       │   └── SwerveConfig.java     # Module configuration
│       └── vision/
│           └── VisionSubsystem.java  # MegaTag2 vision integration
├── deploy/pathplanner/
│   ├── autos/                        # Autonomous routines (.auto files)
│   └── paths/                        # Path definitions (.path files)
└── vendordeps/                       # Third-party dependencies
```

## Key Technologies

### Vision: Limelight MegaTag2

Advanced AprilTag-based localization with:
- **Multi-tag fusion** - Uses multiple tags simultaneously
- **Dynamic trust scaling** - Automatically adjusts confidence based on:
  - Number of tags visible
  - Distance to tags
  - Tag geometry in frame
- **Alliance validation** - Rejects poses on wrong side of field
- **Outlier rejection** - Filters bad measurements automatically

**Telemetry Topics:**
- `Vision/Has Target` - Tag detection status
- `Vision/Tag Count` - Number of tags tracked
- `Vision/Rejection Reason` - Why measurements rejected
- `Vision/Measurement Accepted` - Whether pose update applied

### Autonomous: PathPlanner

Create autonomous routines with PathPlanner GUI:
- Visual path editor with bezier curves
- Event markers for named commands
- Alliance color flipping (automatic)
- Holonomic path following with PPHolonomicDriveController

**Named Commands Available:**
- `stopDrive` - Stop all swerve modules
- `lockWheels` - X formation for defense

See [PATHPLANNER_SETUP.md](PATHPLANNER_SETUP.md) for detailed guide.

### Telemetry: Elastic Dashboard

Modern web-based dashboard with NetworkTables:

**SwerveDrive/** topics:
- `Gyro Angle` - Robot heading
- `Robot Pose` - Position on field
- `Field Oriented` - Drive mode status
- `FL/FR/BL/BR Velocity` - Module speeds
- `FL/FR/BL/BR Angle` - Module orientations

**Performance Optimization:**
- Critical data (pose, gyro) published every 20ms
- Detailed data (velocities, angles) published every 100ms
- Reduces NetworkTables bandwidth by ~80%

## Important Notes

### Gyro Calibration

⚠️ **CRITICAL:** Robot MUST remain stationary for 2 seconds after power-on!

The ADIS16470 IMU calibrates on startup. You'll see console messages:
```
==============================================
GYRO CALIBRATION STARTING - DO NOT MOVE ROBOT
Calibration time: 2 seconds
==============================================
```

Do not enable or move the robot during this time.

### Swerve Module Calibration

REV MAXSwerve modules store calibration **on the SparkMax controller**, not in code.

**No offset constants needed!** Calibration persists across:
- Power cycles
- Code deploys
- Software updates

Calibrate using REV Hardware Client. See [SETUP.md](SETUP.md) for instructions.

### Alliance Detection

PathPlanner automatically flips paths for red alliance:
- Blue alliance (default) - Paths as designed
- Red alliance - Paths mirrored across field centerline

Vision also validates pose based on alliance color to reject bad measurements.

### Auto Lockout Protection

If PathPlanner configuration fails to load:
- Auto chooser displays "Auto Disabled - PathPlanner Config Failed"
- Warning sent to DriverStation
- Prevents crashes during autonomous

**Solution:** Open PathPlanner GUI, configure robot, click "Save".

## Tuning Guide

### Drive PID (Wheel Velocity)

Location: [Constants.java:105-108](src/main/java/frc/robot/Constants.java#L105-L108)

```java
public static final double kDriveP = 0.1;
public static final double kDriveFF = 0.0; // Try 0.2-0.3
```

**Symptoms:**
- Wheels oscillate → Decrease P
- Wheels don't reach speed → Increase P or FF

### Steer PID (Module Rotation)

Location: [Constants.java:111-114](src/main/java/frc/robot/Constants.java#L111-L114)

```java
public static final double kSteerP = 0.5;
public static final double kSteerD = 0.1;
```

**Symptoms:**
- Modules overshoot → Decrease P, increase D
- Modules oscillate → Decrease P, increase D

### PathPlanner PID (Autonomous)

Location: [Constants.java:199-205](src/main/java/frc/robot/Constants.java#L199-L205)

```java
public static final double kPTranslation = 5.0;
public static final double kPRotation = 5.0;
```

**Symptoms:**
- Path oscillates → Decrease P
- Slow to reach path → Increase P
- Overshoots → Add D gain

### Vision Trust

Location: [Constants.java:150-156](src/main/java/frc/robot/Constants.java#L150-L156)

```java
// Single tag - less confident
public static final double[] kSingleTagStdDevs = {1.0, 1.0, 2.0};

// Multi-tag - more confident
public static final double[] kMultiTagStdDevs = {0.5, 0.5, 1.0};
```

**Higher = Trust less | Lower = Trust more**

Watch `Vision/Rejection Reason` to tune filtering thresholds.

## Troubleshooting

### Common Issues

| Problem | Solution |
|---------|----------|
| Modules turn wrong direction | Re-calibrate in REV Hardware Client |
| Robot drives backward | Flip `kDriveMotorInverted` |
| Gyro drifting | Keep stationary during 2s calibration |
| PathPlanner config failed | Open GUI, configure robot, click Save |
| Vision not working | Check Limelight config, enable MegaTag2 |
| No telemetry in Elastic | Verify NetworkTables connection |

See [SETUP.md](SETUP.md) for detailed troubleshooting.

## Code Improvements (January 2026)

Recent improvements to the codebase:

✅ **Alliance Detection** - Automatic path flipping for red alliance
✅ **Auto Lockout** - Prevents crashes if PathPlanner config missing
✅ **Vision Filtering** - Alliance validation and rejection logging
✅ **Telemetry Optimization** - Reduced bandwidth usage by 80%
✅ **Field Visualization** - Field2d published to dashboard
✅ **Gyro Calibration Warning** - Clear console messages during startup
✅ **Constants Cleanup** - Removed duplicate OIConstants
✅ **Error Handling** - Better PathPlanner configuration validation

## Documentation

- **[README.md](README.md)** - This file - Project overview, features, and important notes
- **[SETUP.md](SETUP.md)** - Complete step-by-step robot configuration guide
- **[WPILib-License.md](WPILib-License.md)** - Project license

## Resources

- **WPILib Docs**: https://docs.wpilib.org/
- **REV Docs**: https://docs.revrobotics.com/
- **PathPlanner**: https://pathplanner.dev/
- **Limelight Docs**: https://docs.limelightvision.io/
- **Elastic Dashboard**: https://docs.wpilib.org/en/stable/docs/software/dashboards/elastic.html

## License

This project is licensed under the WPILib BSD license. See [WPILib-License.md](WPILib-License.md).

## Contributing

This is a competition robot codebase. Contributions should follow FRC best practices and WPILib coding standards.

## Support

For questions or issues:
1. Check [SETUP.md](SETUP.md) troubleshooting section
2. Review relevant documentation links above
3. Check WPILib forums or Chief Delphi
4. Contact your team's software mentor

---

**Built with WPILib 2025 | REV Robotics | PathPlanner | Limelight**
