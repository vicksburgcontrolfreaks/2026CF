# 2026CF FRC Robot Code

**Team:** Vicksburg Control Freaks
**Season:** 2026 FIRST Robotics Competition
**Framework:** WPILib Command-Based Architecture
**Language:** Java 17

---

## Table of Contents
- [Project Overview](#project-overview)
- [Hardware Components](#hardware-components)
- [Subsystems](#subsystems)
- [Commands](#commands)
- [Autonomous Modes](#autonomous-modes)
- [Controller Bindings](#controller-bindings)
- [Dependencies](#dependencies)
- [Getting Started](#getting-started)
- [Configuration](#configuration)
- [Key Features](#key-features)

---

## Project Overview

This repository contains the robot code for the 2026 FRC season. The robot features a sophisticated swerve drive system with multi-camera AprilTag vision, a velocity-controlled shooter with distance-based RPM adjustment, and an automated collection/indexing system.

**Key Capabilities:**
- 360° swerve drive with field-relative control
- 4-camera AprilTag-based localization
- Vision-guided autonomous shooting
- Distance-adaptive shooter RPM control
- Automated game piece collection and indexing

---

## Hardware Components

### Drivetrain (Swerve Drive)
- **Type:** 4-wheel swerve drive
- **Dimensions:** 26.5" × 26.5" (square chassis)
- **Max Speed:** 4.8 m/s translational, 3π rad/s rotational
- **Motors:** 8x REV SPARKmax (NEO brushless)
  - 4x drive motors (CAN IDs: 2, 4, 6, 8)
  - 4x turn motors (CAN IDs: 3, 5, 7, 9)
- **IMU:** ADIS16470 gyroscope (SPI interface)

**Module Layout:**
```
Front-Left (2,3)    Front-Right (4,5)
        [Robot Front]
Rear-Left (8,9)     Rear-Right (6,7)
```

### Shooter System
- **Purpose:** Launches game pieces into scoring targets
- **Motors:** REV SPARKFlex (NEO Vortex brushless)
  - 3x Shooter wheels (CAN IDs: 17, 18, 19)
  - 3x Indexer wheels (CAN IDs: 14, 15, 16)
  - 1x Floor feeder (CAN ID: 13)
- **Max RPM:** 6784 (NEO Vortex motor spec)
- **Control:** Closed-loop velocity PID with feedforward

**Distance-Based RPM Table:**
| Distance to Speaker | Target RPM |
|---------------------|------------|
| 1.75 m              | 2950 RPM   |
| 2.69 m              | 3450 RPM   |
| 3.30 m              | 4000 RPM   |

### Collector System
- **Purpose:** Intake and store game pieces
- **Motors:** REV SPARKFlex/SPARKmax
  - 2x Collector rollers (CAN IDs: 10, 11)
  - 1x Hopper actuator (CAN ID: 12)
- **Hopper Positions:**
  - Up (retracted): 0.01 rotations
  - Down (extended): 0.180 rotations
- **Collection Speed:** 35% power (configurable)

### Vision System
- **Cameras:** 4x Arducam OV9281 (PhotonVision)
  - **Front:** -7.17" forward, 20.28" up, 6° pitch
  - **Back:** -10" forward, 8.27" left, 20.28" up, 180° yaw
  - **Left:** 3.35" right, 13.75" left, 20.28" up, 90° yaw
  - **Right:** 3.35" right, -13.75" right, 20.28" up, -90° yaw
- **Field Layout:** 2026 Rebuilt Welded AprilTag layout
- **Pose Estimation:** Multi-camera confidence-weighted fusion
- **Max Detection Distance:** 5.0 meters
- **Max Ambiguity:** 0.2 (for pose rejection)

---

## Subsystems

### 1. DriveSubsystem
**Location:** [src/main/java/frc/robot/subsystems/drive/DriveSubsystem.java](src/main/java/frc/robot/subsystems/drive/DriveSubsystem.java)

Controls the robot's swerve drive movement system.

**Features:**
- Field-relative and robot-relative drive modes
- Multiple speed modes: Normal (50%), Precision (25%), Turbo (90%)
- Integrated gyroscope heading tracking
- Vision-fused pose estimation via SwerveDrivePoseEstimator
- Choreo trajectory following for autonomous
- Real-time odometry and field visualization

**Key Methods:**
- `drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative)` - Main drive control
- `setX()` - Lock wheels in X-pattern (defense mode)
- `resetOdometry(Pose2d pose)` - Reset position estimate
- `resetPoseEstimatorWithVision()` - Align pose using vision at autonomous start

### 2. ShooterSubsystem
**Location:** [src/main/java/frc/robot/subsystems/shooter/ShooterSubsystem.java](src/main/java/frc/robot/subsystems/shooter/ShooterSubsystem.java)

Manages the game piece launching mechanism with velocity control.

**Features:**
- Closed-loop PID velocity control
- Distance-based RPM lookup with linear interpolation
- Vision-guided dynamic RPM adjustment
- Separate control for shooter wheels, indexers, and floor feeder
- Motor current monitoring (60A limit per motor)
- NetworkTables tuning support

**Key Methods:**
- `spinUpToVisionVelocity(double distance)` - Set RPM based on distance to target
- `shooterOn()/shooterOff()` - Control main shooter wheels
- `indexerOn()/indexerOff()` - Control indexer wheels
- `floorOn()/floorOff()` - Control floor feeder

**PID Configuration:**
- P: 0.00045
- I: 0.00000025
- D: 0.0001
- FF: 0.000442 (feedforward)

### 3. CollectorSubsystem
**Location:** [src/main/java/frc/robot/subsystems/collector/CollectorSubsystem.java](src/main/java/frc/robot/subsystems/collector/CollectorSubsystem.java)

Controls game piece intake and storage.

**Features:**
- Dual roller intake system
- Position-controlled hopper mechanism
- Forward/reverse operation
- Current monitoring for jam detection
- Configurable speeds via NetworkTables

**Key Methods:**
- `runCollector(boolean reverse)` - Start/stop collector rollers
- `stopCollector()` - Stop all collector motion
- `extendHopper()/retractHopper()` - Position control
- `hopperHalfway()` - Intermediate hopper position

**Hopper PID Configuration:**
- P: 0.5
- I: 0.001
- D: 0.0

### 4. PhotonVisionSubsystem
**Location:** [src/main/java/frc/robot/subsystems/vision/PhotonVisionSubsystem.java](src/main/java/frc/robot/subsystems/vision/PhotonVisionSubsystem.java)

Provides AprilTag-based localization and target tracking.

**Features:**
- 4-camera 360° AprilTag detection
- Multi-camera pose fusion with confidence weighting
- Dynamic standard deviation adjustment (based on distance and tag count)
- Speaker distance calculation for shooter RPM
- Pose rejection filters (ambiguity, distance, tag count)
- Field layout integration (2026 Rebuilt Welded)

**Key Methods:**
- `getEstimatedGlobalPose()` - Fused pose estimate from all cameras
- `getSpeakerDistance()` - Distance to alliance speaker target
- `getRobotPoseUsingVisionAtStart()` - Initial pose from vision (autonomous)
- `getLatestAprilTagIDs()` - Currently visible tag IDs

**Pose Estimation Strategy:**
- Single tag: Higher std deviations (distance-dependent)
- Multiple tags: Lower std deviations (more confident)
- Confidence-weighted average across all cameras
- Gyro-corrected yaw for stability

---

## Commands

### Autonomous Commands

#### ExtendBackupAndShootCommand
**Location:** [src/main/java/frc/robot/commands/auton/ExtendBackupAndShootCommand.java](src/main/java/frc/robot/commands/auton/ExtendBackupAndShootCommand.java)

Default autonomous routine.

**Sequence:**
1. Extend hopper to collect pre-loaded game piece
2. Back up 1 meter at 0.5 m/s
3. Stop and stabilize
4. Auto-aim at speaker using vision
5. Shoot for 5 seconds with full system (shooter + indexer + floor)

#### RedRightLoopAndShootCommand
**Status:** Partially implemented, trajectory following commented out

Uses Choreo trajectory generation for complex autonomous paths.

### Collector Commands

| Command | Description | Binding |
|---------|-------------|---------|
| `RunCollectorCommand` | Run collector rollers (forward/reverse) | Mechanism A + X buttons |
| `StopCollectorCommand` | Stop all collector motion | Mechanism B button |
| `ExtendHopperCommand` | Move hopper to extended position | Mechanism POV Right |
| `RetractHopperCommand` | Move hopper to retracted position | Mechanism POV Left |
| `ExtendHopperHalfwayCommand` | Position hopper halfway | Part of sequence |
| `HopperHalfwaySequenceCommand` | Execute hopper positioning sequence | Mechanism POV Up |

### Shooter Commands

#### ShooterWithAutoAimCommand
**Location:** [src/main/java/frc/robot/commands/shooter/ShooterWithAutoAimCommand.java](src/main/java/frc/robot/commands/shooter/ShooterWithAutoAimCommand.java)

Primary shooting command with vision-guided aiming.

**Features:**
- Spins up shooter to vision-calculated RPM (already running in auto)
- PID-controlled rotation to face speaker target
- Maintains driver translational control (strafe while aiming)
- Left trigger override for manual rotation control
- Activates indexer and floor feeder when ready
- Held on right trigger (mechanism controller)

**Auto-Aim PID:**
- Targets alliance speaker position
- Rotation controlled via PID
- Driver retains X/Y movement control

#### ShooterCommand
Basic shooter control without auto-aim (legacy).

### Drive Commands

#### RotateToTargetCommand
**Location:** [src/main/java/frc/robot/commands/drive/RotateToTargetCommand.java](src/main/java/frc/robot/commands/drive/RotateToTargetCommand.java)

PID-based rotation to face a target position.

**Usage:**
- Bound to driver Y button
- Rotates robot to face speaker
- Completes when within tolerance

---

## Autonomous Modes

### Current Default: ExtendBackupAndShootCommand
Located in [Robot.java:116](src/main/java/frc/robot/Robot.java#L116)

**Strategy:**
1. **Initialization:**
   - Spin up shooter wheels to target RPM
   - Reset pose estimator using vision data
   - Align gyro with vision-detected heading

2. **Execution:**
   - Extend hopper (collect pre-load)
   - Drive backward 1m to create shooting distance
   - Auto-aim using PID rotation toward speaker
   - Fire for 5 seconds with full system

3. **Vision Integration:**
   - Initial pose set from AprilTag detection
   - Continuous pose updates during movement
   - Distance-based RPM calculation for optimal shot

### Future Autonomous Options
- **Choreo Trajectories:** RedRightLoopAndShootCommand (currently disabled)
- Support for PathPlanner autonomous paths
- Multi-piece autonomous routines

---

## Controller Bindings

### Driver Controller (Port 0)
**Movement:**
- **Left Stick Y:** Forward/backward translation
- **Left Stick X:** Left/right strafe
- **Right Stick X:** Rotation

**Speed Modes:**
- **Right Bumper (hold):** Turbo mode (90% max speed)
- **Left Bumper (hold):** Precision mode (25% max speed)
- **Default:** Normal mode (50% max speed)

**Special Functions:**
- **Y Button:** Rotate to face speaker target
- **Start Button:** Zero gyro heading (reset forward direction)

### Mechanism Controller (Port 1)
**Collector:**
- **A Button (hold):** Run collector + indexer forward
- **B Button:** Stop collector
- **X Button (hold):** Run collector + floor forward

**Hopper:**
- **POV Up:** Hopper halfway sequence
- **POV Left:** Retract hopper (up position)
- **POV Right:** Extend hopper (down position)

**Shooter:**
- **Right Trigger (hold):** Shoot with auto-aim
  - Auto-aims at speaker using vision
  - Maintains shooter RPM
  - Activates indexer + floor feeder
  - **Left Trigger:** Override auto-aim (manual rotation)

---

## Dependencies

### Core Libraries
- **WPILib 2026.2.1** - FRC framework and utilities
- **WPILibNewCommands** - Command-based architecture
- **REVLib** - REV Robotics motor controller drivers
- **PhotonLib** - AprilTag vision processing
- **PathPlannerLib 2026.1.2** - Path planning and trajectory generation
- **ChoreoLib 2026** - Advanced trajectory generation
- **JUnit 5** - Unit testing framework

### Vendor Dependencies
Vendor JSON files located in `vendordeps/`:
- `ChoreoLib2026.json`
- `PathplannerLib-2026.1.2.json`
- `photonlib.json`
- `REVLib.json`
- `WPILibNewCommands.json`

### Build Tool
- **Gradle 8.10** with GradleRIO plugin
- Java 17 toolchain

---

## Getting Started

### Prerequisites
- **WPILib 2026.2.1** installed
- **Java 17** JDK
- **Git** for version control
- **FRC Driver Station** software

### Building and Deploying

1. **Clone the repository:**
   ```bash
   git clone <repository-url>
   cd 2026CF
   ```

2. **Build the project:**
   ```bash
   ./gradlew build
   ```

3. **Deploy to robot:**
   ```bash
   ./gradlew deploy
   ```

4. **Run unit tests:**
   ```bash
   ./gradlew test
   ```

### Simulation

Run robot simulation for testing without hardware:
```bash
./gradlew simulateJava
```

### Project Structure
```
2026CF/
├── src/main/java/frc/robot/
│   ├── Robot.java                  # Main robot class
│   ├── RobotContainer.java         # Subsystems and bindings
│   ├── subsystems/                 # Hardware subsystems
│   │   ├── drive/                  # Swerve drive
│   │   ├── shooter/                # Shooter system
│   │   ├── collector/              # Collector/hopper
│   │   └── vision/                 # PhotonVision
│   ├── commands/                   # Robot behaviors
│   │   ├── auton/                  # Autonomous commands
│   │   ├── collector/              # Collector commands
│   │   ├── drive/                  # Drive commands
│   │   └── shooter/                # Shooter commands
│   ├── constants/                  # Configuration constants
│   └── configs/                    # Motor configurations
├── vendordeps/                     # Vendor library dependencies
├── build.gradle                    # Build configuration
└── README.md                       # This file
```

---

## Configuration

### NetworkTables Tuning

All subsystems publish configuration values to NetworkTables for live tuning:

**Shooter Tuning:**
- `Shooter/TargetRPM` - Base shooter wheel velocity
- `Shooter/ShooterP/I/D/FF` - Velocity PID gains
- `Shooter/RPMTable/*` - Distance-based RPM lookup values

**Collector Tuning:**
- `Collector/CollectorSpeed` - Roller speed percentage
- `Collector/HopperP/I/D` - Hopper position PID gains
- `Collector/HopperUpPosition` - Retracted position
- `Collector/HopperDownPosition` - Extended position

**Drive Tuning:**
- `Drive/MaxSpeedMetersPerSecond` - Maximum translational speed
- `Drive/MaxAngularSpeed` - Maximum rotational speed
- Various PID and feedforward gains for modules

**Vision Configuration:**
- Camera transforms in [PhotonVisionConstants.java](src/main/java/frc/robot/constants/PhotonVisionConstants.java)
- Pose rejection thresholds (ambiguity, distance)
- Standard deviation matrices

### Constants Files

Located in [src/main/java/frc/robot/constants/](src/main/java/frc/robot/constants/):
- `CollectorConstants.java` - Collector/hopper configuration
- `DriveConstants.java` - Swerve drive parameters
- `PhotonVisionConstants.java` - Camera and vision settings
- `ShooterConstants.java` - Shooter/indexer configuration

### Motor Configuration

Located in [src/main/java/frc/robot/configs/](src/main/java/frc/robot/configs/):
- `MotorConfig.java` - Motor controller base configuration
- Device IDs, current limits, idle modes

---

## Key Features

### 1. Multi-Camera Vision Fusion
- 4 cameras provide 360° AprilTag coverage
- Confidence-weighted pose averaging
- Dynamic standard deviation based on detection quality
- Robust pose rejection filters

### 2. Distance-Adaptive Shooter
- Vision-calculated distance to speaker
- Linear interpolation between RPM table points
- Continuous closed-loop velocity control
- Configurable via NetworkTables

### 3. Auto-Aim Shooting
- PID-controlled rotation to target
- Driver retains translational control
- Manual override capability
- Integrated with shooter RPM management

### 4. Advanced Swerve Drive
- Field-relative control by default
- Multiple speed modes (precision, normal, turbo)
- Gyro-fused odometry
- X-pattern wheel lock for defense

### 5. Choreo Trajectory Support
- Integration with Choreo trajectory generator
- PID-based path following
- Vision-corrected paths during autonomous

### 6. Comprehensive Telemetry
- All subsystems publish to NetworkTables
- Real-time tuning without redeployment
- Field2d visualization in Glass/SmartDashboard
- Motor current monitoring and diagnostics

---

## Contributing

### Current Branch
- Main development: `main`
- Active feature branch: `Aaden-auton`

### Workflow
1. Create feature branch from `main`
2. Make changes and test in simulation
3. Test on physical robot
4. Create pull request to `main`
5. Review and merge

### Code Style
- Follow WPILib Java conventions
- Use descriptive variable/method names
- Document complex algorithms
- Keep command logic separate from subsystem logic

---

## Troubleshooting

### Common Issues

**Vision Not Detecting Tags:**
- Check PhotonVision web interface (http://photonvision.local:5800)
- Verify camera connections and power
- Check AprilTag field layout matches competition field
- Ensure adequate lighting conditions

**Swerve Modules Not Aligning:**
- Verify encoder absolute offsets in DriveConstants
- Check module CAN IDs match physical layout
- Recalibrate module offsets if hardware changed

**Shooter RPM Unstable:**
- Check motor temperature and current draw
- Verify PID gains in ShooterConstants
- Ensure adequate battery voltage (>12V)
- Inspect wheel/belt condition

**Autonomous Path Not Following:**
- Verify pose estimator is initialized correctly
- Check that vision is detecting tags at autonomous start
- Confirm trajectory file exists and is loaded
- Review PID gains for trajectory following

### Diagnostic Tools
- **NetworkTables:** Monitor all subsystem states
- **Glass/SmartDashboard:** Visualize field position and paths
- **FRC Driver Station:** View console output and errors
- **REV Hardware Client:** Configure and test SPARK motor controllers

---

## Credits

**Team:** Vicksburg Control Freaks
**Season:** 2026 FIRST Robotics Competition
**Framework:** WPILib Command-Based Architecture

**Key Technologies:**
- WPILib (FRC control system)
- PhotonVision (AprilTag vision processing)
- REV Robotics (motor controllers)
- PathPlanner & Choreo (autonomous path planning)

---

## License

This project is intended for FRC competition use by the Vicksburg Control Freaks team.

---

## Additional Resources

- [WPILib Documentation](https://docs.wpilib.org/)
- [PhotonVision Documentation](https://docs.photonvision.org/)
- [REV Robotics Documentation](https://docs.revrobotics.com/)
- [PathPlanner Documentation](https://pathplanner.dev/)
- [Choreo Documentation](https://sleipnirgroup.github.io/Choreo/)
- [FRC Control System Documentation](https://docs.wpilib.org/en/stable/docs/controls-overviews/control-system-hardware.html)
