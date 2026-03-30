# Changelog

All notable changes to the 2026 Control Freaks robot code will be documented in this file.

## [BCC Competition] - 2026-03-30

### Added
- **Camera-Free Autonomous**: New `DriveAndShootNoCameraCommand` for camera failure scenarios
  - Uses odometry-only navigation (wheel encoders + gyro)
  - Fixed 3500 RPM shooting with no alignment
  - Selectable from auto chooser as "Drive and Shoot (No Camera)"

- **Two-Piece Autonomous Routines**:
  - `BlueTwoPieceAutoCommand` - Alliance-specific two-piece auto
  - `RedTwoPieceAutoCommand` - Alliance-specific two-piece auto

- **Vision Heading Validation**: Added 45° heading delta check (`kMaxHeadingDelta`)
  - Rejects vision poses that differ >45° from gyro heading
  - Prevents single-tag 180° flip ambiguity while allowing real drift correction

- **Auto Hopper Deploy**: Hopper automatically extends to down position on teleop enable

- **Alignment Helper Method**: `DriveSubsystem.isAlignedToSpeaker()` for speaker alignment verification

### Changed
- **PhotonVisionConstants**: Updated `kMaxHeadingDelta` from infinity to 45.0 degrees
- **CollectorConstants**: Adjusted hopper position constants for competition
- **Robot.java**: Added hopper auto-deploy in `teleopInit()`

### Competition Results
- Tested through BCC competition matches 28, 34, 41, and 51
- Camera fallback options validated in match conditions
- Field-oriented drive confirmed working for both alliances

---

## [Competition Ready] - 2026-03-27

### Fixed
- **Auto-Aim Feeding Timing**: `ShooterWithAutoAimCommand` now waits for alignment before feeding
  - Added `m_feedingStarted` flag to track feeding state
  - Only starts feeding when `isAligned` AND `isAtRPM`
  - Prevents balls from launching before robot aligns to target
  - Feeding continues smoothly once started (no jitter from PID micro-corrections)

### Changed
- **Auto-Aim Rotation Constants**: Kept `kRotateToTargetP` at 0.015 (tested and working)

---

## [Field-Oriented Drive Fix] - 2026-03-27

### Fixed
- **Field-Oriented Drive for Red Alliance**: Alliance-specific joystick input flipping
  - Blue alliance: Forward = toward red side (positive X) ✓
  - Red alliance: Forward = toward blue side (negative X/X=0) ✓
  - Both alliances now have correct field-relative "forward" direction

- **Auto-Aim Orientation**: Fixed ShooterWithAutoAimCommand to point back of robot at target
  - Added 180° offset to target angle calculation
  - Matches ShooterCommand behavior (shooter is at back of robot)

- **Vision Pose Reset**: Fixed vision system to correctly update robot heading while disabled
  - `DriveSubsystem.resetOdometry()` now syncs gyro with vision rotation
  - Eliminates offset between gyro and vision measurements
  - Ensures heading display shows correct field-relative angle

### Changed
- **Vision Mode Switching**:
  - `disabledInit()`: Enables 100% vision trust (allows moving robot while disabled)
  - `teleopInit()` and `autonomousInit()`: Switches to sensor fusion mode

- **Field Layout**: Updated from "2025 Reefscape" to "2026 Rebuilt Welded"

### Files Modified
1. `RobotContainer.java` - Alliance-specific joystick flipping
2. `DriveSubsystem.java` - Vision pose reset with gyro sync, isAlignedToSpeaker() helper
3. `Robot.java` - Vision mode switching, hopper auto-deploy
4. `PhotonVisionSubsystem.java` - Vision pose reset methods, field layout, heading validation
5. `ShooterSubsystem.java` - Fixed uninitialized publishers
6. `ShooterWithAutoAimCommand.java` - Auto-aim orientation fix, feeding timing fix
7. `DriveAndShootCommand.java` - Simplified autonomous

### Testing
- ✓ Field-oriented drive works correctly for both alliances
- ✓ Vision pose updates correctly while disabled
- ✓ Auto-aim points correct end of robot at target
- ✓ Auto-aim waits for alignment before feeding
- ✓ Camera fallback options validated

---

## System Resilience

### Camera Failure Handling

**No Camera Feed from Start:**
- Vision subsystem: Gracefully continues without crashes
- Drive subsystem: Falls back to odometry-only (wheel encoders + gyro)
- Field-oriented drive: Still works (gyro maintains heading)
- Auto-aim: Degraded accuracy (use manual shooting instead)
- Manual shooting: Fully functional (left trigger, fixed 3500 RPM)
- Autonomous: "Drive and Shoot (No Camera)" option available

**Camera Loss Mid-Match:**
- Seamless transition from vision+odometry to odometry-only
- Heading remains accurate via gyro
- Position drifts gradually based on wheel slippage
- Driver switches to manual shooting (left trigger)
- Field-oriented drive continues working

### Critical Systems
- ✅ Gyro (for rotation/field-oriented)
- ✅ Wheel encoders (for odometry)
- ✅ Shooter motors (for fixed RPM)

### Recovery Strategy
1. Pre-match: If cameras down, select "Drive and Shoot (No Camera)" auto
2. Mid-match: Switch to left trigger (manual shoot), continue field-oriented drive
3. Drive coach: Assists with positioning/aiming calls

---

## Key Features

### Control Bindings

**Driver Controller:**
- Left Stick: Translational control (X/Y movement)
- Right Stick X: Rotational control
- Left Bumper: Precision speed (50%)
- Right Bumper: Turbo speed (100%)
- Start Button: Reset gyro heading

**Mechanism Controller:**
- Right Trigger: Auto-aim shooting (with alignment)
- Left Trigger: Manual shooting (fixed RPM, no alignment)
- A Button: Run collector forward
- X Button: Reverse collector (unjam)
- B Button: Stop all feed mechanisms
- POV Up: Retract hopper
- POV Down: Extend hopper

### Autonomous Options
1. RedRightLoopAndShoot (default)
2. Do Nothing
3. Drive and Shoot
4. Drive and Shoot (No Camera)
5. Red Two Piece
6. Blue Two Piece
7. Shooter Test

### Vision System
- 4x Arducam OV9281 cameras (360° coverage)
- PhotonVision AprilTag localization
- Multi-camera pose fusion
- Dynamic standard deviations based on distance/tag count
- Heading validation to prevent 180° flips

### Shooter System
- Distance-based RPM calculation
- PID velocity control
- Real-time NetworkTables tuning
- Velocity capture for analysis
- Auto-aim with rotation control
- Manual fixed-RPM fallback
