# Field-Oriented Drive Fix for Alliance-Specific Control

## Problem
Field-oriented drive was not working correctly for the Red alliance. When drivers pushed "forward" on the joystick, the robot would drive in the wrong direction relative to the field.

## Root Cause
The FRC field coordinate system is defined with:
- **0° = facing toward Red alliance side** (positive X direction)
- **180° = facing toward Blue alliance side** (negative X direction, X=0)

Both alliances face **toward each other** across the field:
- **Blue alliance**: stationed at X=0, faces 0° toward red side
- **Red alliance**: stationed at far X, faces 180° toward blue side

The issue was that field-oriented drive always interpreted "forward" as driving toward positive X (0°), which is correct for Blue alliance but **backwards** for Red alliance.

## Solution
Added alliance-specific joystick input flipping in `RobotContainer.java`:

```java
// For Red alliance, flip X/Y inputs so "forward" drives toward blue alliance (X=0)
boolean isRed = DriverStation.getAlliance().isPresent() &&
                DriverStation.getAlliance().get() == Alliance.Red;
double allianceFlip = isRed ? -1.0 : 1.0;

// Apply alliance flip to X and Y joystick inputs
xSpeed = joystickX * speedMultiplier * allianceFlip;
ySpeed = joystickY * speedMultiplier * allianceFlip;
```

This ensures:
- **Blue alliance**: Forward = toward red side (positive X) ✓
- **Red alliance**: Forward = toward blue side (negative X/X=0) ✓

## Additional Fixes

### Vision Pose Reset
Fixed vision system to correctly update robot heading while disabled:

**File**: `DriveSubsystem.java` - `resetOdometry()` method
- Now calls `m_gyro.setGyroAngleY()` to sync the physical gyro with vision rotation
- Eliminates offset between gyro and vision measurements
- Ensures heading display shows correct field-relative angle (e.g., 180° when on red alliance facing blue)

**File**: `Robot.java`
- `disabledInit()`: Enables 100% vision trust for accurate pose while disabled
- `teleopInit()` and `autonomousInit()`: Switches to sensor fusion mode

### Field Layout
Updated field layout reference to correct 2026 game:
- Changed from "2025 Reefscape" to "2026 Rebuilt Welded" in `PhotonVisionSubsystem.java`

## Testing
✓ Tested on Blue alliance: Field-oriented drive works correctly
✓ Tested on Red alliance: Field-oriented drive works correctly
✓ Vision pose updates correctly while disabled
✓ Heading display shows correct field-relative angle
✓ Gyro reset button continues to function as fallback

## Files Modified
1. `src/main/java/frc/robot/RobotContainer.java` - Alliance-specific joystick flipping
2. `src/main/java/frc/robot/subsystems/drive/DriveSubsystem.java` - Vision pose reset with gyro sync
3. `src/main/java/frc/robot/Robot.java` - Vision mode switching
4. `src/main/java/frc/robot/subsystems/vision/PhotonVisionSubsystem.java` - Vision pose reset methods, field layout fix
5. `src/main/java/frc/robot/subsystems/shooter/ShooterSubsystem.java` - Fixed uninitialized publishers
6. `src/main/java/frc/robot/commands/auto/DriveAndShootCommand.java` - Simplified autonomous (no rotation needed)

## Date
March 27, 2026
