# Shoot While Driving System

## Overview

The shoot while driving system allows the robot to continuously shoot at the speaker while the driver maintains control over translation (forward/backward and left/right movement). The robot automatically keeps the back facing the speaker and adjusts shooter RPM based on distance.

## How It Works

### 1. **Robot Enable**
- Shooter **automatically spins up** when teleop is enabled
- Uses high startup current before driving begins
- Continuously maintains RPM based on distance to speaker

### 2. **Shooting Sequence Activation**
- **Mech Controller Right Trigger (hold)**: Activates shooting sequence
- Takes over robot rotation control (driver keeps translation)
- Driver left stick still controls forward/backward and left/right

### 3. **Automatic Alignment**
- Robot continuously calculates angle to speaker based on current pose
- PID controller automatically rotates the **back** of the robot to face the speaker
- Uses same PID values as RotateToTargetCommand ([AutoConstants.java](src/main/java/frc/robot/constants/AutoConstants.java:26-30))
- When right trigger is released, rotation control returns to driver immediately

### 4. **Dynamic RPM Adjustment**
- Shooter continuously reads distance to speaker from PhotonVision
- RPM interpolated from lookup table based on distance ([ShooterConstants.java](src/main/java/frc/robot/constants/ShooterConstants.java:34-41))
- Shooter automatically adjusts setpoint in real-time as robot moves

### 5. **Conditional Ball Feeding**
- Indexer **only** feeds balls when **both** conditions are met:
  1. Shooter is at target RPM (within 100 RPM tolerance)
  2. Robot is aligned to speaker (within 2° tolerance)
- If either condition fails, indexer stops feeding until both are met again
- Provides visual/tactile feedback when alignment is lost

## Implementation Details

### Modified Files

#### [Robot.java](src/main/java/frc/robot/Robot.java)
- **Line 51**: Added `spinUpShooter()` call in `teleopInit()`
- Shooter spins up automatically when teleop is enabled

#### [RobotContainer.java](src/main/java/frc/robot/RobotContainer.java)
- **Line 327-329**: Added `spinUpShooter()` method to activate shooter on enable
- **Line 208**: Updated ShooterCommand binding to pass DriveSubsystem
- Right trigger on mech controller activates shooting with alignment

#### [ShooterCommand.java](src/main/java/frc/robot/commands/shooter/ShooterCommand.java)
- **Complete rewrite**: Now takes over rotation control when activated
- Calculates angle to speaker and maintains alignment
- Only feeds balls when aligned (within 2°) AND at target RPM
- Driver retains translation control via left stick
- Rotation control returns to driver immediately when trigger released

#### [ShooterSubsystem.java](src/main/java/frc/robot/subsystems/shooter/ShooterSubsystem.java)
- **Line 318-334**: Enabled dynamic RPM calculation (was previously commented out)
- **Line 450-452**: Added `isReadyToFeed()` method to check if shooter is at target RPM
- Shooter now continuously updates setpoint in `periodic()` when active

## Usage

### During Match

1. **Enable robot** → Shooter automatically spins up (before driving)
2. **Drive normally** with driver controller (shooter stays spinning)
3. **Mech operator holds right trigger** → Shooting sequence starts:
   - Robot takes over rotation to maintain alignment
   - Driver can still translate (left stick)
   - Indexer feeds when aligned and at RPM
4. **Mech operator releases trigger** → Rotation control returns to driver
5. Shooter stays spinning for quick re-engagement

### What to Expect

**When aligned and at RPM:**
- Indexer and floor motors running
- Balls shooting continuously
- Robot maintains orientation to speaker

**When not aligned or RPM insufficient:**
- Indexer and floor motors stop
- Shooter continues spinning
- Robot continues trying to align

**While moving:**
- Shooter RPM adjusts automatically based on distance
- Robot rotation adjusts to keep back facing speaker
- Driver maintains translation control (left stick)
- Driver rotation (right stick) disabled during shooting

**Control transitions:**
- Pressing right trigger: Smooth takeover of rotation
- Releasing right trigger: Immediate return of rotation control to driver

## Tuning Parameters

### Alignment PID (Real-Time Tunable!)
**Location**: ShuffleBoard → `AlignmentPID` table

You can tune these **without redeploying** - changes take effect immediately:
- **P**: Proportional gain (default: 0.02)
- **I**: Integral gain (default: 0.0001)
- **D**: Derivative gain (default: 0.003)
- **Tolerance (deg)**: Alignment tolerance before feeding (default: 2.0°)

**See [PID_TUNING_GUIDE.md](PID_TUNING_GUIDE.md) for complete tuning instructions**

### Other Parameters

#### RPM Tolerance
- **Current**: 100 RPM ([ShooterSubsystem.java](src/main/java/frc/robot/subsystems/shooter/ShooterSubsystem.java:460))
- **Adjust**: Hardcoded in `isReadyToFeed()` method (requires redeploy)

#### Max Rotation Velocity
- **Current**: 0.5 (50% of max angular speed) ([AutoConstants.java](src/main/java/frc/robot/constants/AutoConstants.java:30))
- **Adjust**: Change `kRotateToTargetMaxVelocity` (requires redeploy)

### Velocity Lookup Table
- **Location**: [ShooterConstants.java](src/main/java/frc/robot/constants/ShooterConstants.java:34-41)
- **Update** after characterization testing with ShooterTestCommand

## Testing Recommendations

1. **Stationary First**
   - Test shoot while driving while stationary
   - Verify shooter spins up to correct RPM
   - Verify indexer only feeds when aligned

2. **Slow Movement**
   - Test with slow translation (half stick deflection)
   - Verify alignment is maintained
   - Verify RPM adjusts as distance changes

3. **Full Speed**
   - Test with full translation speed
   - Check for alignment lag or overshoot
   - Tune PID if oscillation occurs

4. **Edge Cases**
   - Test when vision loses AprilTags
   - Test near field edges
   - Test rapid direction changes

## Limitations

### Current Implementation
- **No velocity compensation** - does not account for robot velocity affecting projectile trajectory
- **Alignment required** - will not shoot if alignment tolerance is exceeded
- **Vision dependent** - requires active AprilTag detection for distance and pose

### Future Enhancements (Not Implemented)
- Velocity compensation for robot motion
- Predictive aiming for moving targets
- Fallback behavior when vision is lost
- Configurable tolerances via NetworkTables

## Troubleshooting

### Robot won't feed balls
- Check shooter is at target RPM (view in ShuffleBoard: `Shooter/Target RPM`)
- Check alignment (robot should be pointing back at speaker)
- Verify vision has AprilTag lock (view in ShuffleBoard: `Vision/Has Target`)

### Robot oscillates when trying to align
- Reduce rotation P gain in AutoConstants
- Increase rotation D gain for damping
- Check for mechanical binding in swerve modules

### Shooter RPM incorrect for distance
- Update velocity lookup table in ShooterConstants
- Run ShooterTestCommand to collect characterization data
- Verify distance reading is accurate (ShuffleBoard: `Shooter/Distance to Target`)

### Robot doesn't maintain alignment while driving
- Increase rotation PID gains
- Increase max rotation velocity
- Check IMU calibration

---

**Last Updated**: 2026-03-26
**Branch**: Shooter-Tuning
