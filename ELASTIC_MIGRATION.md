# Elastic Migration Summary

## What Changed

This project has been successfully migrated from **SmartDashboard** (deprecated) to **Elastic**, the modern telemetry dashboard for FRC robots.

## Files Modified

### 1. [RobotContainer.java](src/main/java/frc/robot/RobotContainer.java)
- Removed custom autonomous commands (DriveForwardCommand, SimpleAutoCommand)
- Configured PathPlanner AutoBuilder to use auto chooser
- Auto chooser is published to SmartDashboard/Elastic via `SmartDashboard.putData()`
- All autonomous routines now use PathPlanner

### 2. [SwerveDrive.java](src/main/java/frc/robot/subsystems/swerve/SwerveDrive.java)
- Replaced `SmartDashboard.putNumber/String/Boolean()` calls with NetworkTables publishers
- Created dedicated publishers for all telemetry data:
  - Gyro angle, robot pose, field orientation
  - Module velocities and angles (FL, FR, BL, BR)
  - Absolute encoder values for calibration
- All telemetry published to `SwerveDrive/` table in NetworkTables

### 3. [VisionSubsystem.java](src/main/java/frc/robot/subsystems/vision/VisionSubsystem.java)
- Replaced SmartDashboard calls with NetworkTables publishers
- Created dedicated publishers for vision data:
  - Target detection status
  - AprilTag count, distance, area, span
  - Latency and pose information
- All telemetry published to `Vision/` table in NetworkTables

### 4. Files Deleted
- `src/main/java/frc/robot/commands/autonomousCommands/DriveForwardCommand.java` ❌
- `src/main/java/frc/robot/commands/autonomousCommands/SimpleAutoCommand.java` ❌

These custom autonomous commands have been replaced with PathPlanner autos.

## What is Elastic?

**Elastic** is the official replacement for SmartDashboard and Shuffleboard, providing:
- Modern, responsive web-based UI
- Better performance with large amounts of telemetry
- Built-in field visualization
- Automatic widget generation from NetworkTables
- Mobile-friendly interface

## How to Use Elastic

### Installation
Elastic comes pre-installed with WPILib 2024+. No separate installation needed!

### Accessing Elastic

#### During Simulation
1. Run your robot code
2. Open a web browser
3. Navigate to `http://localhost:5800`

#### On Real Robot
1. Deploy code to your robot
2. Connect to robot WiFi or be on the same network
3. Navigate to `http://roboRIO-TEAM-frc.local:5800` (replace TEAM with your team number)
4. Or use the IP address: `http://10.TE.AM.2:5800`

### Using the Dashboard

#### Telemetry Data
All subsystem telemetry is organized in NetworkTables:

**SwerveDrive/** table contains:
- `Gyro Angle` - Current heading in degrees
- `Robot Pose` - Current pose on field
- `Field Oriented` - Whether field-oriented drive is enabled
- `FL/FR/BL/BR Velocity` - Wheel velocities in m/s
- `FL/FR/BL/BR Angle` - Module angles in degrees
- `FL/FR/BL/BR Absolute Encoder` - Raw encoder values

**Vision/** table contains:
- `Has Target` - Whether AprilTags are detected
- `Tag Count` - Number of tags in view
- `Avg Distance` - Average distance to tags
- `Avg Area` - Average tag area in camera frame
- `Tag Span` - Geometric spread of tags
- `Latency` - Vision processing latency
- `Bot Pose` - Robot pose from vision
- `Raw FID Count` - Raw fiducial count

#### Auto Chooser
The "Auto Chooser" widget automatically appears in Elastic and contains all PathPlanner `.auto` files from `deploy/pathplanner/autos/`:
- ExampleAuto
- TestAuto
- (Any additional autos you create)

## Migration Benefits

### Before (SmartDashboard)
- ❌ Deprecated technology
- ❌ Slower performance
- ❌ Desktop application only
- ❌ Manual widget configuration
- ❌ Less maintainable

### After (Elastic)
- ✅ Modern, actively maintained
- ✅ Faster, web-based
- ✅ Access from any device
- ✅ Automatic widget generation
- ✅ Better developer experience

## Technical Details

### NetworkTables Publishers
The migration uses the modern NetworkTables API with publishers:

```java
// Old approach (SmartDashboard)
SmartDashboard.putNumber("Gyro Angle", getHeading());

// New approach (Elastic via NetworkTables)
private final DoublePublisher m_gyroAnglePub;
m_gyroAnglePub = m_telemetryTable.getDoubleTopic("Gyro Angle").publish();
m_gyroAnglePub.set(getHeading());
```

**Benefits:**
- More efficient (publishers cache connections)
- Type-safe at compile time
- Better performance for high-frequency updates
- Clear ownership of telemetry topics

### PathPlanner Integration
PathPlanner's `AutoBuilder.buildAutoChooser()` automatically:
1. Scans `deploy/pathplanner/autos/` for `.auto` files
2. Creates a `SendableChooser<Command>` with all autos
3. Publishes to NetworkTables at "Auto Chooser" key
4. Elastic displays it as a dropdown widget

## Troubleshooting

### Elastic Not Loading
**Solution**: Check that:
1. Robot code is running
2. You're connected to robot network
3. URL is correct (`http://localhost:5800` for sim or `http://roboRIO-TEAM-frc.local:5800` for robot)

### No Telemetry Data Showing
**Solution**:
1. Ensure robot code is enabled (not just running)
2. Check NetworkTables connection in Elastic
3. Look for data under `SwerveDrive/` and `Vision/` tables

### Auto Chooser Empty
**Solution**:
1. Verify `.auto` files exist in `src/main/deploy/pathplanner/autos/`
2. Rebuild and redeploy code
3. Check console for PathPlanner configuration errors

### Want to Use Glass Instead?
Glass (WPILib's visualization tool) also works with NetworkTables publishers:
1. Launch Glass from WPILib menu
2. Add NetworkTables widgets
3. Navigate to SwerveDrive/ and Vision/ tables
4. Field2d widget shows robot pose automatically

## Next Steps

1. ✅ SmartDashboard removed
2. ✅ Elastic integration complete
3. ✅ PathPlanner autonomous configured
4. ✅ All telemetry migrated to NetworkTables

**You're all set!** Deploy your code and access Elastic to see your robot's telemetry and select autonomous routines.

## Resources

- **Elastic Documentation**: https://docs.wpilib.org/en/stable/docs/software/dashboards/elastic.html
- **NetworkTables Guide**: https://docs.wpilib.org/en/stable/docs/software/networktables/index.html
- **PathPlanner Docs**: https://pathplanner.dev/home.html
