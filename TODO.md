# Project TODO

## ✅ Completed

### Elastic Migration (2026-01-05)
- ✅ Migrated from SmartDashboard to Elastic (NetworkTables publishers)
- ✅ Updated SwerveDrive subsystem to use NetworkTables publishers
- ✅ Updated VisionSubsystem to use NetworkTables publishers
- ✅ Removed deprecated SmartDashboard calls (except for auto chooser, which is correct)

### Autonomous Migration (2026-01-05)
- ✅ Removed custom autonomous commands (DriveForwardCommand, SimpleAutoCommand)
- ✅ Configured PathPlanner AutoBuilder for all autonomous routines
- ✅ Set up auto chooser with PathPlanner integration
- ✅ All autonomous now uses PathPlanner .auto files

### Documentation (2026-01-05)
- ✅ Created [ELASTIC_MIGRATION.md](ELASTIC_MIGRATION.md) - Complete guide to Elastic migration
- ✅ Updated [PATHPLANNER_SETUP.md](PATHPLANNER_SETUP.md) - Updated for Elastic and PathPlanner-only autos

## 📋 Pending

### Testing
- ⏳ Test autonomous selection via Elastic dashboard
- ⏳ Verify telemetry data appears in Elastic
- ⏳ Test PathPlanner autos (ExampleAuto, TestAuto) in simulation
- ⏳ Verify vision telemetry works correctly

### Future Enhancements
- 🔲 Create competition-specific PathPlanner autos
- 🔲 Add more named commands for game piece manipulation
- 🔲 Tune autonomous PID constants
- 🔲 Configure robot dimensions in PathPlanner GUI
- 🔲 Add custom Elastic layouts for driver/operator stations

## 📝 Notes

### Elastic Dashboard Access
- **Simulation**: http://localhost:5800
- **Robot**: http://roboRIO-TEAM-frc.local:5800 (replace TEAM with team number)

### Telemetry Organization
- **SwerveDrive/** - All swerve drive telemetry
- **Vision/** - All vision/AprilTag telemetry
- **Auto Chooser** - PathPlanner autonomous selection

### PathPlanner Files
- Autos: `src/main/deploy/pathplanner/autos/`
- Paths: `src/main/deploy/pathplanner/paths/`
