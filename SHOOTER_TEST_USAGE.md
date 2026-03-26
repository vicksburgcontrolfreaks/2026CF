# Shooter Test System - Usage Guide

## Overview

The shooter test system automatically collects telemetry data during shooting sequences for characterization and tuning. Data is logged to CSV format for building the distance-based velocity lookup table.

## How It Works

1. **Enable autonomous mode** → Test sequence starts automatically
2. **Shooter spins up** → Records spin-up time
3. **Indexer runs** for configured duration → Samples telemetry at 50 Hz
4. **Motors stop** → Wait for operator input
5. **Operator inputs scored count** via controller
6. **Data logged to CSV** → Ready for next test

## Setup Before Testing

### 1. Configure Test Parameters (via ShuffleBoard/NetworkTables)

Navigate to the `ShooterTest` table in ShuffleBoard:

- **`Balls Loaded`**: Number of balls in hopper (default: 30)
- **`Test Duration`**: How long to run indexer in seconds (default: 5.0)
  - 8 balls ≈ 1.5 seconds
  - 30 balls ≈ 5.0 seconds
- **`Shooter RPM`**: Target shooter velocity in RPM (default: 4500)
  - Range: 3000-4500 RPM recommended
  - Adjust based on distance to target
- **`Indexer RPM`**: Target indexer velocity in RPM (default: 1500)
  - Range: 800-1200 RPM typical
  - Coordinate with shooter velocity

### 2. Select Autonomous Mode

In ShuffleBoard autonomous chooser, select:
- **"Shooter Test"**

### 3. Position Robot

- Place robot at desired test distance from speaker
- Robot should be stationary during test
- Distance will be recorded from odometry at test start

### 4. Load Hopper

- Load hopper with known number of balls
- Make sure number matches `Balls Loaded` parameter

## Running a Test

### Step 1: Enable Autonomous

- Switch to **autonomous mode**
- **Enable** the robot
- Test sequence starts immediately

### Step 2: Test Sequence Runs

You'll see status updates in `ShooterTest/Test Status`:
- "Test X: Spinning up shooter"
- "Test X: Shooting"
- "Test X: Complete - Enter scored count"

### Step 3: Input Scored Count

After test completes, use **mechanism controller**:

- **Left Bumper**: Increment scored count (+1)
- **Right Bumper**: Decrement scored count (-1)
- **Back Button**: Confirm and log data

The current scored count is visible in `ShooterTest/Balls Scored`.

### Step 4: Data Logged

- CSV file updated with test data
- Sequence number increments
- Ready for next test (reload balls, reposition robot, enable again)

## Controller Bindings (Mechanism Controller)

### During Test:
- **Left Bumper**: Increase scored ball count
- **Right Bumper**: Decrease scored ball count
- **Back Button**: Confirm and save test data

### Normal Operation (still active):
- A button: Run collector + indexer reverse
- B button: Stop all
- POV Up: Retract hopper
- POV Down: Extend hopper
- Right Trigger: Shooter command

## Data Output

### CSV File Location
`/home/lvuser/shooter_test_data.csv` (on roboRIO)

### CSV Columns

| Column | Description |
|--------|-------------|
| `Sequence_ID` | Test number (auto-increment) |
| `Timestamp` | Date and time of test |
| `Distance_m` | Distance to speaker (meters) |
| `Balls_Loaded` | Number of balls loaded |
| `Balls_Scored` | Number of balls scored (manual input) |
| `Success_Rate` | Percentage (calculated) |
| `Target_RPM` | Commanded shooter RPM |
| `Avg_RPM` | Average actual shooter RPM during sequence |
| `Min_RPM` | Minimum shooter RPM during sequence |
| `Max_RPM` | Maximum shooter RPM during sequence |
| `SpinUp_Time_s` | Time to reach target velocity (seconds) |
| `Indexer_Target_RPM` | Commanded indexer RPM |
| `Indexer_Avg_RPM` | Average actual indexer RPM |
| `Indexer_Min_RPM` | Minimum indexer RPM |
| `Indexer_Max_RPM` | Maximum indexer RPM |
| `Shooter_Avg_Current` | Average shooter motor current (amps) |
| `Shooter_Peak_Current` | Peak shooter motor current (amps) |
| `Indexer_Avg_Current` | Average indexer motor current (amps) |
| `Voltage_Start` | Battery voltage at test start |
| `Voltage_Min` | Minimum battery voltage during test |
| `Voltage_Min_Time_s` | When min voltage occurred (seconds offset) |
| `Voltage_Avg` | Average battery voltage during test |
| `Sequence_Duration_s` | Total test duration (seconds) |
| `Fire_Rate_balls_per_sec` | Calculated firing rate |
| `Notes` | (reserved for future use) |

### Downloading CSV Data

**Option 1: FTP/SFTP**
- Connect to roboRIO via FTP
- Navigate to `/home/lvuser/`
- Download `shooter_test_data.csv`

**Option 2: USB**
- SSH into roboRIO
- Copy file to USB drive

**Option 3: RoboDash/WPILib Tools**
- Use RoboRIO web dashboard file browser

## Building the Velocity Lookup Table

### 1. Open CSV in Excel/Google Sheets

Import `shooter_test_data.csv`

### 2. Identify Best Tests

Look for tests with:
- High success rate (>80%)
- Stable RPM (small Min/Max range)
- Consistent voltage (no major sag)

### 3. Create Lookup Table

Group by distance and average the RPM values:

```java
// ShooterConstants.kShooterVelocityTable
{
  {1.0, 3000},  // 1.0m → 3000 RPM
  {1.5, 3200},
  {2.0, 3400},
  {2.5, 3600},
  {3.0, 3800},
  {4.0, 4000},
  {5.0, 4200},
  {6.0, 4500}
}
```

### 4. Enable Dynamic RPM

In `ShooterSubsystem.java`, uncomment the dynamic RPM code in `periodic()` method (lines 252-269).

## Troubleshooting

### Issue: CSV file not created
- Check roboRIO file permissions
- Verify `/home/lvuser/` directory exists
- Check Driver Station console for error messages

### Issue: Distance shows -1
- Vision subsystem not detecting AprilTags
- Robot pose not initialized
- Check PhotonVision is running

### Issue: Test never completes
- Check test duration is appropriate for ball count
- Verify shooter reaches target velocity
- Monitor telemetry in ShuffleBoard

### Issue: Data looks incorrect
- Verify PID tuning on shooter motors
- Check encoder connections
- Monitor current draw for motor issues

## Testing Workflow Recommendations

### Initial Characterization (3-4 test distances)
1. Start close (1.5-2.0m) with conservative RPM (~3200)
2. Run 3-5 tests per distance for consistency
3. Gradually increase distance in 0.5m increments
4. Adjust RPM in `ShooterConstants.kShooterTargetRPM` between tests

### Fine-Tuning (8-10 test distances)
1. Fill in gaps in distance range
2. Test edge cases (very close, very far)
3. Validate consistency with 2-3 tests per distance

### Competition Prep
1. Test with actual game pieces (not practice balls)
2. Test with full 30-ball hopper capacity
3. Verify performance across battery voltage range
4. Collect data on realistic field positions

## Notes

- CSV file appends - never overwrites existing data
- Sequence numbers persist across robot reboots
- Test during autonomous only - telemetry not active in teleop
- Collector motors do NOT run during test (hopper pre-loaded only)
- All three shooter motors and all three indexer motors are sampled and averaged

---

**Last Updated**: 2026-03-26
**System Version**: 1.0
**Branch**: Shooter-Tuning
