# Robot Setup Guide

This guide walks you through configuring this FRC robot code on your hardware.

## Prerequisites

- WPILib 2026 installed
- REV Hardware Client installed
- PathPlanner GUI installed ([download here](https://pathplanner.dev/))
- Robot with REV MAXSwerve modules
- ADIS16470 IMU (included with REV Control System)
- Raspberry Pi 5 + PhotonVision + 4x Arducam OV9281 cameras (for vision - see [PHOTONVISION_SETUP.md](PHOTONVISION_SETUP.md))

## Step 1: Update CAN IDs and Constants

### 1.1 Configure Motor CAN IDs

Edit [Constants.java](src/main/java/frc/robot/Constants.java#L68-L79) to match your robot's CAN IDs:

```java
// SwerveConstants - Update these CAN IDs
public static final int kFrontLeftDriveMotorId = 1;
public static final int kFrontLeftSteerMotorId = 2;
public static final int kFrontRightDriveMotorId = 3;
public static final int kFrontRightSteerMotorId = 4;
public static final int kBackLeftDriveMotorId = 5;
public static final int kBackLeftSteerMotorId = 6;
public static final int kBackRightDriveMotorId = 7;
public static final int kBackRightSteerMotorId = 8;
```

### 1.2 Measure Robot Dimensions

Measure the distance between swerve module centers and update [Constants.java](src/main/java/frc/robot/Constants.java#L118-L120):

```java
// Distance from robot center to module (left-right)
public static final double kTrackWidthMeters = Units.inchesToMeters(22.0); // MEASURE YOUR ROBOT

// Distance from robot center to module (front-back)
public static final double kWheelBaseMeters = Units.inchesToMeters(22.0); // MEASURE YOUR ROBOT
```

### 1.3 Verify Gear Ratios

If using 12T or 13T pinions (instead of 14T), update [Constants.java](src/main/java/frc/robot/Constants.java#L33):

```java
public static final int kDrivingMotorPinionTeeth = 14; // Change if using 12T or 13T
```

## Step 2: Calibrate Swerve Modules

**IMPORTANT:** REV MAXSwerve modules store calibration on the SparkMax controller using the absolute encoder offset feature, NOT in code.

### 2.1 Install REV Hardware Client

Download from: https://docs.revrobotics.com/rev-hardware-client/

### 2.2 Align Wheels Using REV Calibration Jig (Recommended Method)

**REV provides a calibration jig specifically for MAXSwerve modules:**

1. Power on robot
2. Place the REV MAXSwerve calibration jig on each module (one at a time)
3. The jig ensures the wheel is at exactly 0° (straight forward)
4. With jig installed, follow steps in 2.3 below
5. Remove jig and repeat for next module

**Alternative (if no jig available):**
- Manually align each wheel to point straight forward
- Use a straight edge or level to ensure precise alignment
- Ensure bevel gears face the same direction on all modules

### 2.3 Set Absolute Encoder Offset in REV Hardware Client

For each of the 4 steer motors:

1. Open REV Hardware Client
2. Connect to robot (USB or CAN over USB-C)
3. Select the steer motor SparkMax controller
4. Go to **"Parameters"** → **"Absolute Encoder"** section
5. With wheel aligned using jig, click **"Set Absolute Encoder Zero Offset"**
6. Click **"Burn Flash"** to save the offset permanently to the SparkMax
7. Remove jig and repeat for all four modules

**What this does:**
- Sets the absolute encoder's zero position to match the wheel's forward position
- The offset is stored on the SparkMax, not in robot code
- No constants needed in Constants.java for module offsets

### 2.4 Verify Calibration

1. Power cycle the robot (important - ensures offsets are loaded)
2. Deploy code to robot
3. Enable robot in teleop mode
4. **All four wheels should point straight forward when enabled**
5. Try driving - modules should maintain correct orientation

**If wheels don't align correctly:**
- Check that you clicked "Burn Flash" for each module
- Verify encoder direction setting (may need to invert encoder in SparkMax config)
- Re-calibrate any misaligned modules using the jig
- Check Constants.java line 96: `kSteerEncoderInverted` - may need to toggle

**Note:** Since you're using the REV calibration method, no offset constants are needed in code! The SparkMax handles this automatically.

## Step 3: Configure PathPlanner

### 3.1 Install PathPlanner GUI

Download from: https://pathplanner.dev/

### 3.2 Open Your Project

1. Launch PathPlanner
2. File → Open Project
3. Navigate to your project folder (e.g., `c:\Code Projects\2026CF`)

### 3.3 Configure Robot Settings

**CRITICAL:** This generates the required `pplib_commands.json` config file.

1. Click Settings (gear icon)
2. Under "Robot Config" tab, enter:
   - **Mass**: ~50 kg (measure your actual robot)
   - **MOI**: ~6.0 kg·m² (calculate or estimate)
   - **Wheel Base**: Your measured wheelbase in meters
   - **Track Width**: Your measured track width in meters
   - **Wheel Radius**: 0.0381 m (for 3" wheels)
   - **Max Module Speed**: 3.0 m/s
   - **Max Drive Acceleration**: 3.0 m/s²
   - **Max Angular Velocity**: 540 deg/s
   - **Max Angular Acceleration**: 720 deg/s²
3. Click "Save" - this creates `deploy/pathplanner/pplib_commands.json`

### 3.4 Create Your First Path (Optional)

1. Click `+` in Paths panel
2. Name it (e.g., "TestPath")
3. Click on field to add waypoints
4. Save (auto-saves to `deploy/pathplanner/paths/`)

### 3.5 Create Your First Auto (Optional)

1. Click `+` in Autos panel
2. Name it (e.g., "TestAuto")
3. Set starting pose by clicking on field
4. Drag your path from Paths panel into the auto
5. Save (auto-saves to `deploy/pathplanner/autos/`)

## Step 4: Configure PhotonVision (Optional)

For complete PhotonVision setup instructions with 4 cameras, see **[PHOTONVISION_SETUP.md](PHOTONVISION_SETUP.md)**.

**Quick PhotonVision checklist:**
1. Flash PhotonVision image to Raspberry Pi 5 SD card
2. Connect and configure all 4 cameras (front, back, left, right)
3. Calibrate each camera using PhotonVision web UI
4. Update camera positions in [Constants.java](src/main/java/frc/robot/Constants.java) PhotonVisionConstants section
5. Test AprilTag detection and pose estimation

### 4.1 Tune Vision Standard Deviations

After testing, you may want to adjust PhotonVisionConstants in [Constants.java](src/main/java/frc/robot/Constants.java):

```java
// Single tag - less confident
public static final double[] kSingleTagStdDevs = {1.5, 1.5, 3.0};

// Multi-tag - more confident
public static final double[] kMultiTagStdDevs = {0.5, 0.5, 1.0};
```

**Higher values = trust vision less | Lower values = trust vision more**

## Step 5: Test Motor Inversions

### 5.1 Initial Test

1. Deploy code
2. Enable robot (lift off ground if possible)
3. Push forward on controller left stick

### 5.2 Check Drive Motors

If wheels spin backward when driving forward, flip drive motor inversion in [Constants.java](src/main/java/frc/robot/Constants.java#L96):

```java
public static final boolean kDriveMotorInverted = true; // Change to true
```

### 5.3 Check Steer Motors

If modules turn to wrong angles, try flipping steer motor or encoder inversion in [Constants.java](src/main/java/frc/robot/Constants.java#L97-L98):

```java
public static final boolean kSteerMotorInverted = false; // Try flipping
public static final boolean kSteerEncoderInverted = false; // Try flipping
```

## Step 6: Tune PID Constants

### 6.1 Drive PID (Wheel Velocity)

Default values in [Constants.java](src/main/java/frc/robot/Constants.java#L105-L108):

```java
public static final double kDriveP = 0.1;
public static final double kDriveI = 0.0;
public static final double kDriveD = 0.0;
public static final double kDriveFF = 0.0; // Try 0.2-0.3 for NEOs
```

**Tuning:**
- Wheels oscillate → Decrease P
- Wheels don't reach speed → Increase P or add FF
- Test on blocks with slow commands

### 6.2 Steer PID (Module Rotation)

Default values in [Constants.java](src/main/java/frc/robot/Constants.java#L111-L114):

```java
public static final double kSteerP = 0.5;
public static final double kSteerI = 0.0;
public static final double kSteerD = 0.1;
```

**Tuning:**
- Modules overshoot → Decrease P, increase D
- Modules slow to turn → Increase P
- Modules oscillate → Decrease P, increase D

### 6.3 Autonomous PID

After testing paths, tune these in [Constants.java](src/main/java/frc/robot/Constants.java#L199-L205):

```java
// PathPlanner PID constants
public static final double kPTranslation = 5.0; // Translation P gain
public static final double kITranslation = 0.0;
public static final double kDTranslation = 0.0;

public static final double kPRotation = 5.0;    // Rotation P gain
public static final double kIRotation = 0.0;
public static final double kDRotation = 0.0;
```

**Tuning:**
- Robot oscillates on path → Decrease P
- Robot slow to reach path → Increase P
- Robot overshoots → Add D gain

## Step 7: Configure Current Limits

Adjust if motors are browning out or underperforming in [Constants.java](src/main/java/frc/robot/Constants.java#L101-L102):

```java
public static final int kDriveMotorCurrentLimit = 40; // Amps
public static final int kSteerMotorCurrentLimit = 20; // Amps
```

**Guidelines:**
- Decrease if experiencing brownouts
- Increase if motors underperforming (stay within breaker ratings)
- Monitor current draw in REV Hardware Client

## Step 8: Verify Gyro

### 8.1 Gyro Calibration

**CRITICAL:** Robot MUST remain stationary during the first 2 seconds after power-on!

The gyro automatically calibrates on startup. You'll see:
```
==============================================
GYRO CALIBRATION STARTING - DO NOT MOVE ROBOT
Calibration time: 2 seconds
==============================================
```

### 8.2 Test Gyro

1. Deploy code and enable robot
2. Open Elastic dashboard or Glass
3. Check `SwerveDrive/Gyro Angle`
4. Rotate robot clockwise - angle should increase
5. Rotate robot counter-clockwise - angle should decrease

### 8.3 Troubleshooting Gyro Drift

- Ensure robot is stationary during calibration
- Power cycle robot while keeping it still
- Vision will automatically correct drift over time

## Step 9: Test Everything

### 9.1 Driving Test

- [ ] Enable robot in teleop mode
- [ ] Test forward/backward (left stick Y)
- [ ] Test strafe left/right (left stick X)
- [ ] Test rotation (right stick X)
- [ ] Test field-oriented vs robot-oriented (Back button)
- [ ] Test speed modes (triggers)

### 9.2 Vision Test (if configured)

- [ ] Open Elastic dashboard
- [ ] Check `Vision/Has Target` when AprilTag visible
- [ ] Verify `Vision/Tag Count` shows correct number
- [ ] Watch `Vision/Measurement Accepted` status
- [ ] Check `Vision/Rejection Reason` when tags rejected

### 9.3 Autonomous Test

- [ ] Open Elastic dashboard
- [ ] Select auto from "Auto Chooser"
- [ ] Enable autonomous mode
- [ ] Verify robot follows path
- [ ] Tune PID if path following is poor

### 9.4 Telemetry Check

Open Elastic dashboard at `http://10.TE.AM.2:5800` or `http://roboRIO-TEAM-frc.local:5800`

Check these NetworkTables topics exist:
- `SwerveDrive/Gyro Angle`
- `SwerveDrive/Robot Pose`
- `SwerveDrive/Field Oriented`
- `SwerveDrive/FL Velocity` (and FR, BL, BR)
- `Vision/Has Target`
- `Vision/Tag Count`
- `Vision/Rejection Reason`
- Field2d widget shows robot position

## Common Issues

### Modules Won't Turn Correctly
- Re-check absolute encoder calibration in REV Hardware Client
- Verify CAN IDs match Constants.java
- Try flipping steer motor/encoder inversions

### Robot Drives Wrong Direction
- Check if field-oriented mode is enabled (might be unexpected)
- Flip drive motor inversion in Constants.java
- Verify gyro angle is correct

### PathPlanner Config Failed Error
- Open PathPlanner GUI
- Configure robot settings
- Click "Save" to generate pplib_commands.json
- Rebuild and redeploy code

### Vision Not Working
- Ping PhotonVision: `ping photonvision.local`
- Access web interface: http://photonvision.local:5800
- Verify all cameras are connected and detected
- Check AprilTag pipeline is configured
- Ensure 2025 Reefscape field layout is selected
- Verify camera transforms in Constants.java match physical mounting

### Gyro Drifting
- Keep robot stationary during 2-second calibration
- Don't move robot immediately after power-on
- Power cycle if drift occurs
- Vision will correct drift automatically

## Next Steps

After completing setup:

1. **Create Competition Autos** - Use PathPlanner GUI to design autonomous routines
2. **Add Named Commands** - Register mechanism commands for PathPlanner event markers
3. **Tune Performance** - Fine-tune all PID constants based on driving
4. **Practice Driving** - Get drivers familiar with controls and field-oriented drive
5. **Test Vision** - Verify pose estimation works reliably on practice field

## Support Resources

- **WPILib Docs**: https://docs.wpilib.org/
- **REV Docs**: https://docs.revrobotics.com/
- **REV Hardware Client**: https://docs.revrobotics.com/rev-hardware-client/
- **PathPlanner Docs**: https://pathplanner.dev/home.html
- **PhotonVision Docs**: https://docs.photonvision.org/
- **Elastic Dashboard**: https://docs.wpilib.org/en/stable/docs/software/dashboards/elastic.html
- **ADIS16470 IMU**: https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/gyros-hardware.html

## For More Information

See [README.md](README.md) for project overview and important notes.
