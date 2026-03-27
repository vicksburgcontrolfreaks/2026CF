# Robot Systems Documentation - 2026 Season

## Shooting System

### Overview
The shooting system is designed for accurate, distance-adjusted shooting using vision-guided targeting and dynamic velocity control. The system consists of three shooter wheels and three indexer wheels, all driven by NEO Vortex brushless motors.

---

## 1. Shooter Wheels

### Hardware Specifications

**Wheel Configuration:**
- **Quantity:** 3 shooter wheels
- **Wheel Type:** 35A 4" Stealth Shooter Wheels
  - Manufacturer: AndyMark
  - Product Link: https://andymark.com/products/stealth-and-sushi-wheels
  - Diameter: 4 inches (101.6 mm)
  - Durometer: 35A (compliant contact surface)
  - Material: Polyurethane tread on plastic hub

**Motor Configuration:**
- **Motor Type:** NEO Vortex Brushless DC Motor
  - Manufacturer: REV Robotics
  - Product Code: REV-21-1652
  - Product Link: https://www.revrobotics.com/rev-21-1652/
  - Free Speed: 6,784 RPM
  - Stall Torque: 3.6 N⋅m (2.7 ft⋅lbs)
  - Stall Current: 211 A
  - Free Current: 3.6 A
  - Motor Weight: 1.1 lbs (499g)
  - **Gear Ratio:** 1:1 (Direct Drive - No Gearing)

**Flywheel System:**
- **Flywheel Mass:** 1 pound per wheel (0.454 kg)
- **Total Rotating Mass:** ~1.5 lbs (including motor rotor, wheel hub, flywheel)
- **Purpose:** Energy storage and shot consistency

**Controller:**
- **Type:** SparkFlex Motor Controller (CAN ID: 15, 16, middle TBD)
- **Control Mode:** Closed-loop velocity control
- **Current Limit:** 40A (configured in software)
- **Feedback:** Integrated NEO Vortex encoder

### Performance Characteristics

**Operating Range:**
- **Minimum Operating RPM:** 3,000 RPM
- **Maximum Operating RPM:** 4,500 RPM
- **Optimal Range:** 3,500 - 4,000 RPM (to be determined through testing)

**Velocity Calculations:**
- At 3,000 RPM: 50 rev/s → Surface speed ≈ 52.4 ft/s (15.9 m/s)
- At 4,500 RPM: 75 rev/s → Surface speed ≈ 78.5 ft/s (23.9 m/s)

**Current Draw Estimates:**
- Spin-up phase: ~60-80A per motor (brief)
- Steady-state no load: ~5-8A per motor
- Under game piece load: ~15-30A per motor (varies with shot frequency)
- **Configured Limit:** 40A per motor (safety threshold)

**Spin-up Time:**
- Estimated time to reach 3,500 RPM: 0.5-1.0 seconds (with flywheel inertia)
- Time varies with flywheel mass and PID tuning

### Control Strategy

**PID Control:**
- Velocity control using SparkFlex onboard PID
- PID gains configured in `ShooterConfig.java`
- Real-time velocity monitoring via encoder feedback

**Dynamic RPM Adjustment:**
- Vision subsystem provides distance-to-target measurement
- Odometry fusion provides accurate robot pose
- Linear interpolation from lookup table (distance → RPM)
- Continuous RPM updates during operation for shooting while moving

**Motor Coordination:**
- Three shooter motors configured independently
- Middle shooter may run inverted (TBD based on mechanism geometry)
- All motors commanded to same target RPM
- Velocity tolerance checking ensures synchronization before shot

---

## 2. Indexer System

### Hardware Specifications

**Wheel Configuration:**
- **Quantity:** 3 indexer wheels
- **Wheel Type:** 35A 2.25" Compliant Indexer Wheels
  - Manufacturer: AndyMark
  - Product Link: https://andymark.com/products/compliant-wheels
  - Diameter: 2.25 inches (57.15 mm)
  - Durometer: 35A (soft, compliant contact)
  - Material: Polyurethane

**Motor Configuration:**
- **Motor Type:** NEO Vortex Brushless DC Motor
  - Same specifications as shooter motors (see above)
  - **Gear Ratio:** 1:1 (Direct Drive - No Gearing)

**Controller:**
- **Type:** SparkFlex Motor Controller (CAN ID: 13, 14, middle TBD)
- **Control Mode:** Closed-loop velocity control
- **Current Limit:** 40A (configured in software)
- **Feedback:** Integrated NEO Vortex encoder

### Performance Characteristics

**Operating Range:**
- **Minimum Operating RPM:** 800 RPM
- **Maximum Operating RPM:** 1,200 RPM
- **Purpose:** Controlled feeding of game pieces into shooter wheels

**Velocity Calculations:**
- At 800 RPM: 13.3 rev/s → Surface speed ≈ 7.8 ft/s (2.4 m/s)
- At 1,200 RPM: 20 rev/s → Surface speed ≈ 11.7 ft/s (3.6 m/s)

**Current Draw Estimates:**
- Steady-state operation: ~8-15A per motor
- Under compression (game piece feeding): ~20-35A per motor
- **Configured Limit:** 40A per motor (safety threshold)

**Feed Rate:**
- Indexer speed tuned to maintain consistent game piece presentation to shooter
- Lower RPM than shooter prevents premature ejection
- Speed coordinated with shooter spin-up time

### Control Strategy

**Motor Coordination:**
- Left and right indexers run at same RPM (forward)
- Middle indexer runs inverted (negative RPM) - different geometry
- All three motors synchronized for even game piece feeding

**Integration with Shooter:**
- Indexer activated only after shooter reaches target velocity
- Velocity tolerance check prevents mis-timed shots
- Sequential operation: shooter spin-up → velocity check → indexer feed

---

## 3. Vision and Targeting System

### Vision Hardware
- **System:** PhotonVision with multiple camera configuration
- **Cameras:** 4 cameras (front, back, left, right)
- **Target Detection:** AprilTag recognition for field positioning
- **Target:** Speaker position (alliance-dependent)

### Pose Estimation
- **Primary:** Vision-based pose estimation using AprilTags
- **Secondary:** Swerve drive odometry (wheel encoders + gyro)
- **Fusion:** SwerveDrivePoseEstimator combines vision and odometry
- **Update Rate:** 50 Hz odometry, variable vision (10-30 Hz)

### Distance Calculation
- Robot pose from vision/odometry fusion
- Speaker position from field geometry (alliance-dependent)
- Euclidean distance calculation in 2D plane
- Real-time distance updates for dynamic shooting

### Targeting Capabilities
- **Automatic Alignment:** RotateToTargetCommand calculates required rotation
- **Dynamic Shooting:** Shooter RPM continuously updated based on distance
- **Shooting While Moving:** Pose updates allow accurate shots during motion

---

## 4. Planned Tuning and Testing

### Shooter Wheel Testing
**Objectives:**
1. Characterize RPM vs. distance relationship
2. Determine optimal RPM for various field positions
3. Measure shot accuracy at different RPMs
4. Validate current draw under various conditions
5. Optimize PID gains for fast, stable spin-up

**Test Points (preliminary):**
- Close range (1.0-2.0m): ~3,000-3,500 RPM
- Medium range (2.0-4.0m): ~3,500-4,000 RPM
- Far range (4.0-6.0m): ~4,000-4,500 RPM

**Data Collection:**
- Distance to target (m)
- Required shooter RPM
- Indexer RPM
- Shot success rate
- Motor current draw
- Spin-up time

### Indexer Wheel Testing
**Objectives:**
1. Determine optimal feed speed for different shooter RPMs
2. Test game piece consistency and reliability
3. Measure current draw during feeding
4. Verify synchronization between three indexers

**Test Variables:**
- Indexer RPM range: 800-1,200 RPM
- Coordination with shooter velocities
- Feed timing relative to shooter spin-up

### Velocity Lookup Table
**Purpose:** Automatically adjust shooter RPM based on distance to target

**Implementation:**
- Linear interpolation between data points
- Lookup table in `ShooterConstants.kShooterVelocityTable`
- Format: `{distance_meters, target_rpm}`
- Real-time updates in `ShooterSubsystem.periodic()`

**Example Structure:**
```java
{1.0, 3000},  // Close shot
{2.0, 3300},
{3.0, 3700},
{4.0, 4000},
{5.0, 4300},
{6.0, 4500}   // Far shot
```

### Integration Testing
1. **Vision-guided shooting:** Verify automatic RPM adjustment
2. **Shooting while moving:** Test pose estimation accuracy
3. **Auto-alignment:** Validate RotateToTargetCommand accuracy
4. **Sequence timing:** Optimize shooter spin-up → velocity check → indexer feed
5. **Current monitoring:** Verify no motors exceed 40A limit
6. **Battery voltage sag:** Test performance under load

---

## 5. Current Configuration Status

### Motor IDs (SparkFlex CAN)
- Right Shooter: 15
- Left Shooter: 16
- Middle Shooter: TBD
- Right Indexer: 14
- Left Indexer: 13
- Middle Indexer: TBD

### PID Configuration
- Located in: `src/main/java/frc/robot/configs/ShooterConfig.java`
- Control type: Velocity (closed-loop)
- Feedback: Primary encoder (integrated NEO Vortex)
- Current tuning: Initial values, requires optimization

### Software Architecture
- **Subsystem:** `ShooterSubsystem.java`
- **Commands:**
  - `activateShooter()` - Spin up shooter wheels
  - `runIndexer()` - Feed game pieces
  - `stopShooter()` / `StopIndexer()` - Stop motors
- **Velocity Check:** `isAtTargetVelocity(tolerance)` - Verify ready to shoot
- **Dynamic RPM:** `getRPMForDistance(distance)` - Lookup table interpolation

### Safety Features
- Current limiting: 40A per motor
- Velocity tolerance checking before feeding
- Emergency stop capability
- Motor initialization to zero on startup

---

## 6. Next Steps

### Immediate Testing Priorities
1. **Characterize shooter velocity vs. distance**
   - Test multiple field positions
   - Record RPM, accuracy, consistency
   - Build preliminary lookup table

2. **Optimize indexer speed**
   - Test feed rates with different shooter RPMs
   - Verify game piece control
   - Measure current draw

3. **Tune PID gains**
   - Minimize spin-up time
   - Reduce velocity oscillation
   - Improve shot-to-shot consistency

4. **Validate current limits**
   - Monitor peak current during operation
   - Adjust limits if needed
   - Ensure battery voltage stability

5. **Vision integration testing**
   - Verify distance calculation accuracy
   - Test automatic RPM adjustment
   - Validate shooting while moving

### Future Enhancements
- Feedforward gain tuning for faster response
- Shot prediction for moving targets
- Automatic shot sequencing
- Performance telemetry and logging
- Competition-optimized velocity profiles

---

## Appendix A: Key Formulas

**Surface Velocity of Wheel:**
```
v = π × D × (RPM / 60)
where:
  v = surface velocity (m/s or ft/s)
  D = wheel diameter (m or ft)
  RPM = rotational speed (revolutions per minute)
```

**Distance to Target:**
```
distance = sqrt((x_robot - x_target)² + (y_robot - y_target)²)
```

**Linear Interpolation:**
```
rpm = rpm1 + (rpm2 - rpm1) × (distance - d1) / (d2 - d1)
where:
  rpm1, rpm2 = table RPM values
  d1, d2 = table distance values
  distance = current distance to target
```

---

**Document Version:** 1.0
**Last Updated:** 2026-03-26
**Status:** Initial characterization - pending tuning data
