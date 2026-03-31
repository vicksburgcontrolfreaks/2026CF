---
layout: default
title: Trajectory Testing
---

# Shooter Trajectory Testing Procedure

## Overview
Systematic testing to optimize shooter trajectory angle and RPM for:
1. **Extended scoring distance** - Hit hub from farther away
2. **Ball clearing** - Launch balls from neutral/opponent zones back to our side when hub is inactive

## Hardware Setup

### Trajectory Angle Adjustment
- **Baseline**: 22 degrees
- **Adjustment**: 2-degree increments
- **Range to test**: 18° to 30° (depending on results)

Before each test session:
1. Physically adjust shooter trajectory angle hardware
2. Record the angle setting in NetworkTables

## Software Setup

### Shuffleboard Configuration
Navigate to `Shooter/TrajectoryTest` table and set:

| Parameter | Purpose | Starting Value |
|-----------|---------|----------------|
| **Test Mode Enabled** | Enable test mode | `true` |
| **Trajectory Angle (degrees)** | Current hardware angle | Match hardware (e.g., 22) |
| **Test RPM** | Shooter wheel speed | 3500 |

### Test Mode Behavior
- When `Test Mode Enabled = true`, shooter uses `Test RPM` instead of distance-based calculation
- Left trigger (manual shoot) automatically uses test RPM
- Shooter stays spinning between shots for faster testing

## Test Procedure

### For Each Trajectory Angle:

#### 1. Hardware Adjustment
- [ ] Adjust physical trajectory angle
- [ ] Verify angle with measuring tool
- [ ] Record angle in Shuffleboard (`Trajectory Angle`)

#### 2. RPM Testing at Fixed Distances

**Test Distances for Scoring:**
- 1.75m (close - baseline)
- 2.69m (mid - baseline)
- 3.3m (far - current max)
- 4.0m (extended 1)
- 4.5m (extended 2)
- 5.0m (extended 3)

**For each distance:**
1. Mark distance on field
2. Position robot
3. Set starting RPM in Shuffleboard (try 3500 first)
4. Load 3 balls into one shooter lane
5. Hold left trigger to shoot all 3 balls
6. Record results in data table below
7. Adjust RPM (±200) and repeat until you find optimal RPM
8. Move to next distance

#### 3. Clearing Shot Tests

**Scenario**: Hub inactive, balls in opponent zone, need to clear back to our side

1. Position robot at various field locations (neutral zone, opponent zone)
2. Test high-trajectory, high-RPM combinations
3. Measure distance traveled
4. Record whether balls reached our side

## Data Recording Template

### Scoring Distance Tests

| Angle | Distance | RPM  | Result (0-3 scored) | Notes |
|-------|----------|------|---------------------|-------|
| 22°   | 1.75m    | 3500 | 3/3                | Baseline close |
| 22°   | 2.69m    | 3500 | 3/3                | Baseline mid |
| 22°   | 3.3m     | 4000 | 2/3                | Baseline far |
| 22°   | 4.0m     | 4200 | 1/3                | Too low |
| 22°   | 4.0m     | 4500 | 2/3                | Better |
| 24°   | 4.0m     | 4300 | 3/3                | Good! |
| ...   | ...      | ...  | ...                 | ... |

### Clearing Shot Tests

| Angle | Robot Position | RPM  | Distance Traveled | Reached Our Side? | Notes |
|-------|----------------|------|-------------------|-------------------|-------|
| 26°   | Neutral center | 4800 | ~8m              | Yes               | High arc |
| 28°   | Opponent zone  | 5000 | ~9m              | Yes               | Very high |
| ...   | ...            | ...  | ...               | ...               | ... |

## Analysis After Testing

### Optimal Settings for Scoring
Fill in based on results:

| Distance | Optimal Angle | Optimal RPM | Success Rate |
|----------|---------------|-------------|--------------|
| 1.75m    |              |             |              |
| 2.69m    |              |             |              |
| 3.3m     |              |             |              |
| 4.0m     |              |             |              |
| 4.5m     |              |             |              |
| 5.0m     |              |             |              |

### Optimal Settings for Clearing
- **Angle**: ___°
- **RPM**: ___
- **Max Distance**: ___m

## Updating Constants

After finding optimal trajectory angle, update `ShooterConstants.java`:

```java
// Update velocity table with new distance/RPM pairs
public static final double[][] kShooterVelocityTable = {
  {1.75, XXXX},  // From test results
  {2.69, XXXX},
  {3.3,  XXXX},
  {4.0,  XXXX},  // New extended distances
  {4.5,  XXXX},
  {5.0,  XXXX},
};
```

If you want a clearing shot command, add to `TrajectoryTestConstants.java`:
```java
public static final double kOptimalTrajectoryAngle = XX.0;  // From hardware tests
public static final double kClearingShotRPM = XXXX;         // From clearing tests
```

## Safety Notes
- Test on practice field only
- Clear area behind target
- Verify ball containment
- Start with lower RPMs and increase gradually
- Stop immediately if hardware issues observed

## Tips
- Test in consistent conditions (battery voltage, temperature)
- Multiple shots per configuration for statistical significance
- Record environmental factors (wind, temperature)
- Take video for analysis
- Mark sweet spots on field with tape for quick distance reference
