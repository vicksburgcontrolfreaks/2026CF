---
layout: default
title: PID Tuning Guide
---

# Real-Time PID Tuning Guide

## Overview

The alignment PID constants can be tuned in real-time via ShuffleBoard without redeploying code. This allows rapid iteration during testing to find optimal values for robot rotation control during shooting.

## Location in ShuffleBoard

Navigate to the **`AlignmentPID`** table in ShuffleBoard. You'll see four parameters:

| Parameter | Description | Default Value | Units |
|-----------|-------------|---------------|-------|
| **P** | Proportional gain - response to current error | 0.02 | - |
| **I** | Integral gain - response to accumulated error | 0.0001 | - |
| **D** | Derivative gain - response to rate of error change | 0.003 | - |
| **Tolerance (deg)** | Alignment tolerance before feeding balls | 2.0 | degrees |

## How It Works

When you hold the **mech controller right trigger** to activate shooting:
1. The ShooterCommand reads PID values from NetworkTables **every loop cycle** (50Hz)
2. Updates the rotation PID controller with new values
3. Changes take effect immediately - no redeploy needed

## Tuning Procedure

### Initial Setup

1. **Enable robot** in teleop mode
2. **Open ShuffleBoard** and navigate to `AlignmentPID` table
3. **Position robot** with clear line of sight to speaker
4. Have driver ready to control translation

### Step 1: Test Current Values

1. **Mech operator holds right trigger** to activate alignment
2. **Observe behavior**:
   - Does robot rotate toward speaker?
   - Does it oscillate (shake back and forth)?
   - Does it overshoot the target?
   - How quickly does it settle?
3. **Release trigger** and note observations

### Step 2: Adjust P Gain (Proportional)

**P gain controls response strength to error**

**If robot oscillates (shakes):**
- **Decrease P** (try reducing by 25-50%)
- Example: 0.02 → 0.015 → 0.01
- Test again after each change

**If robot responds too slowly:**
- **Increase P** (try increasing by 25-50%)
- Example: 0.02 → 0.025 → 0.03
- Test again after each change

**Goal**: Robot rotates smoothly toward target without oscillating

### Step 3: Adjust D Gain (Derivative)

**D gain provides damping to reduce oscillations**

**If robot still oscillates after P adjustment:**
- **Increase D** (try doubling)
- Example: 0.003 → 0.006 → 0.012
- Provides more damping

**If robot is sluggish or overshoots:**
- **Decrease D** slightly
- Example: 0.003 → 0.002 → 0.001

**Goal**: Smooth approach with minimal overshoot

### Step 4: Adjust I Gain (Integral)

**I gain eliminates steady-state error (small offset from target)**

**If robot settles but has small constant offset:**
- **Increase I** slightly
- Example: 0.0001 → 0.0002 → 0.0005
- Be careful - too much I can cause instability

**If robot is unstable:**
- **Decrease I** or set to 0
- Example: 0.0001 → 0.00005 → 0.0

**Goal**: Robot settles exactly on target with no offset

### Step 5: Adjust Tolerance

**Tolerance determines when indexer starts feeding**

**If balls feed too early (while still aligning):**
- **Decrease tolerance** (tighter alignment required)
- Example: 2.0° → 1.5° → 1.0°

**If balls never feed (too strict):**
- **Increase tolerance** (looser alignment acceptable)
- Example: 2.0° → 3.0° → 5.0°

**Goal**: Balance between accuracy and responsiveness

## Testing While Moving

Once alignment works while stationary:

1. **Driver translates slowly** with left stick
2. **Mech operator holds right trigger**
3. **Observe** if alignment is maintained during motion
4. **Adjust P gain** if response is too slow during movement
5. **Adjust D gain** if there's excessive lag or oscillation

## Common Issues and Solutions

### Robot oscillates continuously
- **Reduce P gain** by 30-50%
- **Increase D gain** to add damping
- Check for mechanical binding in swerve modules

### Robot doesn't reach target
- **Increase P gain** by 25-50%
- Check that max velocity isn't limiting response
- Verify vision system is providing valid pose data

### Robot overshoots then corrects repeatedly
- **Increase D gain** for more damping
- **Reduce P gain** slightly
- Check loop timing (should be 50Hz)

### Small constant offset from target
- **Increase I gain** very slightly
- Check for IMU drift or calibration issues
- Verify pose estimation is accurate

### Balls feed before fully aligned
- **Decrease tolerance** (make stricter)
- Monitor alignment error in NetworkTables
- Verify PID is actually converging

## Saving Your Values

### Temporary (Current Session Only)
Changes in ShuffleBoard persist until robot is rebooted. They are **not saved** after power cycle.

### Permanent (Update Code)
Once you find good values:

1. Open [AutoConstants.java](src/main/java/frc/robot/constants/AutoConstants.java)
2. Update the constants:
```java
public static final double kRotateToTargetP = 0.025;  // Your tuned value
public static final double kRotateToTargetI = 0.0002; // Your tuned value
public static final double kRotateToTargetD = 0.005;  // Your tuned value
public static final double kRotateToTargetTolerance = 1.5; // Your tuned value
```
3. Commit and deploy code

## Advanced Tips

### Start Conservative
- Begin with low P gain (0.01)
- Gradually increase until response is adequate
- Add D only if oscillation occurs

### One Parameter at a Time
- Change only ONE value per test
- Make small adjustments (10-25%)
- Document what works and what doesn't

### Use ShuffleBoard Graphs
- Graph the alignment error over time
- Look for oscillation patterns
- Identify settling time

### Test at Different Distances
- PID tuning may need adjustment based on distance
- Test close (2m), medium (4m), and far (6m)
- Ensure consistent performance across range

## Troubleshooting

### Values don't seem to change
- Verify you're editing the **AlignmentPID** table, not AutoConstants
- Check that ShooterCommand is actually running (right trigger held)
- Confirm NetworkTables connection is active

### Robot behavior is erratic
- Return to default values and start over
- Check for mechanical issues first
- Verify battery voltage is adequate (>11.5V)

### Can't find AlignmentPID table
- Ensure code is deployed with latest changes
- Check NetworkTables viewer in ShuffleBoard
- Try restarting ShuffleBoard

---

**Last Updated**: 2026-03-26
**Branch**: Shooter-Tuning
