---
layout: default
title: Load-In Test Plan
---

# Load-In Day Test Plan - March 27, 2026

[Return to Home](index.md)

## Overview

**Total Available Time**: 7.5 hours (0900-1630)
**Realistic Working Time**: ~6 hours (accounting for breaks, setup, unexpected issues)
**Field Setup**: Red alliance half-field

---

## Time Budget & Priorities

**Priority 1 (Must Have)**: Robot must drive and score
**Priority 2 (Should Have)**: Optimizations and training
**Priority 3 (Nice to Have)**: Data collection for future

---

## Quick Navigation

- [Block 1: Critical Systems (60 min)](#block-1-critical-systems-validation-0900-1000---60-min)
- [Block 2: Autonomous (120 min)](#block-2-autonomous-development-1000-1200---120-min)
- [Block 3: Driver Training (75 min)](#block-3-driver-training-1200-1315---75-min)
- [Block 4: Mechanical & Pack (135 min)](#block-4-mechanical--finalization-1315-1530---135-min)
- [Go/No-Go Decisions](#gono-decision-points)
- [Data Collection](#data-collection-requirements)

---

## Block 1: Critical Systems Validation (0900-1000) - 60 min
**Goal**: Confirm robot is competition-ready for basic operation

### Test 1.1: Field-Oriented Drive & Vision Pose Reset (20 min)
**Why**: Core driving functionality, new vision reset feature must work cross-alliance

**Setup**:
- ☐ Place robot at Red 2 starting position (center)
- ☐ Driver station set to **Red alliance**
- ☐ AdvantageScope recording: `Pose/X`, `Pose/Y`, `Pose/Rotation`
- ☐ Have tape measure ready

**Test Procedure**:
1. Power on robot (disabled), wait 5 seconds
2. Record initial pose on dashboard: X=_____ Y=_____ Rot=_____
3. Physically move robot 1 meter toward blue side (while disabled)
4. Record new pose - **should update instantly** (vision reset working)
5. Enable teleop
6. **Driver: Push forward** → should move toward blue side (opponent)
7. **Driver: Strafe right** → should move right relative to field
8. **Driver: Zero gyro** (Start button)
9. Repeat steps 6-7 to verify zero worked

**Test Blue Alliance on Red Field**:
10. Disable robot
11. Change driver station to **Blue alliance**
12. Enable teleop
13. **Driver: Push forward** → should STILL move toward opponent (now toward red side)
14. **Verify controls flipped** - forward now means opposite direction

**Success Criteria**:
- ☐ Pose updates instantly while disabled (< 1 second lag)
- ☐ Forward = toward opponent on Red alliance
- ☐ Forward direction flips when switching to Blue alliance
- ☐ Strafing matches field orientation

**Data to Log**:
```
Initial Pose: X=_____ Y=_____ Rot=_____
After Move:   X=_____ Y=_____ Rot=_____
Vision Update Time: _____ seconds
```

**Decision Point**:
- ✅ If vision reset works → Continue
- ❌ If fails → Disable vision reset, use manual gyro zero only (save 20 min)

---

### Test 1.2: Auto-Aim Functionality (20 min)
**Why**: Primary scoring mechanism, must work reliably

**Setup**:
- ☐ Position robot 2.5m from speaker (measure with tape)
- ☐ AdvantageScope recording: `Shooter/Target RPM`, `Alignment Error`, `Vision/Distance To Speaker`
- ☐ Load 5 game pieces in hopper
- ☐ Driver station: Red alliance

**Test Procedure**:
1. Enable robot
2. **Hold Right Trigger** (mechanism controller)
3. Observe:
   - Robot auto-rotates to face speaker
   - Shooter spins up
   - Feeding starts when aligned
4. **Keep holding trigger** - should feed all 5 pieces
5. Release trigger after shooting
6. **Repeat from different angles**:
   - 0° (facing speaker)
   - 45° (angled)
   - 90° (side)
   - 180° (back to speaker)

**Test Translation Control During Auto-Aim**:
7. Hold Right Trigger
8. **Driver: Strafe left/right** while aligned
9. **Driver: Drive forward/back** while aligned
10. Verify translation works while auto-aim maintains rotation

**Success Criteria**:
- ☐ Robot rotates to face speaker (back toward target)
- ☐ Alignment within 3° tolerance
- ☐ Shooter spins up before feeding
- ☐ Feeding starts automatically when aligned + at RPM
- ☐ Driver retains translation control during auto-aim
- ☐ No jitter when aligned

**Data to Log**:
```
Test 1 (0°):   Time to first shot: _____ s | Success: Y/N
Test 2 (45°):  Time to first shot: _____ s | Success: Y/N
Test 3 (90°):  Time to first shot: _____ s | Success: Y/N
Test 4 (180°): Time to first shot: _____ s | Success: Y/N
Alignment settling time: _____ s
```

**Decision Point**:
- ✅ Smooth and accurate → Continue
- ⚠️ Jittery → Tune PID on-the-fly (NetworkTables)
- ❌ Unreliable → Add manual shooting mode (fixed RPM)

---

### Test 1.3: Pre-Spin RPM Cap Validation (20 min)
**Why**: New feature needs validation before competition

**Setup**:
- ☐ AdvantageScope recording:
  - `Shooter/Target RPM`
  - `Shooter/Motor 1 Velocity`
  - `Battery Voltage`
  - `Shooter/Motor 1 Current`
- ☐ Mark distances on field: 2m, 3m, 4m from speaker

**Test Procedure**:

**Part A: Pre-Spin Cap Behavior**
1. Enable robot (shooter active, trigger NOT held)
2. Drive to 2m from speaker → Observe RPM (should be ~2950, no cap)
3. Drive to 3m → Observe RPM (should cap at 3500)
4. Drive to 4m → Observe RPM (should cap at 3500)

**Part B: Trigger Pull Spin-Up**
5. Position at 4m from speaker
6. Observe current RPM (should be 3500)
7. **Pull trigger** and start timer
8. Observe RPM ramp: 3500 → 4000
9. Record spin-up time when feeding starts

**Part C: Current Draw Analysis**
10. Compare current draw at 3500 RPM vs full RPM
11. Measure battery voltage drop

**Success Criteria**:
- ☐ RPM caps at 3500 when trigger not held
- ☐ RPM uncaps immediately on trigger pull
- ☐ Spin-up time < 0.75 seconds
- ☐ All three motors track together during spin-up

**Data to Log**:
```
Distance | Pre-Spin RPM | Target RPM | Capped?
---------|--------------|------------|--------
2m       | _____        | _____      | Y/N
3m       | _____        | _____      | Y/N
4m       | _____        | _____      | Y/N

Spin-up time (3500→4000 RPM): _____ s
Battery voltage drop: _____ V
Current draw at 3500 RPM: _____ A
Current savings: _____ A
```

**Decision Point**:
- ✅ Spin-up < 0.75s & savings > 10A → Keep cap at 3500
- ⚠️ Spin-up 0.75-1.0s → Raise cap to 3800 RPM
- ❌ Spin-up > 1.0s → Disable cap entirely

---

## Block 2: Autonomous Development (1000-1200) - 120 min
**Goal**: 3 functional auto routines based on starting position

### Test 2.1: Position Detection Logic (30 min)
**Why**: Auto must know where it starts (Red 1, 2, or 3)

**Test Procedure**:
1. ☐ Place robot at Red 1 (source side) → Enable auto → Verify console output
2. ☐ Place robot at Red 2 (center) → Enable auto → Verify console output
3. ☐ Place robot at Red 3 (amp side) → Enable auto → Verify console output
4. ☐ Change driver station to Blue alliance
5. ☐ Repeat steps 1-3, verify detects Blue positions

**Success Criteria**:
- ☐ Correctly identifies all 6 positions

---

### Test 2.2: Simple Auto Sequences (90 min)

#### Position 2 (Center) - 30 min [HIGHEST PRIORITY]

**Sequence**:
1. Shoot preloaded game piece
2. Drive forward 2m to collect game piece
3. Drive back, shoot again

**Test**:
- ☐ Run 3 times, log success rate
- ☐ Completes in < 8 seconds

---

#### Position 1 (Source Side) - 20 min

**Sequence**:
1. Shoot preloaded game piece
2. Drive forward 1m
3. Stop

**Test**:
- ☐ Run 2 times
- ☐ Completes in < 3 seconds

---

#### Position 3 (Amp Side) - 40 min [TIME PERMITTING]

**Sequence**:
1. Shoot preloaded game piece
2. Drive to amp-side game piece
3. Return and shoot

**Test**:
- ☐ Run 2 times if time allows

---

## Block 3: Driver Training (1200-1315) - 75 min
**Goal**: Driver comfortable with controls

### Training 3.1: Basic Controls (20 min)

**Exercises**:
1. ☐ Drive 4m x 4m square (no rotation)
2. ☐ Drive forward while rotating 360°
3. ☐ Test alliance flip (Red vs Blue)
4. ☐ Practice speed modes

---

### Training 3.2: Auto-Aim Practice (30 min)

**Drills**:
1. ☐ Stationary shooting (2m, 3m, 4m)
2. ☐ Shoot while moving (strafe + shoot)
3. ☐ Quick-shot cycle (5 reps)

---

### Training 3.3: Match Simulation (25 min)

**Simulation**:
- ☐ 2:30 timer
- ☐ Score as many as possible
- ☐ Log battery voltage, success rate

---

## Block 4: Mechanical & Finalization (1315-1530) - 135 min

### 4.1: Wheel Replacement (45 min)
- ☐ Replace worn wheels
- ☐ Check alignment

### 4.2: Systems Check (30 min)
- ☐ Drive 5m, measure odometry accuracy
- ☐ Run one auto sequence
- ☐ Score 3 shots

### 4.3: Competition Readiness (30 min)
- ☐ Battery charged (12.7V+)
- ☐ Code deployed
- ☐ Bumpers attached
- ☐ Inspection checklist ready

### 4.4: Pack Out (30 min)
- ☐ Tools packed
- ☐ Spare parts secured
- ☐ Robot in cart
- ☐ Nothing left in pit

---

## Go/No-Go Decision Points

### By 1200 (3 hours in):
- ☐ Can drive field-oriented?
- ☐ Can score with auto-aim?
- ☐ Vision pose reset working?

### By 1400 (5 hours in):
- ☐ At least 1 auto sequence working?
- ☐ Driver trained and confident?

### By 1530 (6.5 hours in):
- ☐ Robot mechanically sound?
- ☐ Ready for load-in

---

## Data Collection Requirements

### AdvantageScope Must Record:
1. **Shooter**: RPM, current, voltage
2. **Pose**: X, Y, rotation
3. **Auto-Aim**: Alignment error
4. **Battery**: Voltage throughout tests

### Export at End of Day:
- ☐ Save AdvantageScope log: `LoadIn_2026-03-27.wpilog`
- ☐ Screenshot dashboard
- ☐ Export CSV summary

---

## Post-Day Action Items

**Determine based on data**:

1. **Shooter Cap**: Keep / Adjust / Disable
2. **Auto-Aim PID**: Keep / Tune
3. **Shot Success Rate**: Update lookup table if needed
4. **Battery Performance**: Adjust power budget
5. **Driver Feedback**: Strategy refinements

---

**Test Plan Version**: 1.0
**Last Updated**: 2026-03-27
**Field Configuration**: Red alliance half-field

[Return to Home](index.md) | [Driver Controls](driver-controls.md) | [Shooter Testing](shooter-testing.md)
