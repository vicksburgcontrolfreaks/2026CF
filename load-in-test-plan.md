---
layout: default
title: Load-In Test Plan
---
# Load-In Day Test Plan - March 27, 2026

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

## TEST PLAN

### Block 1: Critical Systems Validation (0900-1000) - 60 min
**Goal**: Confirm robot is competition-ready for basic operation

---

#### Test 1.1: Field-Oriented Drive & Vision Pose Reset (20 min)
**Why**: Core driving functionality, new vision reset feature must work cross-alliance

**Setup**:
- [ ] Place robot at Red 2 starting position (center)
- [ ] Driver station set to **Red alliance**
- [ ] AdvantageScope recording: `Pose/X`, `Pose/Y`, `Pose/Rotation`
- [ ] Have tape measure ready

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
- [ ] Pose updates instantly while disabled (< 1 second lag)
- [ ] Forward = toward opponent on Red alliance
- [ ] Forward direction flips when switching to Blue alliance
- [ ] Strafing matches field orientation

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

#### Test 1.2: Auto-Aim Functionality (20 min)
**Why**: Primary scoring mechanism, must work reliably

**Setup**:
- [ ] Position robot 2.5m from speaker (measure with tape)
- [ ] AdvantageScope recording: `Shooter/Target RPM`, `Alignment Error`, `Vision/Distance To Speaker`
- [ ] Load 5 game pieces in hopper
- [ ] Driver station: Red alliance

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
- [ ] Robot rotates to face speaker (back toward target)
- [ ] Alignment within 3° tolerance
- [ ] Shooter spins up before feeding
- [ ] Feeding starts automatically when aligned + at RPM
- [ ] Driver retains translation control during auto-aim
- [ ] No jitter when aligned

**Data to Log**:
```
Test 1 (0°):  Time to first shot: _____ s | Success: Y/N
Test 2 (45°): Time to first shot: _____ s | Success: Y/N
Test 3 (90°): Time to first shot: _____ s | Success: Y/N
Test 4 (180°): Time to first shot: _____ s | Success: Y/N
Alignment settling time: _____ s
```

**Decision Point**:
- ✅ Smooth and accurate → Continue
- ⚠️ Jittery → Tune PID on-the-fly (NetworkTables)
- ❌ Unreliable → Add manual shooting mode (fixed RPM)

---

#### Test 1.3: Pre-Spin RPM Cap Validation (20 min)
**Why**: New feature needs validation before competition

**Setup**:
- [ ] AdvantageScope recording:
  - `Shooter/Target RPM`
  - `Shooter/Motor 1 Velocity`
  - `Shooter/Motor 2 Velocity`
  - `Shooter/Motor 3 Velocity`
  - `Battery Voltage`
  - `Shooter/Motor 1 Current`
- [ ] Mark distances on field: 2m, 3m, 4m from speaker

**Test Procedure**:

**Part A: Pre-Spin Cap Behavior**
1. Enable robot (shooter active, trigger NOT held)
2. Drive to 2m from speaker
   - Observe Target RPM: _____ (should be ~2950, no cap applied)
   - Observe Actual RPM: _____
3. Drive to 3m from speaker
   - Observe Target RPM: _____ (should cap at 3500)
   - Observe Actual RPM: _____
4. Drive to 4m from speaker
   - Observe Target RPM: _____ (should cap at 3500)
   - Observe Actual RPM: _____

**Part B: Trigger Pull Spin-Up**
5. Position at 4m from speaker
6. Observe current RPM: _____ (should be 3500)
7. **Pull trigger** and start timer
8. Observe RPM ramp: 3500 → _____ (target ~4000)
9. Stop timer when feeding starts
10. Record spin-up time: _____ seconds

**Part C: Current Draw Analysis**
11. Position at 3m, trigger NOT held (capped at 3500)
    - Battery voltage: _____
    - Motor current: _____
12. Pull trigger, observe uncapped RPM (~3450)
    - Battery voltage: _____
    - Motor current: _____
13. Calculate current difference: _____ amps

**Success Criteria**:
- [ ] RPM caps at 3500 when trigger not held
- [ ] RPM uncaps immediately on trigger pull
- [ ] Spin-up time < 0.75 seconds (acceptable delay)
- [ ] All three motors track together during spin-up

**Data to Log**:
```
Distance | Pre-Spin RPM | Target RPM | Capped?
---------|--------------|------------|--------
2m       | _____        | _____      | Y/N
3m       | _____        | _____      | Y/N
4m       | _____        | _____      | Y/N

Spin-up Performance:
- Time (3500→4000 RPM): _____ s
- Battery voltage drop: _____ V
- Current draw at 3500 RPM: _____ A
- Current draw at 4000 RPM: _____ A
- Current savings: _____ A
```

**Decision Point**:
- ✅ Spin-up < 0.75s & current savings > 10A → **Keep cap at 3500**
- ⚠️ Spin-up 0.75-1.0s → **Raise cap to 3800 RPM**
- ❌ Spin-up > 1.0s → **Disable cap entirely** (too slow)
- 💡 Current savings < 5A → **Lower cap to 3000 RPM** or disable

---

### Block 2: Autonomous Development (1000-1200) - 120 min
**Goal**: 3 functional auto routines based on starting position

---

#### Test 2.1: Position Detection Logic (30 min)
**Why**: Auto must know where it starts (Red 1, 2, or 3)

**Implementation Required**:
Create `getAutonomousCommand()` logic in RobotContainer:

```java
public Command getAutonomousCommand() {
  Pose2d startPose = m_swerveDrive.getPose();

  // Determine alliance
  Alliance alliance = startPose.getX() > 8.27 ? Alliance.Red : Alliance.Blue;

  // Determine position based on Y coordinate
  int position;
  if (startPose.getY() < 2.0) {
    position = 1;  // Source side
  } else if (startPose.getY() > 6.0) {
    position = 3;  // Amp side
  } else {
    position = 2;  // Center
  }

  System.out.println("Auto: " + alliance + " Position " + position);

  // Return appropriate command
  switch (position) {
    case 1: return getPosition1Auto(alliance);
    case 2: return getPosition2Auto(alliance);
    case 3: return getPosition3Auto(alliance);
    default: return getPosition2Auto(alliance);
  }
}
```

**Test Procedure**:
1. Place robot at Red 1 (source side)
   - Enable auto
   - Verify console: "Auto: Red Position 1"
2. Place robot at Red 2 (center)
   - Enable auto
   - Verify console: "Auto: Red Position 2"
3. Place robot at Red 3 (amp side)
   - Enable auto
   - Verify console: "Auto: Red Position 3"
4. **Change driver station to Blue alliance**
5. Repeat steps 1-3, verify detects "Blue Position X"

**Success Criteria**:
- [ ] Correctly identifies all 6 positions (Red 1,2,3 and Blue 1,2,3)
- [ ] Console output matches physical position
- [ ] Works with both alliance selections

**Data to Log**:
```
Position | Actual X | Actual Y | Detected | Correct?
---------|----------|----------|----------|----------
Red 1    | _____    | _____    | _____    | Y/N
Red 2    | _____    | _____    | _____    | Y/N
Red 3    | _____    | _____    | _____    | Y/N
Blue 1   | _____    | _____    | _____    | Y/N
Blue 2   | _____    | _____    | _____    | Y/N
Blue 3   | _____    | _____    | _____    | Y/N
```

**Decision Point**: If Y thresholds are wrong, adjust based on actual poses

---

#### Test 2.2: Simple Auto Sequences (90 min)
**Why**: Need scoring in auto for competition

**Sequences to Implement** (in order of priority):

##### Position 2 (Center) - 30 min [HIGHEST PRIORITY]
**Why**: Most common starting position

**Sequence**:
1. Shoot preloaded game piece (auto-aim)
2. Drive forward 2m to collect center game piece
3. Drive back to starting position
4. Shoot second game piece

**Test Procedure**:
- [ ] Place robot at Red 2, enable auto
- [ ] Observe sequence execution
- [ ] Record: Shot 1 success Y/N, Collected Y/N, Shot 2 success Y/N
- [ ] Repeat 3 times for consistency

**Success Criteria**:
- [ ] Shoots preload successfully
- [ ] Drives to game piece location (±0.3m)
- [ ] Returns to shooting position
- [ ] Shoots second piece
- [ ] Completes in < 8 seconds

**Data to Log**:
```
Run | Shot 1 | Collected | Shot 2 | Time | Notes
----|--------|-----------|--------|------|------
1   | Y/N    | Y/N       | Y/N    | ___s |
2   | Y/N    | Y/N       | Y/N    | ___s |
3   | Y/N    | Y/N       | Y/N    | ___s |
```

---

##### Position 1 (Source Side) - 20 min [MEDIUM PRIORITY]
**Why**: Simplest sequence, good fallback

**Sequence**:
1. Shoot preloaded game piece
2. Drive forward 1m
3. Stop

**Test Procedure**:
- [ ] Place robot at Red 1, enable auto
- [ ] Observe sequence
- [ ] Repeat 2 times

**Success Criteria**:
- [ ] Shoots successfully
- [ ] Moves out of starting zone
- [ ] Completes in < 3 seconds

---

##### Position 3 (Amp Side) - 40 min [LOWER PRIORITY]
**Why**: More complex path, time permitting

**Sequence**:
1. Shoot preloaded game piece
2. Drive toward amp-side game piece (curved path)
3. Collect game piece
4. Return to shooting position
5. Shoot second game piece

**Test Procedure**:
- [ ] Place robot at Red 3, enable auto
- [ ] Observe path and collection
- [ ] Repeat 2 times if time allows

**Success Criteria**:
- [ ] Completes without collisions
- [ ] Collects game piece
- [ ] Scores both pieces

---

**CONTINGENCY**: If running short on time, implement only Position 1 and 2

---

### Block 3: Driver Training (1200-1315) - 75 min
**Goal**: Driver comfortable with controls, understands new features

**Note**: Lunch during this block (drivers rotate eating)

---

#### Training 3.1: Basic Controls Review (20 min)
**Why**: New alliance-relative driving

**Setup**:
- [ ] Mark 4m x 4m square on floor with tape
- [ ] Driver station: Red alliance
- [ ] Driver has controller reference sheet

**Exercises**:

**Exercise 1: Square Pattern** (5 min)
- Driver drives 4m x 4m square using only translation (no rotation)
- Goal: Test field-oriented strafing

**Exercise 2: Rotation Independence** (5 min)
- Driver drives forward while rotating 360°
- Should move in straight line regardless of robot orientation

**Exercise 3: Alliance Flip Test** (5 min)
- Drive forward on Red alliance (toward blue)
- Switch driver station to Blue alliance
- Drive forward again (should now move toward red)
- Confirm driver understands "forward = opponent"

**Exercise 4: Speed Modes** (5 min)
- Practice with all three modes:
  - Right Bumper: Turbo (100%)
  - No Bumpers: Normal (60%)
  - Left Bumper: Precision (30%)

**Success Criteria**:
- [ ] Can drive square without rotating
- [ ] Understands field-oriented behavior
- [ ] Comfortable with speed modes
- [ ] Understands alliance-relative controls

**Driver Feedback**:
```
Clarity of controls (1-10): _____
Comfortable with field-oriented: Y/N
Preferred speed mode: _____
Issues/Confusion: _____
```

---

#### Training 3.2: Auto-Aim Practice (30 min)
**Why**: Primary scoring method, driver must be confident

**Setup**:
- [ ] Mark positions: 2m, 3m, 4m from speaker
- [ ] Load 15 game pieces (5 per drill)

**Drill 1: Stationary Shooting** (10 min)
1. Position at 2m, score 3 pieces
2. Position at 3m, score 3 pieces
3. Position at 4m, score 3 pieces

**Drill 2: Shoot While Moving** (10 min)
1. Start at 3m, hold trigger
2. Strafe left/right while shooting
3. Move forward/back while shooting
4. Goal: Understand translation control during auto-aim

**Drill 3: Quick-Shot Cycle** (10 min)
1. Drive to scoring position
2. Shoot
3. Drive away
4. Repeat 5 times
5. Goal: Practice trigger timing and flow

**Success Criteria**:
- [ ] Consistently scores from 2m, 3m, 4m
- [ ] Can shoot while moving
- [ ] Understands when to hold vs release trigger
- [ ] Comfortable with auto-rotation

**Driver Feedback**:
```
Shot Success Rate:
- 2m: ___/3
- 3m: ___/3
- 4m: ___/3

Comfortable shooting while moving: Y/N
Trigger timing intuitive: Y/N
Issues: _____
```

---

#### Training 3.3: Match Simulation (25 min)
**Why**: Stress test under realistic conditions

**Setup**:
- [ ] Scatter 10 game pieces on half-field
- [ ] Set 2:30 timer (teleop length)
- [ ] AdvantageScope recording battery voltage

**Simulation Procedure**:
1. Start timer
2. Driver: Collect and score as many as possible
3. Use auto-aim for all shots
4. Practice speed transitions (precision → turbo)
5. Stop at 2:30

**Observe**:
- Driver stress level
- Decision making under pressure
- Battery voltage at end
- Robot consistency

**Success Criteria**:
- [ ] Scores > 5 pieces in 2:30
- [ ] Battery voltage > 11.5V at end
- [ ] Driver maintains composure
- [ ] No collisions or errors

**Data to Log**:
```
Game pieces scored: _____
Battery voltage start: _____
Battery voltage end: _____
Shot accuracy: ____%
Driver confidence (1-10): _____
Areas for improvement: _____
```

---

### Block 4: Mechanical & Finalization (1315-1530) - 135 min

---

#### 4.1: Wheel Replacement (45 min)
**Why**: Worn wheels affect odometry accuracy and traction

**Procedure**:
- [ ] Remove all 4 swerve modules
- [ ] Replace worn wheels (check tread depth)
- [ ] Reinstall modules
- [ ] Check wheel alignment

**While mechanics work**:
- [ ] Programming reviews auto code
- [ ] Fix any bugs found during testing
- [ ] Update documentation

---

#### 4.2: Post-Mechanical Systems Check (30 min)
**Why**: Ensure nothing broken during wheel replacement

**Quick Test Checklist**:
- [ ] Drive 5m forward, measure actual distance traveled
  - Expected: 5.0m ± 0.1m
  - Actual: _____ m
  - Odometry error: _____ %
- [ ] Run Position 2 auto sequence once
  - Success: Y/N
- [ ] Score 3 shots from 3m
  - Success rate: ___/3

**If odometry is off**:
- Check wheel diameter in constants
- Verify module offsets

---

#### 4.3: Final Competition Readiness (30 min)

**Robot Checklist**:
- [ ] Battery fully charged (12.7V+)
- [ ] Code deployed and verified
- [ ] Bumpers attached correctly
- [ ] All fasteners tight
- [ ] No loose wires
- [ ] Radio programmed correctly
- [ ] Robot passes inspection (weight, size, etc.)

**Driver Station Checklist**:
- [ ] Controller batteries fresh
- [ ] Laptop charged
- [ ] Driver station software updated
- [ ] Team number configured
- [ ] Alliance selection tested

**Software Checklist**:
- [ ] Latest code on robot
- [ ] Auto routines tested
- [ ] NetworkTables tuning values recorded
- [ ] Emergency stop tested

---

#### 4.4: Pack Out (30 min)
**Why**: Must be ready to load by 1600

**Pack Out Checklist**:

**Tools & Spares**:
- [ ] Tool box secured
- [ ] Spare wheels packed
- [ ] Spare batteries charged
- [ ] Extra bumpers
- [ ] Cable ties, tape, fasteners

**Robot Prep**:
- [ ] Battery removed (charging)
- [ ] Robot in cart
- [ ] Bumpers protected
- [ ] All components secured

**Paperwork**:
- [ ] Inspection checklist printed
- [ ] Team roster
- [ ] Safety forms
- [ ] Robot specifications

**Final Check**:
- [ ] Nothing left in pit
- [ ] All equipment accounted for
- [ ] Load-in schedule confirmed

---

## Go/No-Go Decision Points

### By 1200 (3 hours in):
- [ ] **Can drive field-oriented?**
  - ✅ Yes → Continue
  - ❌ No → Drop everything, fix immediately
- [ ] **Can score with auto-aim?**
  - ✅ Yes → Continue
  - ❌ No → Debug or implement manual mode
- [ ] **Vision pose reset working?**
  - ✅ Yes → Great, keep it
  - ⚠️ No → Acceptable, use gyro zero only

### By 1400 (5 hours in):
- [ ] **At least 1 auto sequence working?**
  - ✅ Yes → Implement more if time allows
  - ❌ No → Focus on one simple sequence (Position 1)
- [ ] **Driver trained and confident?**
  - ✅ Yes → Proceed to mechanical
  - ⚠️ No → Extend training, reduce pack time

### By 1530 (6.5 hours in):
- [ ] **Robot mechanically sound?**
  - ✅ Yes → Final pack out
  - ❌ No → Prioritize repairs over pack aesthetics

---

## Data Collection Requirements

### AdvantageScope Must Record:
1. **Shooter Performance**:
   - `Shooter/Target RPM`
   - `Shooter/Motor 1 Velocity`
   - `Shooter/Motor 2 Velocity`
   - `Shooter/Motor 3 Velocity`
   - `Shooter/Motor 1 Current`

2. **Battery/Power**:
   - `Battery Voltage`
   - Total current draw

3. **Pose/Vision**:
   - `Pose/X`
   - `Pose/Y`
   - `Pose/Rotation`
   - `Vision/Distance To Speaker`

4. **Auto-Aim**:
   - `Alignment Error`
   - Rotation command output

### Export at End of Day:
- [ ] Save AdvantageScope recording: `LoadIn_2026-03-27.wpilog`
- [ ] Export CSV summary of key metrics
- [ ] Screenshot dashboard for reference

---

## Post-Day Action Items

**Based on collected data, determine**:

1. **Shooter Cap Effectiveness**:
   - Current savings: _____ A
   - Spin-up delay: _____ s
   - **Decision**: Keep at 3500 / Adjust to _____ / Disable

2. **Auto-Aim Tuning**:
   - Average alignment time: _____ s
   - Jitter observed: Y/N
   - **Decision**: Keep current PID / Adjust to P=_____ D=_____

3. **Shot Success Rate by Distance**:
   - 2m: ____%
   - 3m: ____%
   - 4m: ____%
   - **Decision**: Update lookup table values if needed

4. **Battery Performance**:
   - Voltage at end of 2:30 run: _____ V
   - Concerning voltage sag: Y/N
   - **Decision**: Adjust power budget / Add warnings

5. **Driver Feedback**:
   - Most comfortable distance: _____ m
   - Preferred shooting strategy: _____
   - Concerns: _____

---

## Emergency Contacts

**Programming Lead**: _____
**Drive Coach**: _____
**Mechanical Lead**: _____
**Mentor**: _____

---

**Test Plan Version**: 1.0
**Last Updated**: 2026-03-27
**Field Configuration**: Red alliance half-field
**Competition**: Load-in day preparation
