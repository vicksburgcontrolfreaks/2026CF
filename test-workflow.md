---
layout: default
title: Test Workflow
---

# Dynamic Shooting Test Workflow

**Branch**: `dynamic-shooting-test`
**Status**: ⚠️ UNPROVEN - Testing Required Before Merge to Main

## Overview

This branch contains two new experimental features:
1. **Velocity Compensation** - Shoot accurately while moving (evasive maneuvers)
2. **Trajectory Testing System** - Systematic testing of hardware angles and RPM

## What's New

### Velocity Compensation System
Compensates for robot motion when shooting while moving. Calculates where the robot will be when the ball reaches the target and adjusts aim angle accordingly.

**Files Modified:**
- `ShooterConstants.java` - Added compensation parameters
- `ShooterSubsystem.java` - Added compensation calculation methods
- `ShooterWithAutoAimCommand.java` - Applies compensation to auto-aim
- `RobotContainer.java` - Updated ShooterSubsystem constructor

### Trajectory Testing System
Enables systematic testing of shooter trajectory angles (hardware adjustments) and RPM combinations.

**Files Added:**
- `TrajectoryTestConstants.java` - Test parameters
- `TrajectoryTestCommand.java` - Test command (not currently bound)
- `TRAJECTORY_TEST_PROCEDURE.md` - Detailed test procedure

**Files Modified:**
- `ShooterSubsystem.java` - Added test mode with NetworkTables control

## Pre-Testing Checklist

Before deploying to robot:

- [ ] Verify branch is `dynamic-shooting-test`
- [ ] Build succeeds (`gradlew.bat build`)
- [ ] Review code changes for obvious issues
- [ ] Ensure practice field is available for testing
- [ ] Have data recording materials ready (laptop, clipboard, camera)
- [ ] Backup current working main branch code

## Testing Workflow

### Phase 1: Basic Functionality (30 minutes)

**Objective**: Verify new code doesn't break existing functionality

1. **Deploy Code**
   - [ ] Deploy `dynamic-shooting-test` branch to practice robot
   - [ ] Enable robot, verify no errors in Driver Station

2. **Test Baseline Shooting (Velocity Compensation DISABLED)**
   - [ ] Open Shuffleboard → `Shooter/Config`
   - [ ] Set `Velocity Compensation Enabled = false`
   - [ ] Test stationary auto-aim shooting (right trigger)
   - [ ] Test stationary manual shooting (left trigger)
   - [ ] Verify all existing controls work normally
   - [ ] **STOP IF BASELINE FAILS** - Debug before proceeding

3. **Test Trajectory Test Mode (Test Mode DISABLED)**
   - [ ] Open Shuffleboard → `Shooter/TrajectoryTest`
   - [ ] Verify `Test Mode Enabled = false` (default)
   - [ ] Test manual shooting with left trigger
   - [ ] Verify it uses normal distance-based RPM
   - [ ] **STOP IF FAILS** - Debug before proceeding

### Phase 2: Velocity Compensation Testing (45 minutes)

**Objective**: Test shooting while moving with compensation enabled

**Setup:**
- [ ] Enable velocity compensation in Shuffleboard (`Shooter/Config/Velocity Compensation Enabled = true`)
- [ ] Position robot at 2.69m from hub (known working distance)
- [ ] Load 3 balls

**Test 1: Stationary Baseline (Control)**
- [ ] Robot stationary, use auto-aim (right trigger)
- [ ] Record: \_\_/3 balls scored
- [ ] **Expected**: Same as Phase 1 results (compensation = 0 when not moving)

**Test 2: Slow Lateral Movement (~1 m/s)**
- [ ] Drive robot sideways at ~50% speed while holding right trigger
- [ ] Record: \_\_/3 balls scored
- [ ] Note: Did aim lead the target? Too much/too little?

**Test 3: Fast Lateral Movement (~2 m/s)**
- [ ] Drive robot sideways at ~75% speed while holding right trigger
- [ ] Record: \_\_/3 balls scored
- [ ] Note: Compensation amount (check if aiming "ahead" of target)

**Test 4: Diagonal Movement**
- [ ] Drive robot diagonally (forward + sideways) while shooting
- [ ] Record: \_\_/3 balls scored
- [ ] Note: Any unexpected behavior?

**Tuning (if needed):**
- If over-compensating (missing "ahead" of target):
  - [ ] Reduce `Angle Compensation Factor` from 1.0 to 0.8
  - [ ] Retest
- If under-compensating (missing "behind" target):
  - [ ] Increase `Angle Compensation Factor` from 1.0 to 1.2
  - [ ] Retest

**Success Criteria:**
- ✅ Moving shots score at least 50% success rate
- ✅ Aim visibly leads the target when moving
- ✅ No jittering or unstable behavior
- ✅ Driver feels confident using it

### Phase 3: Trajectory Testing System (1-2 hours)

**Objective**: Test hardware angle and RPM combinations

**Setup:**
- [ ] Disable velocity compensation (`Velocity Compensation Enabled = false`)
- [ ] Enable test mode in Shuffleboard (`Shooter/TrajectoryTest/Test Mode Enabled = true`)
- [ ] Set hardware trajectory angle to baseline (22°)
- [ ] Set `Trajectory Angle = 22` in Shuffleboard (for records)

**Quick Validation Test:**
- [ ] Set `Test RPM = 3500`
- [ ] Position robot at 2.69m
- [ ] Load 3 balls
- [ ] Hold left trigger to shoot
- [ ] Record: \_\_/3 balls scored
- [ ] **Expected**: Similar to normal shooting (test mode just overrides RPM calculation)

**Full Trajectory Testing:**
- [ ] Follow procedure in `TRAJECTORY_TEST_PROCEDURE.md`
- [ ] Test multiple angles (20°, 22°, 24°, 26°)
- [ ] Test multiple distances at each angle
- [ ] Record all data in provided templates
- [ ] Find optimal angle/RPM combinations

**Success Criteria:**
- ✅ Test mode successfully overrides RPM
- ✅ Can adjust RPM via Shuffleboard without redeploying
- ✅ Shooter responds to RPM changes within 2 seconds
- ✅ Found extended distance settings (>3.3m)

## Post-Testing Analysis

### Data Review
- [ ] Compile test results into summary
- [ ] Identify optimal velocity compensation factor
- [ ] Identify optimal trajectory angle and RPM table
- [ ] Note any issues or unexpected behavior

### Code Updates (if needed)
- [ ] Update `ShooterConstants.java` with optimal values
- [ ] Update `kAngleCompensationFactor` if tuned
- [ ] Update `kShooterVelocityTable` with new distance/RPM pairs
- [ ] Commit changes with test results summary

### Validation Test
- [ ] Deploy updated code
- [ ] Retest critical scenarios
- [ ] Verify improvements work consistently
- [ ] Get driver feedback

## Merge Decision Checklist

### Ready to Merge to Main When:
- [ ] All Phase 1 tests passed (baseline functionality intact)
- [ ] Velocity compensation tested and tuned (if using)
- [ ] Trajectory testing completed with optimal settings found
- [ ] No crashes, freezes, or critical bugs observed
- [ ] Code changes reviewed by at least one other programmer
- [ ] Driver tested and approved for match use
- [ ] Updated constants committed with test data justification
- [ ] Practice matches completed successfully with new code

### After Merge to Main:
1. [ ] Update `CHANGELOG.md` with test results and final settings
2. [ ] Update system documentation files:
   - [ ] `SHOOTER_SYSTEM_DOCUMENTATION.md` - Add velocity compensation section
   - [ ] `SHOOT_WHILE_DRIVING.md` - Document moving shot capabilities
3. [ ] Run `.\sync-docs-to-pages.bat` to publish to GitHub Pages
4. [ ] Notify team of new capabilities and how to use them
5. [ ] Schedule driver training session

## Rollback Plan

### If Testing Reveals Issues:
1. Note issues in branch (don't merge)
2. Create new branch for fixes if needed
3. Continue testing on practice robot only
4. Keep main branch as stable fallback

### If Issues Found After Merge:
1. Document issue in GitHub issue tracker
2. Revert merge if critical
3. Create hotfix branch
4. Test fix on practice robot
5. Fast-track merge when proven

## NetworkTables Reference

### Velocity Compensation Controls
**Location**: `Shooter/Config`

| Parameter | Default | Purpose |
|-----------|---------|---------|
| Velocity Compensation Enabled | true | Master enable/disable |
| Angle Compensation Factor | 1.0 | Scale compensation (0.0-1.5) |
| RPM Velocity Compensation Factor | 0.0 | RPM adjustment per m/s (unused currently) |
| Average Shot Velocity | 10.0 | Ball speed estimate for flight time |

### Trajectory Test Controls
**Location**: `Shooter/TrajectoryTest`

| Parameter | Default | Purpose |
|-----------|---------|---------|
| Test Mode Enabled | false | Override distance-based RPM |
| Test RPM | 3500 | Fixed RPM when test mode enabled |
| Trajectory Angle (degrees) | 22.0 | Current hardware angle (for records) |

## Safety Notes

- Test on practice robot only until proven
- Start with lower speeds/distances and increase gradually
- Clear area around target zone
- Stop immediately if unexpected behavior occurs
- Have driver station e-stop ready
- Keep main branch as emergency fallback

## Contact

Questions or issues during testing? Contact programming team lead.

---

**Created**: 2026-03-31
**Branch**: `dynamic-shooting-test`
**Status**: Ready for Practice Robot Testing
