---
layout: default
title: Driver Controls
---

# Driver Controls Guide

## Controller Overview

The robot uses **two Xbox controllers**:
- **Driver Controller** (Port 0): Drives the robot, triggers auto-aim
- **Mechanism Controller** (Port 1): Operates shooter, hopper, and collector

---

## Driver Controller (Port 0)

### Driving

| Control | Function | Notes |
|---------|----------|-------|
| **Left Stick Y** | Forward/Backward | Alliance-relative (forward = toward opponent) |
| **Left Stick X** | Strafe Left/Right | Alliance-relative |
| **Right Stick X** | Rotate Left/Right | Always rotates robot orientation |

### Speed Control

| Button | Speed Mode | Max Speed |
|--------|------------|-----------|
| **Right Bumper** | Turbo Mode | 100% (fastest) |
| **Left Bumper** | Precision Mode | 30% (slow, accurate) |
| *No Bumpers* | Normal Mode | 60% (default) |

### Special Functions

| Button | Function | Description |
|--------|----------|-------------|
| **Start Button** | Zero Gyro | Resets robot heading to 0° (face current direction) |
| **Y Button** | Rotate to Speaker | Auto-rotates to face speaker (alliance-aware) |

### Field-Oriented Drive

The robot uses **alliance-relative field-oriented drive**:
- **Blue Alliance**: Push forward → moves toward red side (opponent)
- **Red Alliance**: Push forward → moves toward blue side (opponent)
- **Strafing**: Left/right is relative to the field, not robot orientation
- **Rotation**: Always rotates the robot itself

**Example**: If you push the stick forward and right while rotated 180°:
- Robot moves forward-right *relative to the field*
- Robot orientation can be any direction
- This makes driving intuitive regardless of how the robot is facing

---

## Mechanism Controller (Port 1)

### Shooter & Auto-Aim

| Control | Function | Description |
|---------|----------|-------------|
| **Right Trigger** | Auto-Aim & Shoot | Spins up shooter, auto-aims at speaker, feeds when ready |

**Auto-Aim Behavior**:
1. **Shooter spins up** to target RPM (based on distance)
2. **Robot auto-rotates** to face speaker (back of robot toward target)
3. **Driver maintains translation control** - you can still move while aiming!
4. **Indexer feeds** automatically when:
   - Shooter at target RPM
   - Robot aligned within 3° tolerance
5. **Release trigger** to stop shooting

**Note**: The shooter is on the **back** of the robot. Auto-aim rotates so the back faces the speaker.

### Collector & Indexer

| Button | Function | Description |
|--------|----------|-------------|
| **A Button** | Collect | Runs collector + floor + indexer forward (collection mode) |
| **X Button** | Reverse | Runs collector + floor + indexer in reverse (unjam/eject) |
| **B Button** | Stop Feeding | Stops collector, floor, indexer (shooter keeps spinning) |

### Hopper

| Control | Function | Description |
|---------|----------|-------------|
| **POV Up** | Retract Hopper | Pulls hopper up (stowed position) |
| **POV Down** | Extend Hopper | Pushes hopper down (deployed position) |

---

## Operating Procedures

### Starting the Robot

1. **Place robot on field** facing desired direction (if using vision)
2. **Enable robot** (driver station)
3. **Wait for vision lock** (pose updates on dashboard)
4. **Zero gyro** if needed (Start button) - only if you want current direction to be "forward"

**Note**: When disabled, the robot continuously resets its pose based on AprilTag vision. You can move the robot around while disabled and it will track the new position!

### Normal Driving

1. **Select speed mode** (bumpers or none)
2. **Drive with left stick** (translation)
3. **Rotate with right stick** (orientation)
4. Robot drives **alliance-relative** - forward always means toward opponent

### Shooting Sequence

1. **Position robot** roughly facing speaker (not required, but faster)
2. **Hold Right Trigger** (mechanism controller)
3. **Robot auto-aims** - you see/feel rotation happening
4. **Keep driving if needed** - translation still works while auto-aiming!
5. **Wait for feed** - shooter spins up, then indexer feeds automatically
6. **Release trigger** when done shooting

### Emergency Stop

- **B Button** (mechanism controller): Stops collector, floor, and indexer (shooter keeps spinning for quick re-shot)
- **Disable robot** (driver station): Full stop of everything

---

## Tips & Tricks

### Driving Tips

- **Use Normal Mode** (60%) for most driving - good balance of speed and control
- **Use Precision Mode** (30%) for:
  - Tight maneuvering near game pieces
  - Precise positioning for auto-aim
  - Navigating tight spaces
- **Use Turbo Mode** (100%) for:
  - Fast repositioning across field
  - Defensive maneuvers
  - Racing to game pieces

### Auto-Aim Tips

- **Pre-position roughly** - auto-aim works from any angle, but is faster if you're already close
- **Drive while shooting** - you can strafe or move forward/backward while the robot maintains alignment
- **Trust the system** - let the robot handle rotation, you focus on positioning
- **Wait for alignment** - indexer won't feed until robot is aligned and shooter is at RPM

### Vision-Based Positioning

- **Robot knows where it is** - pose estimation from 4 cameras + odometry
- **Move while disabled** - robot tracks new position automatically
- **Field-oriented works everywhere** - consistent controls across entire field
- **Auto-aim is alliance-aware** - always aims at your alliance's speaker

---

## Troubleshooting

### "Robot not responding to controls"
- Check controller connection (green light)
- Verify robot is enabled
- Check speed mode (you might be in precision mode)

### "Auto-aim spinning wrong direction"
- Check alliance selection in driver station
- Verify vision cameras are working (dashboard shows tags detected)
- Robot should point **back** toward speaker (shooter on back)

### "Field-oriented not working"
- Check vision system is detecting AprilTags
- Try zeroing gyro (Start button)
- Verify alliance is set correctly in driver station

### "Robot driving backwards"
- Expected if alliance is set incorrectly
- Forward should always go toward opponent's side
- Check alliance selection in driver station

### "Shooter not feeding"
- Shooter must reach target RPM first (check dashboard)
- Robot must be aligned within 3° (check alignment indicator)
- Balls must be loaded in hopper

---

## Dashboard Indicators

Watch these on your dashboard while driving:

### Drive Telemetry
- **Current X/Y Position**: Robot location on field
- **Current Heading**: Robot rotation (degrees)
- **Alliance**: Blue or Red

### Vision Telemetry
- **Tags Detected**: Number of AprilTags seen
- **Active Cameras**: How many cameras have targets
- **Pose Confidence**: Quality of position estimate

### Shooter Telemetry
- **Target RPM**: Commanded shooter speed
- **Actual RPM**: Current shooter speed
- **Alignment Error**: Degrees off from target
- **Ready to Feed**: Green when aligned and at RPM

---

**Last Updated**: 2026-03-27
**For Questions**: Contact programming team
