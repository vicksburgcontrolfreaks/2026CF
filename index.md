---
layout: default
title: Home
---

# 2026 Control Freaks Robot Documentation

Welcome to the official documentation for the 2026 Control Freaks FRC robot!

## Quick Links

### Testing & Development
- [**🧪 Test Workflow**](test-workflow.md) - Phase-by-phase testing checklist for new features
- [**📐 Trajectory Testing**](trajectory-testing.md) - Hardware angle and RPM testing procedure

### For Drivers
- [**Driver Controls Guide**](driver-controls.md) - Controller button mappings and operating instructions

### For Programmers
- [**Shooter Test Usage**](shooter-testing.md) - Running shooter characterization tests
- [**PID Tuning Guide**](pid-tuning.md) - Tuning motor velocity and alignment PID
- [**Shooter System Documentation**](shooter-system.md) - Technical overview of the shooter subsystem
- [**Shoot While Driving**](shoot-while-driving.md) - Auto-aim system documentation

## Robot Features

### Swerve Drive
- **MAXSwerve** modules with NEO motors
- **Field-oriented** drive with alliance-relative controls
- **Vision-based** pose estimation with 4-camera AprilTag detection

### Vision System
- **4x Arducam OV9281** cameras via PhotonVision
- **360° AprilTag** coverage
- **Multi-camera pose fusion** for accurate localization
- **Instant pose reset** when disabled (100% trust vision)

### Shooter System
- **3x NEO Vortex** shooter motors with velocity PID
- **3x NEO 550** indexer motors
- **Auto-aim** with alliance-aware targeting
- **Dynamic RPM** based on distance to speaker
- **Real-time PID tuning** via NetworkTables

### Auto-Aim Features
- **Alliance-aware targeting** - automatically aims at correct speaker
- **Shoot while driving** - driver controls translation, robot maintains alignment
- **Distance-based velocity** - adjusts shooter RPM based on distance
- **Vision-guided rotation** - uses pose estimation for precise alignment

## System Architecture

```
Robot
├── Drive Subsystem (Swerve)
│   ├── Pose Estimation (odometry + vision fusion)
│   ├── Field-oriented drive
│   └── Alliance-relative controls
├── Vision Subsystem (PhotonVision)
│   ├── 4-camera AprilTag detection
│   ├── Multi-camera pose fusion
│   └── Vision pose reset (when disabled)
├── Shooter Subsystem
│   ├── Velocity PID control
│   ├── Real-time NetworkTables tuning
│   └── Distance-based RPM lookup
└── Commands
    ├── ShooterCommand (auto-aim)
    ├── DriveAndAlignCommand (shoot while moving)
    └── Test commands (characterization)
```

## Branch Information

- **main**: Stable production code
- **Shooter-Tuning**: Active development (shooter characterization, vision improvements)
- **gh-pages**: This documentation site

## Getting Help

- Check the documentation pages for detailed guides
- Review code comments in the repository
- Contact the programming team lead

---

**Last Updated**: 2026-03-31
**Team**: FRC 5647 Vicksburg Control Freaks
**Season**: 2026 Rebuilt Welded
