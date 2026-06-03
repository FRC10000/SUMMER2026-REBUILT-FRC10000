# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

FRC Team 10000 robot code for the 2026 season. Built on WPILib 2026.2.1 with Java 17, using the Command-Based framework and GradleRIO build system.

## Commands

```bash
# Build
./gradlew build

# Deploy to RoboRIO (requires robot network connection)
./gradlew deploy

# Run desktop simulation with SimGUI
./gradlew sim

# Run tests
./gradlew test

# Run a single test class
./gradlew test --tests "frc.robot.ExampleTest"

# Clean build artifacts
./gradlew clean
```

On Windows use `gradlew.bat` or `.\gradlew` instead of `./gradlew`.

## Architecture

**Entry point flow:** `Main.main()` → `RobotBase.startRobot(Robot::new)` → `Robot` constructor creates `RobotContainer` → `Robot.robotPeriodic()` runs `CommandScheduler` at ~50 Hz.

**Key files:**
- [src/main/java/frc/robot/Robot.java](src/main/java/frc/robot/Robot.java) — Robot lifecycle (`robotPeriodic`, `autonomousInit`, `teleopInit`, etc.). Only touch this to add mode-transition logic.
- [src/main/java/frc/robot/RobotContainer.java](src/main/java/frc/robot/RobotContainer.java) — Subsystem instantiation, controller bindings, and autonomous command selection. This is where most wiring lives.
- [src/main/java/frc/robot/Main.java](src/main/java/frc/robot/Main.java) — Entry point only; do not modify.

**Pattern:** New subsystems go in `src/main/java/frc/robot/subsystems/`, commands in `src/main/java/frc/robot/commands/`. Subsystems extend `SubsystemBase`; commands extend `Command` or use inline `Commands.*` factory methods. Subsystems are instantiated once in `RobotContainer` and passed to commands as constructor arguments.

**Deploy directory:** Files placed in `src/main/deploy/` are copied to `/home/lvuser/deploy/` on the RoboRIO at deploy time. Use this for configuration JSON files (e.g., YAGSL swerve configs).

## Vendor Libraries

| Library | Version | Purpose |
|---|---|---|
| YAGSL | 2026.4.1 | Swerve drive abstraction — primary drivetrain library |
| Phoenix 6 | 26.3.0 | CTRE TalonFX / Falcon 500 motor controllers |
| Phoenix 5 | 5.36.0 | Legacy CTRE TalonSRX / VictorSPX controllers |
| REVLib | 2026.0.5 | REV SPARK MAX/FLEX motor controllers (NEO motors) |
| ReduxLib | 2026.1.2 | Redux IMU/sensor support (YAGSL dependency) |
| maple-sim | 0.4.0-beta | Physics simulation for mechanisms |

**YAGSL swerve:** Configuration JSON files are loaded from `src/main/deploy/swerve/`. The swerve subsystem is instantiated with a path to that directory. See YAGSL documentation for the expected JSON schema.
