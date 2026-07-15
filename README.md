# FRC Team 10000 — 2026 Reefscape Robot Code

> Java 17 | WPILib 2026.2.1 | Command-Based Framework | GradleRIO

This repository contains the full robot code for **FRC Team 10000**'s 2026 Reefscape season robot. The robot features a YAGSL swerve drivetrain, a multi-stage shooter with vision-guided auto-aim, a retractable intake, and addressable LED status indicators.

---

## Table of Contents

- [Quick Start](#quick-start)
- [Robot Overview](#robot-overview)
- [Hardware Map](#hardware-map)
- [Software Architecture](#software-architecture)
  - [Entry Point Flow](#entry-point-flow)
  - [Project Structure](#project-structure)
- [Subsystems](#subsystems)
  - [SwerveSubsystem](#swervesubsystem)
  - [IntakeSubsystem](#intakesubsystem)
  - [TurretSubsystem](#turretsubsystem)
  - [PivotSubsystem](#pivotsubsystem)
  - [FlywheelSubsystem](#flywheelsubsystem)
  - [FeederSubsystem](#feedersubsystem)
  - [LEDSubsystem](#ledsubsystem)
- [Commands](#commands)
- [Constants](#constants)
- [Vision System](#vision-system)
- [Shooter Lookup Tables](#shooter-lookup-tables)
- [Controller Bindings](#controller-bindings)
- [Autonomous](#autonomous)
- [Vendor Dependencies](#vendor-dependencies)
- [Build & Deploy](#build--deploy)
- [Simulation](#simulation)
- [License](#license)

---

## Quick Start

**Prerequisites:**
- [WPILib 2026](https://wpilib.org) (includes Java 17, Gradle, and VS Code extension)
- RoboRIO 2 on the robot (or desktop simulation)

**Build the project:**
```bash
# Linux / macOS
./gradlew build

# Windows
.\gradlew.bat build
```

**Deploy to robot** (requires connection to the robot's network):
```bash
.\gradlew deploy
```

**Run desktop simulation:**
```bash
.\gradlew sim
```

---

## Robot Overview

| Attribute | Value |
|---|---|
| Team Number | 10000 |
| Season | 2026 — Reefscape |
| Drivetrain | 4-module swerve (Falcon 500 / TalonFX) |
| Max Speed | 14.5 ft/s (4.42 m/s) |
| Shooter | 4-motor flywheel + pivoting turret + feeder |
| Intake | Dual-deploy + roller, retractable |
| Vision | Limelight 3G (rear-facing, on turret) |
| IMU | Pigeon 2 (CAN ID 40, canivore bus) |
| Robot Mass | ~58.27 kg (148 lbs including bumpers) |

---

## Hardware Map

### CAN ID Assignment

| ID | Device | Bus | Subsystem |
|---|---|---|---|
| 0 | Swerve — Front Right Drive | canivore | Swerve |
| 1 | Swerve — Back Right Drive | canivore | Swerve |
| 2 | Swerve — Back Left Drive | canivore | Swerve |
| 3 | Swerve — Front Left Drive | canivore | Swerve |
| 4 | Flywheel — Front Right (master) | default | Flywheel |
| 5 | Flywheel — Back Right (follower, opposed) | default | Flywheel |
| 6 | Flywheel — Back Left (follower, aligned) | default | Flywheel |
| 7 | Flywheel — Front Left (follower, opposed) | default | Flywheel |
| 10 | Swerve — Front Right Angle | canivore | Swerve |
| 11 | Swerve — Back Right Angle | canivore | Swerve |
| 12 | Swerve — Back Left Angle | canivore | Swerve |
| 13 | Swerve — Front Left Angle | canivore | Swerve |
| 20 | Swerve — Front Right Encoder | canivore | Swerve |
| 21 | Swerve — Back Right Encoder | canivore | Swerve |
| 22 | Swerve — Back Left Encoder | canivore | Swerve |
| 23 | Swerve — Front Left Encoder | canivore | Swerve |
| 24 | Turret | canivore | Turret |
| 25 | Pivot (SPARK MAX, brushless NEO) | default | Pivot |
| 30 | Intake — Deploy Left | canivore | Intake |
| 31 | Intake — Deploy Right | canivore | Intake |
| 32 | Intake — Roller Master | canivore | Intake |
| 33 | Intake — Roller Follower | canivore | Intake |
| 34 | Feeder — Motor | canivore | Feeder |
| 35 | Feeder — Wheel Motor | canivore | Feeder |
| 36 | Feeder — Wheel Follower | canivore | Feeder |
| 40 | Pigeon 2 IMU | canivore | Swerve |

### Swerve Module Layout

| Module | Drive ID | Angle ID | Encoder ID | Encoder Offset | Drive Inverted |
|---|---|---|---|---|---|
| Front Left | 3 | 13 | 23 | 196.08° | No |
| Front Right | 0 | 10 | 20 | 20.13° | No |
| Back Left | 2 | 12 | 22 | 52.65° | No |
| Back Right | 1 | 11 | 21 | 78.05° | **Yes** |

---

## Software Architecture

### Entry Point Flow

```
Main.main()
  → RobotBase.startRobot(Robot::new)
    → Robot constructor creates RobotContainer
      → RobotContainer instantiates all subsystems, configures bindings
    → Robot.robotPeriodic() runs CommandScheduler at ~50 Hz
```

**Lifecycle methods in `Robot.java`:**

| Method | Behavior |
|---|---|
| `robotInit` | Creates `RobotContainer`, starts `DataLogManager` |
| `robotPeriodic` | Runs `CommandScheduler.getInstance().run()` at 50 Hz |
| `disabledInit` | Sets all motors to brake, resets shooter subsystems, starts 10s timer |
| `disabledPeriodic` | After 10s, releases motors from brake to save battery |
| `autonomousInit` | Gets autonomous command from chooser, schedules it |
| `teleopInit` | Cancels auto command, resets shooter subsystems |
| `testInit` | Cancels all commands |

### Project Structure

```
src/main/java/frc/robot/
├── Main.java                          # Entry point — do not modify
├── Robot.java                         # TimedRobot lifecycle
├── RobotContainer.java                # Subsystem wiring, controller bindings, auto chooser
├── Constants.java                     # All robot-wide constants
│
├── subsystems/
│   ├── swervedrive/
│   │   └── SwerveSubsystem.java       # YAGSL swerve drivetrain wrapper
│   ├── IntakeSubsystem.java           # Deploy + roller intake
│   ├── TurretSubsystem.java           # Rotating turret (TalonFX)
│   ├── PivotSubsystem.java            # Shooter pivot/elevation (SPARK MAX)
│   ├── FlywheelSubsystem.java         # 4-motor flywheel (TalonFX)
│   ├── FeederSubsystem.java           # Feeder + wheel (TalonFX)
│   └── LEDSubsystem.java              # Addressable LED strip
│
├── commands/
│   ├── PassShootCommand.java          # Turret counter-rotate + shoot while driving
│   ├── AutoAimAndShootCommand.java    # Full vision aim + shoot pipeline
│   ├── AutoAimCommand.java            # Vision turret + pivot alignment only
│   ├── AutoShootCommand.java          # Vision flywheel + feeder only
│   ├── IntakeRetractCommand.java      # Intake retract + reverse roller
│   └── swervedrive/auto/
│       └── AutoBalanceCommand.java    # PID pitch-based charge station balance
│
└── util/
    ├── VisionUtil.java                # Shared vision math + lookup interpolation
    ├── ShooterLookup.java             # InterpolatingDoubleTreeMap lookup tables
    └── LimelightHelpers.java          # Limelight v1.14 NetworkTables helpers
```

---

## Subsystems

### SwerveSubsystem

A wrapper around YAGSL's `SwerveDrive` providing field-oriented drive with odometry and PathPlanner integration.

| Property | Value |
|---|---|
| IMU | Pigeon 2 (CAN ID 40, canivore) |
| Max Speed | 14.5 ft/s (4.42 m/s) |
| Odometry Update | Every 2nd cycle (40 ms) |
| Drive Gear Ratio | 6.75:1 |
| Angle Gear Ratio | 21.43:1 |
| Wheel Diameter | 4 inches (101.6 mm) |
| Wheel COF | 1.19 |

**Drive PID:** kP=0.15, kI=0, kD=0
**Angle PID:** kP=50, kI=0, kD=0.32
**Heading PID:** kP=0.618, kI=0, kD=0

**Default command:** `driveFieldOriented()` — driver controls are always active.

**Key methods:** `zeroGyro()`, `lock()` (X-pattern), `getPitch()`, `setMotorBrake()`

**PathPlanner Integration:**
- AutoBuilder configured with `PPHolonomicDriveController` (P=5.0 translation, P=5.0 rotation)
- Robot dimensions: 0.628m wide × 0.477m long
- Module positions: FL(0.314, 0.238), FR(0.314, -0.238), BL(-0.314, 0.238), BR(-0.314, -0.238)

---

### IntakeSubsystem

Dual-deploy intake with independent roller motor. Deploy motors are position-controlled and synchronized.

| Property | Value |
|---|---|
| Deploy Motors | 2× TalonFX (ID 30, 31) — position controlled, synchronized |
| Roller Motors | 2× TalonFX (ID 32, 33) — master + opposed follower |
| Deploy Range | -1440° (4 full rotations, 4.714:1 gear ratio) |
| Deploy PID | kP=1.1, kD=0.2, brake mode, 35A stator limit |
| Intake Speed | -1.0 (full power) |
| Sync Tolerance | 5° (dual-deploy alignment) |

**Key commands:** `acquireFuelCommand(reverse)`, `retractIntake()`, `safeDeployCommand(degrees)`, `reverseIntakeCommand()`

---

### TurretSubsystem

Rotating turret with Motion Magic control and soft limits.

| Property | Value |
|---|---|
| Motor | 1× TalonFX (ID 24, canivore) |
| Gear Ratio | 400/14 = 28.571:1 |
| Soft Limits | ±0.25 rotations (±90°) |
| Motion Magic | cruise=1.0 rot/s, accel=2.0 rot/s², jerk=0 |
| PID | kP=75.0, kD=10.0, kS=0.25, kV=2.0 |
| Neutral Mode | Brake |

Reports current angle to SmartDashboard every 100 ms.

---

### PivotSubsystem

Shooter pivot/elevation using a REV SPARK MAX with a NEO brushless motor.

| Property | Value |
|---|---|
| Motor | 1× REV SPARK MAX (ID 25, brushless NEO) |
| Gear Ratio | 320:1 |
| Position Conversion | 360/320 degrees per motor rotation |
| PID | kP=0.04, kI=0, kD=0 |
| Soft Limits | 0°–60° |
| Current Limit | 20A smart current limit |
| Neutral Mode | Brake |

---

### FlywheelSubsystem

4-motor flywheel with VelocityVoltage control for consistent shot velocity.

| Property | Value |
|---|---|
| Motors | 4× TalonFX (default CAN bus, IDs 4–7) |
| Master | Front Right (ID 4) |
| Followers | ID 6 (aligned), IDs 5 & 7 (opposed) |
| PID | kP=0.1, kV=0.12 |
| Current Limit | 60A stator |
| Neutral Mode | Coast |

**Key methods:** `setTargetRPM(rpm)`, `getCurrentRPM()`, `isAtTargetRPM(target, tolerance)`, `stop()`

---

### FeederSubsystem

3-motor feeder system that stages balls from intake to flywheel. Uses a two-phase shoot sequence (wheel first, then feeder).

| Property | Value |
|---|---|
| Motors | 3× TalonFX (canivore, IDs 34–36) |
| Feeder Motor | ID 34 |
| Feeder Wheel | ID 35 (inverted CCW+) |
| Feeder Wheel Follower | ID 36 (opposed) |
| Current Limit | 30A stator (both) |
| Idle Speeds | Feeder: -0.15, Wheel: -0.05 (slow reverse to prevent jams) |
| Shoot Speeds | Feeder: 0.4, Wheel: 0.7 |

**Shoot sequence:** Wheel spins up first, feeder engages 0.5s later to prevent ball jams.

---

### LEDSubsystem

60-LED addressable strip on PWM channel 0. Changes color based on robot state.

| State | Color |
|---|---|
| Disabled | Red (brightness 50) |
| Autonomous | Cyan (0, 50, 50) |
| Teleop | Blue (0, 0, 50) |

Only updates LEDs on state change to avoid unnecessary overhead.

---

## Commands

### AutoAimAndShootCommand

Full vision-guided aim + shoot pipeline. Uses `limelight-back` (rear-facing, mounted on turret) to detect AprilTags and automatically align turret, pivot, flywheel, and feeder.

**Pipeline:**
1. Read raw fiducials every 3 cycles from limelight
2. Filter by alliance tag IDs (Red: 10, 11 | Blue: 26, 27)
3. Turret PID correction using `txnc` (kP=0.5, deadband 1.5°)
4. Compute horizontal distance from `tync` → lookup pivot angle and flywheel RPM
5. Staged feeder sequence: wheel first, feeder 0.3s later when flywheel at target RPM (±200 tolerance)

### PassShootCommand

Shoot while driving — turret counter-rotates to cancel the robot's heading so the shot travels straight.

**Logic:** `turret target = -(car heading)`. Refuses to shoot if `|heading| > 90°`.

**Phases:**
1. (0–0.3s) Turret moves to counter-rotate, flywheel starts at 2000 RPM
2. (0.3s+) Pivot goes to 30°, feeder engages when flywheel ready

**Constants:** PASS_RPM=2000, PASS_PIVOT_ANGLE=30°, PASS_DELAY=0.3s

### AutoAimCommand

Vision turret + pivot alignment only (no flywheel/feeder activation). Used as a building block.

### AutoShootCommand

Vision flywheel + feeder only (no turret/pivot control). Defaults to 1500 RPM if no target visible.

### IntakeRetractCommand

- **While held:** Retracts intake to 50% deploy, roller at -0.2 power
- **On release:** Stops roller, returns to full deploy

### AutoBalanceCommand

PID-based charge station balancing using SwerveSubsystem pitch.

| Parameter | Value |
|---|---|
| kP | 0.025 |
| Tolerance | 1.5° |
| Setpoint | 0° (level) |
| Max Drive | ±0.5 (clamped) |

Ends by locking swerve wheels in X-pattern.

---

## Constants

| Constant | Value |
|---|---|
| Robot Mass | 58.27 kg (148 lbs with bumpers) |
| Loop Time | 0.020 s (50 Hz) |
| Max Speed | 4.42 m/s (14.5 ft/s) |
| Wheel Lock Time | 10 s (brake hold when disabled) |
| Deadband | 0.2 |
| Drive Enabled | true |
| Vision Drive Enabled | false |

---

## Vision System

The robot uses a **Limelight 3G** camera (`limelight-back`) mounted on the turret, facing rearward toward the Reef.

### Camera Setup

| Parameter | Value |
|---|---|
| Camera Height | 0.54 m from floor |
| Camera Mount Angle | 29.3° upward |
| Target (AprilTag) Height | 1.14 m (Reefscape Reef tag center) |
| Height Difference | 0.60 m |

### Alliance Tag IDs

| Alliance | Tag IDs | Priority |
|---|---|---|
| Red | 10, 11 | 10 |
| Blue | 26, 27 | 26 |

### Vision Math

Horizontal distance is computed from `tync` (raw vertical offset) using trigonometry:

```
distance = heightDifference / tan(mountAngle + tync)
```

The `VisionUtil.findNearestTarget()` method filters fiducials by alliance and returns the nearest target by distance.

---

## Shooter Lookup Tables

Distance-based lookup tables using `InterpolatingDoubleTreeMap` for smooth interpolation between setpoints.

### Distance → Pivot Angle

| Distance (m) | 1.0 | 1.5 | 2.0 | 2.5 | 3.0 | 3.5 | 4.0 |
|---|---|---|---|---|---|---|---|
| Angle (°) | 15 | 20 | 23 | 30 | 35 | 35 | 35 |

### Distance → Flywheel RPM

| Distance (m) | 1.0 | 1.5 | 2.0 | 2.5 | 3.0 | 3.5 | 4.0 |
|---|---|---|---|---|---|---|---|
| RPM | 1500 | 1800 | 2000 | 2600 | 3000 | 3400 | 3800 |

---

## Controller Bindings

Driver controller on **port 0** (Xbox):

| Input | Action |
|---|---|
| Left Stick Y/X | Field-oriented drive (40% speed) |
| Right Stick X | Robot rotation |
| Start + Back (hold) | Zero gyro |
| POV Up | Deploy intake |
| POV Down | Retract intake |
| POV Left / Right | Manual turret adjust (−/+ 15°) |
| Left Bumper | Reverse intake roller |
| A | Stop flywheel |
| Y (hold) | IntakeRetractCommand (retract to 50% + reverse) |
| X (hold) | Manual feeder shoot (staged) |
| Left Trigger (hold) | Acquire fuel (deploy + roller) |
| Right Trigger (hold) | AutoAimAndShootCommand (full vision pipeline) |
| Right Bumper (hold) | PassShootCommand (counter-rotate + shoot) |

---

## Autonomous

Autonomous mode is selected via `SendableChooser` in `RobotContainer`. PathPlanner is integrated for path following.

### PathPlanner Named Commands

| Name | Action |
|---|---|
| `test` | Prints "I EXIST" |
| `intake` | Acquire fuel (forward) |
| `intake_reverse` | Acquire fuel (reverse) |
| `spin_up_shooter` | Sets flywheel to 3000 RPM |
| `stop_shooter` | Stops flywheel |
| `pass_shoot` | PassShootCommand |
| `auto_aim` | AutoAimCommand |

### PathPlanner Configuration

| Parameter | Value |
|---|---|
| Robot Width | 0.628 m |
| Robot Length | 0.477 m |
| Holonomic Mode | true |
| Max Velocity | 3.0 m/s |
| Max Acceleration | 3.0 m/s² |
| Max Angular Velocity | 540°/s |
| Max Angular Acceleration | 720°/s² |

### Available Paths

- `Auto Test.path`
- `SamplePath.path`

---

## Vendor Dependencies

| Library | Version | Purpose |
|---|---|---|
| **YAGSL** | 2026.4.1 | Swerve drive abstraction — primary drivetrain library |
| **Phoenix 6** | 26.3.0 | CTRE TalonFX / Falcon 500 motor controllers |
| **Phoenix 5** | 5.36.0 | Legacy CTRE TalonSRX / VictorSPX support |
| **REVLib** | 2026.0.5 | REV SPARK MAX / FLEX motor controllers (NEO motors) |
| **PathPlannerLib** | 2026.1.2 | Autonomous path following and auto builder |
| **PhotonLib** | v2026.3.4 | PhotonVision camera support |
| **ReduxLib** | 2026.1.2 | Redux IMU / sensor support (YAGSL dependency) |
| **ThriftyLib** | 2026.1.2 | ThriftyBot encoder / sensor support (YAGSL dependency) |
| **YAMS** | 2026.4.10.3 | Yet Another Mechanism System |
| **Studica** | 2026.0.0 | Studica motor / sensor support |
| **WPILibNewCommands** | 1.0.0 | WPILib command-based framework |

---

## Build & Deploy

### Prerequisites

- [WPILib 2026](https://wpilib.org) (includes VS Code, Java 17, Gradle)
- Robot connected to the robot network (for deploy) or running simulation

### Commands

```bash
# Build the project
.\gradlew.bat build

# Deploy to RoboRIO
.\gradlew.bat deploy

# Run desktop simulation
.\gradlew.bat sim

# Run all tests
.\gradlew.bat test

# Run a specific test class
.\gradlew.bat test --tests "frc.robot.ExampleTest"

# Clean build artifacts
.\gradlew.bat clean
```

On Linux/macOS, use `./gradlew` instead of `.\gradlew.bat`.

### Deploy Directory

Files placed in `src/main/deploy/` are automatically copied to `/home/lvuser/deploy/` on the RoboRIO at deploy time. This is used for:
- YAGSL swerve configuration JSONs (`deploy/swerve/falcon/`)
- PathPlanner paths and autos (`deploy/pathplanner/`)

---

## Simulation

The project supports desktop simulation via WPILib's SimGUI:

```bash
.\gradlew.bat sim
```

This launches the robot in simulation mode with the Driver Station GUI, allowing testing of commands and subsystems without physical hardware.

**Simulation files** (auto-generated, git-ignored):
- `simgui.json` — SimGUI layout
- `simgui-ds.json` — Driver Station state
- `simgui-window.json` — Window positions
- `ctre_sim/` — Phoenix simulation device config

---

## License

This project is licensed under the **WPILib BSD License**. See [WPILib-License.md](WPILib-License.md) for details.

Copyright 2009–2026 FIRST and other WPILib contributors. All rights reserved.
