---
name: frc-motor-subsystem
description: Add a new motor controller to an FRC Command-Based robot project — creates Constants entry, Subsystem, Command, and wires into RobotContainer.
---

# FRC Motor Subsystem Creation

Reusable workflow for adding a new motor-controlled mechanism to an FRC robot project using WPILib Command-Based framework.

## When to use

- User asks to add a new motor, subsystem, or mechanism to the robot
- User asks to control a motor with a joystick axis or button
- Creating a new feeder, intake, elevator, arm, shooter, or similar subsystem

## Prerequisites

- Existing FRC Command-Based project with `Constants.java`, `RobotContainer.java`
- Known motor controller type (TalonFX, TalonSRX, SPARK MAX, VictorSPX, etc.)
- Known CAN ID and CAN bus (if non-default)

## Workflow

### Step 1 — Explore existing codebase

Read the project structure to understand conventions:

```
src/main/java/frc/robot/
├── Constants.java          ← CAN IDs, operator constants
├── Robot.java              ← lifecycle (don't touch unless needed)
├── RobotContainer.java     ← subsystem wiring, command bindings
├── subsystems/             ← Subsystem classes
└── commands/               ← Command classes
```

Read `Constants.java`, `RobotContainer.java`, and any existing subsystem to match the project's style.

### Step 2 — Add CAN ID constant to Constants.java

Add a new constant inside the appropriate inner class (typically `MotorIDs`):

```java
public static final int FEEDER_MOTOR_ID = 35;
```

Also add any operator constants (deadbands, speeds, button IDs) if the user specified them.

### Step 3 — Create or modify Subsystem class

**New subsystem** — create `src/main/java/frc/robot/subsystems/<Name>Subsystem.java`:

```java
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;  // or appropriate import
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class <Name>Subsystem extends SubsystemBase {
    private final TalonFX m_motor;

    public <Name>Subsystem() {
        m_motor = new TalonFX(Constants.MotorIDs.<MOTOR_ID>);
        // Configure motor: brake/coast, inversion, current limits, etc.
    }

    public void setSpeed(double speed) {
        m_motor.set(speed);
    }

    public void stop() {
        m_motor.stopMotor();
    }

    @Override
    public void periodic() {
        // Optional: publish telemetry to SmartDashboard
    }
}
```

**Modify existing subsystem** — add the new motor field, initialize it in the constructor, and add setter/getter methods.

Key decisions:
- Motor type imports: `com.ctre.phoenix6.hardware.TalonFX` for TalonFX/Kraken, `com.revrobotics.CANSparkMax` for SPARK MAX, `com.ctre.talonfx.TalonFXS` for Falcon 500
- Use `CommandScheduler.getInstance().schedule(cmd)` not deprecated `cmd.schedule()` (WPILib 2026)
- All CAN bus fields default to `""` (empty string) for default bus

### Step 4 — Create Command class

Create `src/main/java/frc/robot/commands/<Name>Command.java`:

```java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.<Name>Subsystem;

public class <Name>Command extends Command {
    private final <Name>Subsystem m_subsystem;
    private final CommandXboxController m_controller;

    public <Name>Command(<Name>Subsystem subsystem, CommandXboxController controller) {
        m_subsystem = subsystem;
        m_controller = controller;
        addRequirements(m_subsystem);
    }

    @Override
    public void execute() {
        double speed = m_controller.getLeftY();  // or whichever axis
        m_subsystem.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.stop();
    }
}
```

Alternative: use `Commands.run()` for inline one-liner commands if the logic is simple.

### Step 5 — Wire into RobotContainer

In `RobotContainer.java`:

1. Instantiate the subsystem as a field:
```java
private final <Name>Subsystem m_<name>Subsystem = new <Name>Subsystem();
```

2. Set default command or bind to button in the constructor:
```java
m_<name>Subsystem.setDefaultCommand(new <Name>Command(m_<name>Subsystem, m_driverController));
// OR bind to a trigger:
m_driverController.a().whileTrue(new <Name>Command(m_<name>Subsystem, m_driverController));
```

### Step 6 — Build to verify

```bash
.\gradlew.bat build 2>&1
```

Fix any compilation errors before reporting success.

## Checklist

- [ ] CAN ID added to `Constants.java`
- [ ] Subsystem class created/modified with motor initialization
- [ ] Command class created with `addRequirements()`
- [ ] Subsystem instantiated in `RobotContainer.java`
- [ ] Default command or button binding added
- [ ] Project builds successfully
