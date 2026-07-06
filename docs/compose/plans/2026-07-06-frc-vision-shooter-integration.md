# FRC Vision & Shooter Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use compose:subagent (recommended) or compose:execute to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Integrate dual Limelight 3G cameras for vision-assisted shooting with automatic turret/pivot aiming and flywheel spin-up.

**Architecture:** Create a VisionSubsystem to handle both odometry (back Limelight) and targeting (front Limelight). Build an AimAndSpinUpCommand that links vision data to turret, pivot, and flywheel control using lookup tables. Map controller bindings for on-demand targeting.

**Tech Stack:** WPILib Command-Based, Phoenix 6 (TalonFX), REVLib (SPARK MAX), Limelight 3G, YAGSL Swerve, PathPlanner

---

## Task 1: Update Constants.java with Vision and Limelight Configuration

**Covers:** Task 1 requirements

**Files:**
- Modify: `src/main/java/frc/robot/Constants.java:1-81`

- [ ] **Step 1: Add VisionConstants inner class**

```java
// Add inside Constants class, after TurretConstants
public static final class VisionConstants {
    // Limelight Names (must match NetworkTables names in Limelight UI)
    public static final String BACK_LIMELIGHT = "limelight-back";  // For odometry
    public static final String FRONT_LIMELIGHT = "limelight-front"; // For targeting (on turret)
    
    // Turret-to-Camera transform (front Limelight on turret)
    // Adjust these values based on your physical mounting
    public static final double TURRET_CAM_X = 0.0;  // meters forward from turret pivot
    public static final double TURRET_CAM_Y = 0.0;  // meters left of turret pivot
    public static final double TURRET_CAM_Z = 0.2;  // meters up from turret pivot
    public static final double TURRET_CAM_PITCH = 15.0; // degrees tilt up from horizontal
    
    // Vision standard deviations for pose estimation
    public static final double[] SINGLE_TAG_STD_DEV = {4.0, 4.0, 8.0};
    public static final double[] MULTI_TAG_STD_DEV = {0.5, 0.5, 1.0};
}
```

- [ ] **Step 2: Add ShooterLookupConstants for interpolation tables**

```java
// Add inside Constants class
public static final class ShooterLookupConstants {
    // Distance (meters) -> Pivot Angle (degrees) lookup
    public static final double[] DISTANCE_TO_PIVOT_DISTANCE = {1.0, 2.0, 3.0, 4.0, 5.0};
    public static final double[] DISTANCE_TO_PIVOT_ANGLE = {10.0, 20.0, 30.0, 40.0, 50.0};
    
    // Distance (meters) -> Flywheel RPM lookup
    public static final double[] DISTANCE_TO_RPM_DISTANCE = {1.0, 2.0, 3.0, 4.0, 5.0};
    public static final double[] DISTANCE_TO_RPM_RPM = {2000.0, 2500.0, 3000.0, 3500.0, 4000.0};
}
```

- [ ] **Step 3: Run verification**

Run: `gradlew.bat build`
Expected: BUILD SUCCESSFUL

- [ ] **Step 4: Commit**

```bash
git add src/main/java/frc/robot/Constants.java
git commit -m "feat: add vision and shooter lookup constants"
```

---

## Task 2: Create VisionSubsystem

**Covers:** Task 1 requirements (VisionSubsystem)

**Files:**
- Create: `src/main/java/frc/robot/subsystems/VisionSubsystem.java`
- Modify: `src/main/java/frc/robot/Constants.java` (already done in Task 1)

- [ ] **Step 1: Create VisionSubsystem skeleton with constructors**

```java
package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
    
    private final NetworkTable backLimelight;
    private final NetworkTable frontLimelight;
    
    public VisionSubsystem() {
        backLimelight = NetworkTableInstance.getDefault().getTable(VisionConstants.BACK_LIMELIGHT);
        frontLimelight = NetworkTableInstance.getDefault().getTable(VisionConstants.FRONT_LIMELIGHT);
    }
```

- [ ] **Step 2: Add back Limelight methods for odometry**

```java
    /**
     * Get the robot pose estimated by the back Limelight (MegaTag 2).
     * @param gyroHeading Current robot heading from Pigeon 2
     * @return Estimated Pose2d, or empty if no valid estimate
     */
    public java.util.Optional<Pose2d> getBackLimelightPose(Rotation2d gyroHeading) {
        // Check if we have a valid pose
        double[] poseArray = backLimelight.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
        if (poseArray.length < 6) {
            return java.util.Optional.empty();
        }
        
        // Extract x, y, yaw from the Limelight pose
        double x = poseArray[0];
        double y = poseArray[1];
        double yaw = poseArray[5]; // Limelight yaw in degrees
        
        // Create pose with Limelight yaw, but we'll use gyro for heading
        // The Pigeon 2 heading should be incorporated into the pose estimator
        Pose2d llPose = new Pose2d(
            new Translation2d(x, y),
            Rotation2d.fromDegrees(yaw)
        );
        
        return java.util.Optional.of(llPose);
    }
    
    /**
     * Get the robot pose estimated by the back Limelight using gyro for heading.
     * This is the preferred method for MegaTag 2 integration.
     * @param gyroHeading Current robot heading from Pigeon 2
     * @return Estimated Pose2d with gyro heading, or empty if no valid estimate
     */
    public java.util.Optional<Pose2d> getBackLimelightPoseWithGyro(Rotation2d gyroHeading) {
        double[] poseArray = backLimelight.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
        if (poseArray.length < 6) {
            return java.util.Optional.empty();
        }
        
        double x = poseArray[0];
        double y = poseArray[1];
        
        // Use gyro heading instead of Limelight yaw to resolve pose ambiguity
        return java.util.Optional.of(new Pose2d(
            new Translation2d(x, y),
            gyroHeading
        ));
    }
```

- [ ] **Step 3: Add front Limelight methods for targeting**

```java
    /**
     * Get the horizontal offset (tx) from the front Limelight.
     * @return tx in degrees, 0 if no target
     */
    public double getTargetTX() {
        return frontLimelight.getEntry("tx").getDouble(0.0);
    }
    
    /**
     * Get the vertical offset (ty) from the front Limelight.
     * @return ty in degrees, 0 if no target
     */
    public double getTargetTY() {
        return frontLimelight.getEntry("ty").getDouble(0.0);
    }
    
    /**
     * Check if the front Limelight has a valid target.
     * @return true if target is visible
     */
    public boolean hasTarget() {
        return frontLimelight.getEntry("tv").getDouble(0.0) > 0.5;
    }
    
    /**
     * Get the distance to the target estimated from ty.
     * Assumes target is at a known height (adjust TARGET_HEIGHT as needed).
     * @return Estimated distance in meters, -1 if no target
     */
    public double getEstimatedDistance() {
        if (!hasTarget()) {
            return -1.0;
        }
        
        // Camera height and target height (adjust these for your field)
        double cameraHeight = 0.8; // meters from ground
        double targetHeight = 2.5; // meters from ground (speaker height)
        double cameraPitch = Math.toRadians(VisionConstants.TURRET_CAM_PITCH);
        
        double ty = getTargetTY();
        double angleToTarget = cameraPitch + Math.toRadians(ty);
        
        // Distance = (targetHeight - cameraHeight) / tan(angle)
        double distance = (targetHeight - cameraHeight) / Math.tan(angleToTarget);
        
        return distance;
    }
```

- [ ] **Step 4: Add periodic method for SmartDashboard**

```java
    @Override
    public void periodic() {
        // Back Limelight status
        SmartDashboard.putBoolean("Vision/Back LL Active", 
            backLimelight.getEntry("tv").getDouble(0.0) > 0.5);
        
        // Front Limelight targeting data
        SmartDashboard.putBoolean("Vision/Front LL Has Target", hasTarget());
        SmartDashboard.putNumber("Vision/Target TX", getTargetTX());
        SmartDashboard.putNumber("Vision/Target TY", getTargetTY());
        SmartDashboard.putNumber("Vision/Estimated Distance", getEstimatedDistance());
    }
}
```

- [ ] **Step 5: Run verification**

Run: `gradlew.bat build`
Expected: BUILD SUCCESSFUL

- [ ] **Step 6: Commit**

```bash
git add src/main/java/frc/robot/subsystems/VisionSubsystem.java
git commit -m "feat: add VisionSubsystem with dual Limelight support"
```

---

## Task 3: Create AimAndSpinUpCommand

**Covers:** Task 2 requirements (AimAndSpinUpCommand)

**Files:**
- Create: `src/main/java/frc/robot/commands/AimAndSpinUpCommand.java`
- Modify: `src/main/java/frc/robot/Constants.java` (already done in Task 1)

- [ ] **Step 1: Create command skeleton with lookup tables**

```java
package frc.robot.commands;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterLookupConstants;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AimAndSpinUpCommand extends Command {
    
    private final TurretSubsystem turret;
    private final PivotSubsystem pivot;
    private final FlywheelSubsystem flywheel;
    private final VisionSubsystem vision;
    
    // Interpolation lookup tables
    private final InterpolatingDoubleTreeMap distanceToPivotAngle = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap distanceToRPM = new InterpolatingDoubleTreeMap();
    
    public AimAndSpinUpCommand(
            TurretSubsystem turret,
            PivotSubsystem pivot,
            FlywheelSubsystem flywheel,
            VisionSubsystem vision) {
        this.turret = turret;
        this.pivot = pivot;
        this.flywheel = flywheel;
        this.vision = vision;
        
        // Build lookup tables from constants
        for (int i = 0; i < ShooterLookupConstants.DISTANCE_TO_PIVOT_DISTANCE.length; i++) {
            distanceToPivotAngle.put(
                ShooterLookupConstants.DISTANCE_TO_PIVOT_DISTANCE[i],
                ShooterLookupConstants.DISTANCE_TO_PIVOT_ANGLE[i]
            );
        }
        
        for (int i = 0; i < ShooterLookupConstants.DISTANCE_TO_RPM_DISTANCE.length; i++) {
            distanceToRPM.put(
                ShooterLookupConstants.DISTANCE_TO_RPM_DISTANCE[i],
                ShooterLookupConstants.DISTANCE_TO_RPM_RPM[i]
            );
        }
        
        addRequirements(turret, pivot, flywheel);
    }
```

- [ ] **Step 2: Add execute method with targeting logic**

```java
    @Override
    public void execute() {
        if (!vision.hasTarget()) {
            // No target visible - don't move anything
            return;
        }
        
        // 1. Turret aiming: Drive tx to 0
        double tx = vision.getTargetTX();
        turret.setTargetAngle(-tx); // Negative because we want to center the target
        
        // 2. Get estimated distance for pivot and flywheel
        double distance = vision.getEstimatedDistance();
        if (distance < 0) {
            return; // Invalid distance
        }
        
        // 3. Look up pivot angle from distance
        double pivotAngle = distanceToPivotAngle.get(distance);
        pivot.setAngle(pivotAngle);
        
        // 4. Look up flywheel RPM from distance
        double targetRPM = distanceToRPM.get(distance);
        flywheel.setTargetRPM(targetRPM);
    }
```

- [ ] **Step 3: Add end method for cleanup**

```java
    @Override
    public void end(boolean interrupted) {
        // When trigger is released, return everything to stow positions
        turret.setTargetAngle(0);      // Return turret to center
        pivot.setAngle(0);             // Stow pivot
        flywheel.stop();               // Stop flywheels
    }
    
    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted by trigger release
    }
}
```

- [ ] **Step 4: Run verification**

Run: `gradlew.bat build`
Expected: BUILD SUCCESSFUL

- [ ] **Step 5: Commit**

```bash
git add src/main/java/frc/robot/commands/AimAndSpinUpCommand.java
git commit -m "feat: add AimAndSpinUpCommand for vision-assisted shooting"
```

---

## Task 4: Update RobotContainer with New Subsystems and Bindings

**Covers:** Task 3 requirements (Controller mapping)

**Files:**
- Modify: `src/main/java/frc/robot/RobotContainer.java:1-84`

- [ ] **Step 1: Add imports and instantiate new subsystems**

```java
// Add to imports section
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.commands.AimAndSpinUpCommand;

// Add after existing subsystem declarations (around line 24)
private final VisionSubsystem vision = new VisionSubsystem();
private final TurretSubsystem turret = new TurretSubsystem();
private final PivotSubsystem pivot = new PivotSubsystem();
private final FlywheelSubsystem flywheel = new FlywheelSubsystem();
```

- [ ] **Step 2: Create AimAndSpinUpCommand instance**

```java
// Add after subsystem declarations
private final AimAndSpinUpCommand aimAndSpinUp = new AimAndSpinUpCommand(
    turret, pivot, flywheel, vision
);
```

- [ ] **Step 3: Add controller bindings for aiming**

```java
// Add inside configureBindings() method, after existing bindings

// Right Trigger: Aim and Spin Up (while held)
driverXbox.rightTrigger().whileTrue(aimAndSpinUp);
```

- [ ] **Step 4: Add drive-by-aiming mode**

```java
// Add a new SwerveInputStream for aim mode that uses vision for rotation
SwerveInputStream driveAngularVelocityWithAim = SwerveInputStream.of(
        drivebase.getSwerveDrive(),
        () -> driverXbox.getLeftY(),
        () -> driverXbox.getLeftX())
    .withControllerRotationAxis(() -> {
        // When aiming, use vision tx for rotation instead of triggers
        if (aimAndSpinUp.isScheduled()) {
            // Return normalized tx as rotation (-1 to 1)
            double tx = vision.getTargetTX();
            return -tx / 30.0; // Normalize assuming 30 degree FOV
        } else {
            // Normal trigger rotation
            return driverXbox.getLeftTriggerAxis() - driverXbox.getRightTriggerAxis();
        }
    })
    .deadband(OperatorConstants.DEADBAND)
    .scaleTranslation(0.2)
    .allianceRelativeControl(true);

// Update the default command to use the new stream
Command driveFieldOriented = drivebase.driveFieldOriented(driveAngularVelocityWithAim);
drivebase.setDefaultCommand(driveFieldOriented);
```

- [ ] **Step 5: Run verification**

Run: `gradlew.bat build`
Expected: BUILD SUCCESSFUL

- [ ] **Step 6: Commit**

```bash
git add src/main/java/frc/robot/RobotContainer.java
git commit -m "feat: integrate vision subsystem and aim command with controller bindings"
```

---

## Task 5: Register PathPlanner Named Commands for Autonomous

**Covers:** Task 4 requirements (Autonomous Integration)

**Files:**
- Modify: `src/main/java/frc/robot/RobotContainer.java:1-84`

- [ ] **Step 1: Create autonomous commands**

```java
// Add after the AimAndSpinUpCommand declaration

// Autonomous Intake Command
private final Command autoIntake = intake.acquireFuelCommand(() -> false)
    .withTimeout(2.0); // Safety timeout

// Autonomous Shoot Command (simplified for auto - no vision, fixed position)
private final Command autoShoot = Commands.sequence(
    Commands.runOnce(() -> turret.setTargetAngle(0)),     // Center turret
    Commands.runOnce(() -> pivot.setAngle(30)),            // Fixed pivot angle
    Commands.runOnce(() -> flywheel.setTargetRPM(3000)),   // Fixed RPM
    new edu.wpi.first.wpilibj2.command.WaitCommand(1.0),  // Wait for spin-up
    Commands.runOnce(() -> intake.setRollerSpeed(-1))      // Eject note
).finallyDo(() -> {
    flywheel.stop();
    pivot.setAngle(0);
    turret.setTargetAngle(0);
    intake.setRollerSpeed(0);
});
```

- [ ] **Step 2: Register NamedCommands in constructor**

```java
// Update the NamedCommands registration in constructor (around line 48)
NamedCommands.registerCommand("intake", autoIntake);
NamedCommands.registerCommand("shoot", autoShoot);
NamedCommands.registerCommand("test", Commands.print("I EXIST"));
```

- [ ] **Step 3: Run verification**

Run: `gradlew.bat build`
Expected: BUILD SUCCESSFUL

- [ ] **Step 4: Commit**

```bash
git add src/main/java/frc/robot/RobotContainer.java
git commit -m "feat: register PathPlanner named commands for autonomous"
```

---

## Task 6: Add PivotSubsystem.setAngle Method

**Covers:** Task 2 requirements (PivotSubsystem needs setAngle method)

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/PivotSubsystem` (or create if not exists)

- [ ] **Step 1: Check if PivotSubsystem exists and add setAngle method**

```java
// If PivotSubsystem.java doesn't exist, create it. If it exists, add this method:

public void setAngle(double targetDegrees) {
    // Convert degrees to motor rotations based on gear ratio
    double targetMotorRotations = (targetDegrees / 360.0) * PIVOT_GEAR_RATIO;
    
    // Apply soft limits check
    if (targetDegrees > PivotConstants.MAX_ANGLE_DEG || targetDegrees < PivotConstants.MIN_ANGLE_DEG) {
        return; // Out of range, don't move
    }
    
    // Send position command to NEO 550 via SPARK MAX
    m_pivotMotor.getPIDController().setReference(
        targetMotorRotations,
        ControlType.kPosition
    );
}
```

- [ ] **Step 2: Run verification**

Run: `gradlew.bat build`
Expected: BUILD SUCCESSFUL

- [ ] **Step 3: Commit**

```bash
git add src/main/java/frc/robot/subsystems/PivotSubsystem.java
git commit -m "feat: add setAngle method to PivotSubsystem"
```

---

## Summary

This plan implements:
1. **VisionSubsystem** - Dual Limelight integration (back for odometry, front for targeting)
2. **AimAndSpinUpCommand** - Links vision to turret, pivot, and flywheel control
3. **Controller Mapping** - Right trigger activates aim mode with drive-by-aiming
4. **Autonomous Integration** - PathPlanner named commands for auto sequences

**Testing Strategy:**
- Build verification after each task
- Test in simulation with SimGUI
- Deploy and test on robot with actual Limelight cameras
- Tune lookup tables based on real-world distance measurements

**Next Steps After Implementation:**
1. Calibrate Limelight mounting positions (update VisionConstants)
2. Tune shooter lookup tables with real distance measurements
3. Test vision pose estimation accuracy
4. Verify autonomous sequences in PathPlanner GUI
