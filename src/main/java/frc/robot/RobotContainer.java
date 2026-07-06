package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AimAndSpinUpCommand;
import frc.robot.commands.PassShootCommand;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightVisionSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

public class RobotContainer {

  final CommandXboxController driverXbox = new CommandXboxController(0);
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/falcon"));
  private final LEDSubsystem led = new LEDSubsystem();
  private final SendableChooser<Command> autoChooser;
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final FlywheelSubsystem flywheel = new FlywheelSubsystem();
  private final TurretSubsystem turret = new TurretSubsystem();
  private final PivotSubsystem pivot = new PivotSubsystem();
  private final FeederSubsystem feeder = new FeederSubsystem();
  private final LimelightVisionSubsystem vision;

  // Preset RPMs for pass/shoot (tune on real robot)
  private static final double PASS_RPM = 2000.0;
  private static final double PASS_PIVOT_ANGLE = 10.0; // degrees

  /**
   * Field-Relative Control.
   * Left Stick = Translate.
   * Right Stick = Self-rotation (rotate in place).
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
          () -> driverXbox.getLeftY(),
          () -> driverXbox.getLeftX())
      .withControllerRotationAxis(() -> -driverXbox.getRightY())
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  public RobotContainer() {
    vision = drivebase.getVision();

    configureBindings();

    // Register PathPlanner Named Commands
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    // Intake NamedCommands
    NamedCommands.registerCommand("intake", intake.acquireFuelCommand(() -> false));
    NamedCommands.registerCommand("intake_reverse", intake.acquireFuelCommand(() -> true));
    NamedCommands.registerCommand("retract_intake", intake.retractIntake());

    // Shooter NamedCommands
    NamedCommands.registerCommand("spin_up_shooter",
        Commands.run(() -> flywheel.setTargetRPM(3000), flywheel));
    NamedCommands.registerCommand("stop_shooter",
        Commands.runOnce(flywheel::stop, flywheel));
    NamedCommands.registerCommand("aim_and_shoot",
        new AimAndSpinUpCommand(turret, flywheel, pivot, vision, drivebase, driveAngularVelocity));
    NamedCommands.registerCommand("pass_shoot",
        new PassShootCommand(turret, flywheel, pivot, PASS_RPM, PASS_PIVOT_ANGLE, 0));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Choose", autoChooser);
  }

  private void configureBindings() {
    // 1. Default drive command (only if drivetrain is enabled)
    if (OperatorConstants.DRIVE_ENABLED) {
      Command driveFieldOriented = drivebase.driveFieldOriented(driveAngularVelocity);
      drivebase.setDefaultCommand(driveFieldOriented);
    }

    // 2. Driver button bindings

    // A: Reset gyroscope
    driverXbox.a().onTrue(Commands.runOnce(drivebase::zeroGyro));

    // X: Chassis X-pattern lock
    driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

    // --- INTAKE ---

    // Left Trigger: Deploy intake and acquire fuel
    driverXbox.leftTrigger().whileTrue(intake.acquireFuelCommand(() -> false));
    driverXbox.povUp().whileTrue(intake.acquireFuelCommand(() -> true));

    // B: Retract intake
    driverXbox.b().onTrue(intake.retractIntake());

    // --- FEEDER (TEST) ---

    // Y: Feeder idle (slowly reverse)
    driverXbox.y().whileTrue(feeder.idleCommand());

    // Left Bumper: Feeder shoot (wheel 80%, feeder 60%)
    driverXbox.leftBumper().whileTrue(feeder.shootCommand());

    // --- SHOOTER ---

    // Right Trigger: Aim turret, spin up flywheel (drive-by-aim only if drivetrain enabled)
    if (OperatorConstants.DRIVE_ENABLED) {
      driverXbox.rightTrigger().whileTrue(
          new AimAndSpinUpCommand(turret, flywheel, pivot, vision, drivebase, driveAngularVelocity)
      );
    } else {
      // Drivetrain disabled: only aim turret + spin flywheel (no chassis drive)
      driverXbox.rightTrigger().whileTrue(
          new AimAndSpinUpCommand(turret, flywheel, pivot, vision, null, null)
      );
    }

    // Right Bumper: Pass/shoot preset (no vision, fixed RPM + pivot angle)
    // driverXbox.rightBumper().whileTrue(
    //     new PassShootCommand(turret, flywheel, pivot, PASS_RPM, PASS_PIVOT_ANGLE, 0)
    // );

    // --- MANUAL TEST (D-Pad) ---

    driverXbox.povRight().whileTrue(Commands.run(() -> turret.setTargetAngle(45), turret));
    driverXbox.povLeft().whileTrue(Commands.run(() -> turret.setTargetAngle(-45), turret));
    driverXbox.povDown().whileTrue(Commands.run(() -> turret.setTargetAngle(0), turret));
    driverXbox.back().whileTrue(Commands.run(() -> pivot.setTargetAngle(30.0), pivot));
    driverXbox.start().whileTrue(Commands.run(() -> pivot.setTargetAngle(0.0), pivot));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
