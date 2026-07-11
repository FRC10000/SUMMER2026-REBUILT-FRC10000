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
import frc.robot.commands.PassShootCommand;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
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
  // Manual test: current angle trackers
  private double pivotTestAngle = 0.0;
  private double turretTestAngle = 0.0;

  private static final double PASS_RPM = 2000.0;
  private static final double PASS_PIVOT_ANGLE = 10.0;

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
          () -> driverXbox.getLeftY(),
          () -> driverXbox.getLeftX())
      .withControllerRotationAxis(() -> -driverXbox.getRightX())
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  public RobotContainer() {

    configureBindings();

    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    NamedCommands.registerCommand("intake", intake.acquireFuelCommand(() -> false));
    NamedCommands.registerCommand("intake_reverse", intake.acquireFuelCommand(() -> true));
    // NamedCommands.registerCommand("retract_intake", intake.retractIntake());
    NamedCommands.registerCommand("spin_up_shooter",
        Commands.run(() -> flywheel.setTargetRPM(3000), flywheel));
    NamedCommands.registerCommand("stop_shooter",
        Commands.runOnce(flywheel::stop, flywheel));
    NamedCommands.registerCommand("pass_shoot",
        new PassShootCommand(turret, flywheel, pivot, PASS_RPM, PASS_PIVOT_ANGLE, 0));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Choose", autoChooser);
  }

  private void configureBindings() {
    if (OperatorConstants.DRIVE_ENABLED) {
      Command driveFieldOriented = drivebase.driveFieldOriented(driveAngularVelocity);
      drivebase.setDefaultCommand(driveFieldOriented);
    }

    // --- GYRO ---
    // Start + Back together: reset gyro
    driverXbox.start().and(driverXbox.back())
        .onTrue(Commands.runOnce(drivebase::zeroGyro));

    // --- PIVOT TEST (D-Pad) ---
    // Up: +10°, Down: -10°, Left: stow (0°)
    driverXbox.povUp().onTrue(Commands.runOnce(() -> {
      pivotTestAngle = Math.min(pivotTestAngle + 10.0, 35.0);
      pivot.setTargetAngle(pivotTestAngle);
    }));
    driverXbox.povDown().onTrue(Commands.runOnce(() -> {
      pivotTestAngle = Math.max(pivotTestAngle - 10.0, 0.0);
      pivot.setTargetAngle(pivotTestAngle);
    }));
    // --- TURRET TEST (D-Pad Left/Right) ---
    driverXbox.povLeft().onTrue(Commands.runOnce(() -> {
      turretTestAngle -= 15.0;
      turret.setTargetAngle(turretTestAngle);
    }));
    driverXbox.povRight().onTrue(Commands.runOnce(() -> {
      turretTestAngle += 15.0;
      turret.setTargetAngle(turretTestAngle);
    }));

    // --- FLYWHEEL TEST ---
    // LB: 1000 RPM, RB: 2000 RPM, LT: 3000 RPM, A: stop
    driverXbox.leftBumper().whileTrue(
        Commands.run(() -> flywheel.setTargetRPM(1000), flywheel));
    driverXbox.rightBumper().whileTrue(
        Commands.run(() -> flywheel.setTargetRPM(2000), flywheel));
    driverXbox.rightTrigger().whileTrue(
        Commands.run(() -> flywheel.setTargetRPM(3000), flywheel));
    driverXbox.a().onTrue(Commands.runOnce(flywheel::stop, flywheel));

    // --- FEEDER TEST ---
    // Y: feeder idle (slow reverse), X: feeder shoot
    driverXbox.y().whileTrue(feeder.idleCommand());
    driverXbox.x().whileTrue(feeder.shootCommand());

    driverXbox.leftTrigger().whileTrue(intake.acquireFuelCommand(() -> false));

    // --- INTAKE ---
    // driverXbox.b().onTrue(intake.retractIntake());

    // --- SHOOTER ---
    // driverXbox.rightTrigger().whileTrue(
    //     new PassShootCommand(turret, flywheel, pivot, PASS_RPM, PASS_PIVOT_ANGLE, 0));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  public void resetShooterSystems() {
    flywheel.stop();
    pivot.setTargetAngle(0.0);
    pivotTestAngle = 0.0;
    turret.setTargetAngle(0.0);
    turretTestAngle = 0.0;
  }
}
