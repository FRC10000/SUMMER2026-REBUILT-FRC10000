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
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightVisionSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.commands.AimAndSpinUpCommand;
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
  private final LimelightVisionSubsystem vision;

  /**
   * Standard Field-Relative Control. 
   * Left Stick = Translate. 
   * Triggers = Rotate (Left Trigger turning left/CCW, Right Trigger turning right/CW).
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
          () -> driverXbox.getLeftY(),
          () -> driverXbox.getLeftX())
      .withControllerRotationAxis(() -> driverXbox.getLeftTriggerAxis() - driverXbox.getRightTriggerAxis())
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.2)
      .allianceRelativeControl(true);

  public RobotContainer() {
    // Initialize vision from the swerve subsystem's Limelight setup
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
    
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Choose", autoChooser);
  }

  private void configureBindings() {
    // 1. Default drive command
    Command driveFieldOriented = drivebase.driveFieldOriented(driveAngularVelocity);
    drivebase.setDefaultCommand(driveFieldOriented);

    // 2. Driver button bindings

    // LB + RB: Reset gyroscope
    driverXbox.leftBumper().and(driverXbox.rightBumper())
        .onTrue(Commands.runOnce(drivebase::zeroGyro));

    // X: Chassis X-pattern lock
    driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

    // Y: Deploy intake and acquire fuel
    driverXbox.y().whileTrue(intake.acquireFuelCommand(() -> false));
    driverXbox.povUp().whileTrue(intake.acquireFuelCommand(() -> true));

    // A: Retract intake
    driverXbox.a().onTrue(intake.retractIntake());

    // --- SHOOTER BINDINGS ---

    // Right Trigger: Aim turret, spin up flywheel, and drive-by-aim
    driverXbox.rightTrigger().whileTrue(
        new AimAndSpinUpCommand(turret, flywheel, pivot, vision, drivebase, driveAngularVelocity)
    );

    // D-Pad Right: Turret to +45 degrees (manual test)
    driverXbox.povRight().whileTrue(Commands.run(() -> turret.setTargetAngle(45), turret));

    // D-Pad Left: Turret to -45 degrees
    driverXbox.povLeft().whileTrue(Commands.run(() -> turret.setTargetAngle(-45), turret));

    // D-Pad Down: Turret to 0
    driverXbox.povDown().whileTrue(Commands.run(() -> turret.setTargetAngle(0), turret));

    // Back: Pivot to 30 degrees
    driverXbox.back().whileTrue(Commands.run(() -> pivot.setTargetAngle(30.0), pivot));

    // Start: Pivot to 0 (stow)
    driverXbox.start().whileTrue(Commands.run(() -> pivot.setTargetAngle(0.0), pivot));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
