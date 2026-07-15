package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PassShootConstants;
import frc.robot.commands.AutoAimAndShootCommand;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.IntakeRetractCommand;
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
  
  private double pivotTestAngle = 0.0;
  private double turretTestAngle = 0.0;

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
          () -> driverXbox.getLeftY(),
          () -> driverXbox.getLeftX())
      .withControllerRotationAxis(() -> -driverXbox.getRightX())
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.4)
      .allianceRelativeControl(true);

  public RobotContainer() {
    
    // 【需求1】让 Feeder 默认处于闲置反转状态
    // feeder.setDefaultCommand(feeder.idleCommand());

    configureBindings();

    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    NamedCommands.registerCommand("intake", intake.acquireFuelCommand(() -> false));
    NamedCommands.registerCommand("intake_reverse", intake.acquireFuelCommand(() -> true));
    NamedCommands.registerCommand("spin_up_shooter",
        Commands.run(() -> flywheel.setTargetRPM(3000), flywheel));
    NamedCommands.registerCommand("stop_shooter",
        Commands.runOnce(flywheel::stop, flywheel));
    NamedCommands.registerCommand("pass_shoot",
        new PassShootCommand(drivebase, turret, flywheel, pivot, feeder));
    NamedCommands.registerCommand("auto_aim", new AutoAimCommand(drivebase, turret, pivot));

    autoChooser = AutoBuilder.buildAutoChooser();
    // 按照需求清理 SmartDashboard，如果你连 Auto Chooser 都不想要，可以注释掉下面这行
    SmartDashboard.putData("Auto Choose", autoChooser);
  }

  private void configureBindings() {
    if (OperatorConstants.DRIVE_ENABLED) {
      Command driveFieldOriented = drivebase.driveFieldOriented(driveAngularVelocity);
      drivebase.setDefaultCommand(driveFieldOriented);
    }

    // --- GYRO ---
    driverXbox.start().and(driverXbox.back())
        .onTrue(Commands.runOnce(drivebase::zeroGyro));

    // --- INTAKE DEPLOY ---
    driverXbox.povUp().onTrue(Commands.runOnce(intake::deployOut, intake));
    driverXbox.povDown().onTrue(Commands.runOnce(intake::setDeployBack, intake));
    driverXbox.povLeft().onTrue(Commands.runOnce(() -> {
      turretTestAngle -= 15.0;
      turret.setTargetAngle(turretTestAngle);
    }, turret));
    driverXbox.povRight().onTrue(Commands.runOnce(() -> {
      turretTestAngle += 15.0;
      turret.setTargetAngle(turretTestAngle);
    }, turret));

    // --- FLYWHEEL TEST ---
    driverXbox.leftBumper().whileTrue(intake.reverseIntakeCommand());
    driverXbox.a().onTrue(Commands.runOnce(flywheel::stop, flywheel));

    // --- Y键：Intake回收 + 滚轮反转 ---
    driverXbox.y().whileTrue(new IntakeRetractCommand(intake));

    // --- X键：手动供弹测试 ---
    driverXbox.x().whileTrue(feeder.shootCommand());

    // --- Left Trigger：进件 (deploy + roller) ---
    driverXbox.leftTrigger().whileTrue(intake.acquireFuelCommand(() -> false));

    // --- Right Trigger：AutoAim + AutoShoot 联合瞄准射击 ---
    driverXbox.rightTrigger().whileTrue(
        new AutoAimAndShootCommand(drivebase, turret, pivot, flywheel, feeder)
    );

    // --- Right Bumper：PassShoot (turret补偿车身朝向 + 射击) ---
    driverXbox.rightBumper().whileTrue(
        new PassShootCommand(drivebase, turret, flywheel, pivot, feeder)
    );
  }

  public Command getAutonomousCommand() { return autoChooser.getSelected(); }
  public void setMotorBrake(boolean brake) { drivebase.setMotorBrake(brake); }
  public void resetShooterSystems() {
    flywheel.stop();
    pivot.setTargetAngle(0.0);
    pivotTestAngle = 0.0;
    turret.setTargetAngle(0.0);
    turretTestAngle = 0.0;
  }
}