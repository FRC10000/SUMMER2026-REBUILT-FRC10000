package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; // 取消这里不必要的 SmartDashboard
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.AutoShootCommand;
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
import frc.robot.commands.AutoShootCommand;

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
    
    // 【需求1】让 Feeder 默认处于闲置反转状态
    feeder.setDefaultCommand(feeder.idleCommand());

    configureBindings();

    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    NamedCommands.registerCommand("intake", intake.acquireFuelCommand(() -> false));
    NamedCommands.registerCommand("intake_reverse", intake.acquireFuelCommand(() -> true));
    NamedCommands.registerCommand("spin_up_shooter",
        Commands.run(() -> flywheel.setTargetRPM(3000), flywheel));
    NamedCommands.registerCommand("stop_shooter",
        Commands.runOnce(flywheel::stop, flywheel));
    NamedCommands.registerCommand("pass_shoot",
        new PassShootCommand(turret, flywheel, pivot, PASS_RPM, PASS_PIVOT_ANGLE, 0));
    NamedCommands.registerCommand("auto_aim", new AutoAimCommand(drivebase, turret, pivot));

    autoChooser = AutoBuilder.buildAutoChooser();
    // 按照需求清理 SmartDashboard，如果你连 Auto Chooser 都不想要，可以注释掉下面这行
    // SmartDashboard.putData("Auto Choose", autoChooser);
  }

  private void configureBindings() {
    if (OperatorConstants.DRIVE_ENABLED) {
      Command driveFieldOriented = drivebase.driveFieldOriented(driveAngularVelocity);
      drivebase.setDefaultCommand(driveFieldOriented);
    }

    // --- GYRO ---
    driverXbox.start().and(driverXbox.back())
        .onTrue(Commands.runOnce(drivebase::zeroGyro));

    // --- PIVOT & TURRET TEST (保留你原本的代码) ---
    driverXbox.povUp().onTrue(Commands.runOnce(() -> {
      pivotTestAngle = Math.min(pivotTestAngle + 10.0, 35.0);
      pivot.setTargetAngle(pivotTestAngle);
    }, pivot));
    driverXbox.povDown().onTrue(Commands.runOnce(() -> {
      pivotTestAngle = Math.max(pivotTestAngle - 10.0, 0.0);
      pivot.setTargetAngle(pivotTestAngle);
    }, pivot));
    driverXbox.povLeft().onTrue(Commands.runOnce(() -> {
      turretTestAngle -= 15.0;
      turret.setTargetAngle(turretTestAngle);
    }, turret));
    driverXbox.povRight().onTrue(Commands.runOnce(() -> {
      turretTestAngle += 15.0;
      turret.setTargetAngle(turretTestAngle);
    }, turret));

    // --- FLYWHEEL TEST ---
    driverXbox.leftBumper().whileTrue(Commands.run(() -> flywheel.setTargetRPM(1000), flywheel));
    driverXbox.rightBumper().whileTrue(Commands.run(() -> flywheel.setTargetRPM(2000), flywheel));
    driverXbox.a().onTrue(Commands.runOnce(flywheel::stop, flywheel));

    // --- 【需求2】 Y键插值射击 (与AutoAim无关) ---
    // 逻辑：按下Y -> 计算距离 -> 启动飞轮(插值或1500) -> 等待提速 -> 供球
    // 松开Y -> 飞轮停止 (Feeder会因为setDefaultCommand自动回退到反转idle)
    driverXbox.y().whileTrue(
        new AutoShootCommand(drivebase, flywheel, feeder)
    );

    // 原来的 X 键 shoot 测试可以保留或注释
    driverXbox.x().whileTrue(feeder.shootCommand());
    driverXbox.leftTrigger().whileTrue(intake.acquireFuelCommand(() -> false));

    // --- AUTOAIM TEST ---
    driverXbox.rightTrigger().whileTrue(new AutoAimCommand(drivebase, turret, pivot));
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