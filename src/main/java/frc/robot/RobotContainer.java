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
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

public class RobotContainer {

  final CommandXboxController driverXbox = new CommandXboxController(0);
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/falcon"));
  private final LEDSubsystem led = new LEDSubsystem();
  private final SendableChooser<Command> autoChooser;
  private final IntakeSubsystem intake = new IntakeSubsystem();

  /**
   * Standard Field-Relative Control. 
   * Left Stick = Translate. 
   * Triggers = Rotate (Left Trigger turning left/CCW, Right Trigger turning right/CW).
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
          () -> driverXbox.getLeftY(),
          () -> driverXbox.getLeftX())
      // 【关键改动】：使用 左扳机 - 右扳机。
      // 左扳机按下时得正值 (向左/逆时针转)，右扳机按下时得负值 (向右/顺时针转)
      // 如果你的机器人在测试时转向是反的，可以在前面加个负号： -(... - ...)
      .withControllerRotationAxis(() -> driverXbox.getLeftTriggerAxis() - driverXbox.getRightTriggerAxis())
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.2) // 保持你原有的平移限速
      .allianceRelativeControl(true);

  // 【已移除】删除了 driveDirectAngle，因为你不再使用右摇杆做朝向控制

  public RobotContainer() {
    configureBindings();
    
    // Register PathPlanner Named Commands here (e.g., for shooting, intake)
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Choose", autoChooser);
  }

  private void configureBindings() {
    // 1. 设置默认底盘指令 (仅使用 driveAngularVelocity)
    Command driveFieldOriented = drivebase.driveFieldOriented(driveAngularVelocity);
    drivebase.setDefaultCommand(driveFieldOriented);

    // 2. 驾驶员按键绑定

    // 【关键改动】：LB 和 RB 同时按下，重置陀螺仪
    driverXbox.leftBumper().and(driverXbox.rightBumper())
        .onTrue(Commands.runOnce(drivebase::zeroGyro));

    // X 键：底盘 X 态自锁 (保持原样)
    driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

    // Y 键：放出 intake 并吸球 (因为不涉及反转，传 () -> false 即可)
    // 松开 Y 键时，由于你之前写了 finallyDo()，它会自动切断电机动力并自然滑行
    driverXbox.y().whileTrue(intake.acquireFuelCommand(() -> false));
    driverXbox.povUp().whileTrue(intake.acquireFuelCommand(() -> true));

    // A 键：主动收回 Intake (切回 Brake 模式并锁死在 0 度)
    driverXbox.a().onTrue(intake.retractIntake());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}