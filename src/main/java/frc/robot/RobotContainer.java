package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;
import frc.robot.subsystems.swervedrive.Vision.Cameras;

public class RobotContainer {

  final CommandXboxController driverXbox = new CommandXboxController(0);
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/falcon"));
  private final SendableChooser<Command> autoChooser;

  /**
   * Standard Field-Relative Control. 
   * Left Stick = Translate. Right Stick X = Rotate.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
          () -> driverXbox.getLeftY(),
          () -> driverXbox.getLeftX())
      .withControllerRotationAxis(() -> -driverXbox.getRightX())
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
      .withControllerHeadingAxis(() -> -driverXbox.getRightX(),
                                 () -> -driverXbox.getRightY())
      .headingWhile(() -> Math.hypot(driverXbox.getRightX(), driverXbox.getRightY()) > 0.33);

  public RobotContainer() {
    configureBindings();
    
    // Register PathPlanner Named Commands here (e.g., for shooting, intake)
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Choose", autoChooser);
  }

  private void configureBindings() {
    // 1. Set the correct default command
    Command driveFieldOriented = drivebase.driveFieldOriented(driveDirectAngle);
    drivebase.setDefaultCommand(driveFieldOriented);

    // 2. Driver Buttons
    driverXbox.a().onTrue(Commands.runOnce(drivebase::zeroGyro)); // Zero Gyro
    driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly()); // X-Stance (Lock wheels)
    driverXbox.b().whileTrue(drivebase.aimAtTarget(Cameras.CENTER_CAM, driveDirectAngle));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}