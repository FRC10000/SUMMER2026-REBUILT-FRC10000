package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AutoBalanceCommand extends Command {

  private final SwerveSubsystem swerveSubsystem;
  private final PIDController   controller;

  public AutoBalanceCommand(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    
    // Lowered P-Gain for smoother deceleration as it levels out.
    // You may need to tune this slightly (e.g., up to 0.05) depending on robot weight.
    controller = new PIDController(0.025, 0.0, 0.0);
    
    // Increased tolerance slightly. 1 degree is very strict and might cause stuttering.
    controller.setTolerance(1.5);
    controller.setSetpoint(0.0);
    
    addRequirements(this.swerveSubsystem);
  }

  @Override
  public void initialize() {
    // Reset the PID controller when the command starts
    controller.reset();
  }

  @Override
  public void execute() {
    SmartDashboard.putBoolean("At Tolerance", controller.atSetpoint());

    // Calculate translation speed based on pitch error
    double translationVal = MathUtil.clamp(
        controller.calculate(swerveSubsystem.getPitch().getDegrees(), 0.0), 
        -0.5, 
        0.5
    );

    // Drive the robot. 
    // Notice fieldRelative is set to FALSE so it drives relative to its own orientation.
    swerveSubsystem.getSwerveDrive().drive(
        new Translation2d(translationVal, 0.0), 
        0.0, 
        false, 
        false
    );
  }

  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    // Lock the wheels in an X pattern to prevent rolling off the balance point
    swerveSubsystem.lock();
  }
}