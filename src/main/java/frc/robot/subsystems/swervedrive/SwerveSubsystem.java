package frc.robot.subsystems.swervedrive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.Vision.Cameras;

import java.io.File;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.targeting.PhotonPipelineResult;

import swervelib.SwerveDrive;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

  private final SwerveDrive swerveDrive;
  private final boolean visionDriveTest = true; // Set to true since you have Limelights
  private Vision vision;

  public SwerveSubsystem(File directory) {
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED, new Pose2d());
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
    
    swerveDrive.setHeadingCorrection(false); 
    swerveDrive.setCosineCompensator(false);
    swerveDrive.setAngularVelocityCompensation(true, true, 0.1); 
    swerveDrive.setModuleEncoderAutoSynchronize(false, 1); 

    if (visionDriveTest) {
      setupPhotonVision();
      swerveDrive.stopOdometryThread();
    }
    setupPathPlanner();
  }

  public void setupPhotonVision() {
    vision = new Vision(swerveDrive::getPose, swerveDrive.field);
  }

  @Override
  public void periodic() {
    if (visionDriveTest) {
      swerveDrive.updateOdometry();
      vision.updatePoseEstimation(swerveDrive);
    }
  }

  public void setupPathPlanner() {
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          this::getPose,
          this::resetOdometry,
          this::getRobotVelocity,
          (speedsRobotRelative, moduleFeedForwards) -> {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces()
              );
          },
          new PPHolonomicDriveController(
              new PIDConstants(5.0, 0.0, 0.0),
              new PIDConstants(5.0, 0.0, 0.0)
          ),
          config,
          () -> {
            var alliance = DriverStation.getAlliance();
            return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
          },
          this
      );
    } catch (Exception e) {
      e.printStackTrace();
    }
    CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
  }

  // Drive Methods

  /**
   * The primary method for controlling the drivebase. Takes a Translation2d and a rotation rate, and
   * calculates and commands module states accordingly.
   *
   * @param translation   The commanded linear velocity of the robot, in meters per second.
   * @param rotation      Robot angular rate, in radians per second. CCW positive.
   * @param fieldRelative Drive mode. True for field-relative, false for robot-relative.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    // The 'false' here tells YAGSL to use closed-loop velocity control instead of open-loop voltage control
    swerveDrive.drive(translation, rotation, fieldRelative, false);
  }

  /**
   * Primary method used by SwerveInputStream to drive the robot.
   */
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> swerveDrive.driveFieldOriented(velocity.get()));
  }

/**
   * Aim the robot at the target returned by PhotonVision while allowing manual translation.
   *
   * @param camera The specific camera/Limelight to use for targeting.
   * @param translationSupplier Supplier of ChassisSpeeds (usually your SwerveInputStream).
   * @return A Command that continuously aligns the robot while allowing the driver to move.
   */
  public Command aimAtTarget(Cameras camera, Supplier<ChassisSpeeds> translationSupplier) {
    return run(() -> {
      // 1. Get the normal teleop translation speeds from the joysticks.
      // This perfectly preserves your deadbands, max speed scaling, and alliance flipping!
      ChassisSpeeds speeds = translationSupplier.get();
      
      // 2. Check vision for targets
      Optional<PhotonPipelineResult> resultO = camera.getBestResult();
      
      if (resultO.isPresent() && resultO.get().hasTargets()) {
        // Calculate rotational speed (P-Loop)
        double yawError = resultO.get().getBestTarget().getYaw();
        
        // Overwrite the joystick's rotation with our vision math
        speeds.omegaRadiansPerSecond = yawError * 0.05; 
      }

      // 3. Command the swerve drive using the combined X/Y from the driver and Rotation from vision
      swerveDrive.driveFieldOriented(speeds);
    });
  }



  public void resetOdometry(Pose2d initialHolonomicPose) {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  public void lock() {
    swerveDrive.lockPose();
  }

  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  // Getters

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase.
   *
   * @return The yaw angle as a Rotation2d
   */
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  /**
   * Get the SwerveDriveConfiguration object.
   *
   * @return The SwerveDriveConfiguration for the current drive.
   */
  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return swerveDrive.swerveDriveConfiguration;
  }

  /**
   * Get the chassis speeds based on controller input of 2 joysticks for absolute drive.
   *
   * @param xInput X joystick input for translation
   * @param yInput Y joystick input for translation
   * @param headingX X joystick which controls the absolute angle of the robot
   * @param headingY Y joystick which controls the absolute angle of the robot
   * @return ChassisSpeeds which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
    return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, headingX, headingY, getHeading().getRadians(), Constants.MAX_SPEED);
  }
  
  /**
   * Get the chassis speeds based on controller input of 2 joysticks and a target angle.
   *
   * @param xInput X joystick input for translation
   * @param yInput Y joystick input for translation
   * @param angle  The desired angle as a Rotation2d
   * @return ChassisSpeeds which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
    return swerveDrive.swerveController.getTargetSpeeds(
        xInput, 
        yInput, 
        angle.getRadians(), 
        getHeading().getRadians(), 
        Constants.MAX_SPEED
    );
  }

  /**
   * Gets the current pitch angle of the robot, as reported by the IMU (Pigeon 2).
   *
   * @return The pitch as a {@link Rotation2d} angle
   */
  public Rotation2d getPitch() {
    return swerveDrive.getPitch();
  }
  
}