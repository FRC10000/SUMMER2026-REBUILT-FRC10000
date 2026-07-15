// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 148 lbs total - 20.3 lbs bumpers, converted to kg
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.020; //s, standard TimedRobot period
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.2;

    // Set to false to disable drivetrain (for subsystem testing)
    public static final boolean DRIVE_ENABLED  = true;

    // Vision alignment
    public static final boolean VISION_DRIVE_ENABLED = false; // Disable Limelight for manual testing
    public static final double  AIM_P_GAIN           = 0.05; // P-gain for vision target aiming
  }

  public static final class FlywheelConstants {
        public static final int FRONT_RIGHT_ID = 4;
        public static final int BACK_LEFT_ID = 6;
        public static final int FRONT_LEFT_ID = 7;
        public static final int BACK_RIGHT_ID = 5;

        public static final double STATOR_CURRENT_LIMIT = 60.0;
  }

  public static final class PivotConstants {
      public static final int PIVOT_ID = 25;
      public static final double GEAR_RATIO = 320.0;

      public static final float MAX_ANGLE_DEG = 60.0f;
      public static final float MIN_ANGLE_DEG = 0.0f;
  }

  public static final class TurretConstants {
      public static final int TURRET_ID = 24;
      public static final double GEAR_RATIO = 400.0 / 14.0;

      public static final double MAX_ROTATIONS = 0.25;
      public static final double MIN_ROTATIONS = -0.25;
  }

  public static final class VisionConstants {
      // Limelight Names
      public static final String BACK_LIMELIGHT = "limelight-back";  // On turret, for AprilTag targeting
      
      // Camera physical mount (on turret)
      public static final double CAM_HEIGHT_METERS = 0.54;   // Camera lens height from floor 0.52
      public static final double CAM_MOUNT_ANGLE_DEG = 29.3; // Camera tilt upward from horizontal (calibrated)
      
      // AprilTag target height (center of tag on Reef, 2026 Reefscape)
      public static final double TARGET_HEIGHT_METERS = 1.14; // Tag center height
      public static final double HEIGHT_DIFF_METERS = TARGET_HEIGHT_METERS - CAM_HEIGHT_METERS; // 0.62m
      
      // Target AprilTag IDs (Reef tags)
      public static final int[] TARGET_TAG_IDS = {2, 9, 10, 11};
      
      // Turret PID for txnc tracking
      public static final double TURRET_KP = 0.5; // Proportional gain for txnc correction (tune on robot)
  }

  public static final class FeederConstants {
      public static final int kFeederMotorCanId = 34;
      public static final int kFeederWheelMotorCanId = 35;
      public static final int kFeederWheelFollowerCanId = 36;

      public static final double kFeederMaxOutput = 1.0;
      public static final double kFeederWheelMaxOutput = 1.0;
      public static final double kFeederWheelCurrentLimit = 30.0;
      public static final double kFeederMotorCurrentLimit = 30.0;

      // Idle state: slowly rotating inversely when not shooting
      public static final double kIdleFeederSpeed = -0.15;
      public static final double kIdleWheelSpeed = -0.05;

      // Shooting state
      public static final double kShootFeederSpeed = 0.4;
      public static final double kShootWheelSpeed = 0.7;
  }

  public static final class ShooterLookupConstants {
      // Horizontal distance (meters) -> Pivot Angle (degrees) lookup
      // These are initial placeholders - TUNE ON ROBOT
      public static final double[] DISTANCE_TO_PIVOT_DISTANCE = {1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0};
      public static final double[] DISTANCE_TO_PIVOT_ANGLE =   {15, 20, 23, 30, 35, 35, 35};
      
      // Horizontal distance (meters) -> Flywheel RPM lookup (for future use)
      public static final double[] DISTANCE_TO_RPM_DISTANCE = {1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0};
      public static final double[] DISTANCE_TO_RPM_RPM = {1500, 1800.0, 2000.0, 2600.0, 3000.0, 3400.0, 3800.0, 4400};
  }
}
