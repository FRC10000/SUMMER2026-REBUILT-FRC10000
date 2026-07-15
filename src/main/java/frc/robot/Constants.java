// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import java.util.ArrayList;
import java.util.List;
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

    // Joystick input curve: output = maxSpeed * |x|^exponent
    public static final double MAX_TRANSLATION_SPEED = 0.6;
    public static final double CURVE_EXPONENT        = 2.41; // f(0.75) = 0.30

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

      // Target AprilTag IDs (Reef tags) — per alliance
      public static final int[] RED_TAG_IDS = {10, 11};   // Red alliance: priority 10
      public static final int[] BLUE_TAG_IDS = {26, 27};  // Blue alliance: priority 26

      // Turret PID for txnc tracking
      public static final double TURRET_KP = 0.5; // Proportional gain for txnc correction (tune on robot)

      // === PhotonVision Constants ===

      // AprilTag field layout for Reefscape 2026 (built from deploy/fieldmap/FRC2026_ANDYMARK.fmap)
      public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = buildFieldLayout();

      private static AprilTagFieldLayout buildFieldLayout() {
          List<AprilTag> tags = new ArrayList<>();
          // Tag positions extracted from FRC2026_ANDYMARK.fmap (x, y, z in meters)
          tags.add(new AprilTag(1,  new Pose3d(new Translation3d( 3.605,  3.390, 0.889), new Rotation3d())));
          tags.add(new AprilTag(2,  new Pose3d(new Translation3d( 3.642,  0.603, 1.124), new Rotation3d())));
          tags.add(new AprilTag(3,  new Pose3d(new Translation3d( 3.039,  0.355, 1.124), new Rotation3d())));
          tags.add(new AprilTag(4,  new Pose3d(new Translation3d( 3.039,  0.000, 1.124), new Rotation3d())));
          tags.add(new AprilTag(5,  new Pose3d(new Translation3d( 3.642, -0.604, 1.124), new Rotation3d())));
          tags.add(new AprilTag(6,  new Pose3d(new Translation3d( 3.605, -3.390, 0.889), new Rotation3d())));
          tags.add(new AprilTag(7,  new Pose3d(new Translation3d( 3.680, -3.390, 0.889), new Rotation3d())));
          tags.add(new AprilTag(8,  new Pose3d(new Translation3d( 3.998, -0.604, 1.124), new Rotation3d())));
          tags.add(new AprilTag(9,  new Pose3d(new Translation3d( 4.246, -0.356, 1.124), new Rotation3d())));
          tags.add(new AprilTag(10, new Pose3d(new Translation3d( 4.246,  0.000, 1.124), new Rotation3d())));
          tags.add(new AprilTag(11, new Pose3d(new Translation3d( 3.998,  0.603, 1.124), new Rotation3d())));
          tags.add(new AprilTag(12, new Pose3d(new Translation3d( 3.680,  3.390, 0.889), new Rotation3d())));
          tags.add(new AprilTag(13, new Pose3d(new Translation3d( 8.240,  3.370, 0.552), new Rotation3d())));
          tags.add(new AprilTag(14, new Pose3d(new Translation3d( 8.240,  2.939, 0.552), new Rotation3d())));
          tags.add(new AprilTag(15, new Pose3d(new Translation3d( 8.240,  0.291, 0.552), new Rotation3d())));
          tags.add(new AprilTag(16, new Pose3d(new Translation3d( 8.240, -0.141, 0.552), new Rotation3d())));
          tags.add(new AprilTag(17, new Pose3d(new Translation3d(-3.610, -3.390, 0.889), new Rotation3d())));
          tags.add(new AprilTag(18, new Pose3d(new Translation3d(-3.647, -0.604, 1.124), new Rotation3d())));
          tags.add(new AprilTag(19, new Pose3d(new Translation3d(-3.044, -0.356, 1.124), new Rotation3d())));
          tags.add(new AprilTag(20, new Pose3d(new Translation3d(-3.044,  0.000, 1.124), new Rotation3d())));
          tags.add(new AprilTag(21, new Pose3d(new Translation3d(-3.647,  0.603, 1.124), new Rotation3d())));
          tags.add(new AprilTag(22, new Pose3d(new Translation3d(-3.610,  3.390, 0.889), new Rotation3d())));
          tags.add(new AprilTag(23, new Pose3d(new Translation3d(-3.685,  3.390, 0.889), new Rotation3d())));
          tags.add(new AprilTag(24, new Pose3d(new Translation3d(-4.003,  0.603, 1.124), new Rotation3d())));
          tags.add(new AprilTag(25, new Pose3d(new Translation3d(-4.251,  0.355, 1.124), new Rotation3d())));
          tags.add(new AprilTag(26, new Pose3d(new Translation3d(-4.251,  0.000, 1.124), new Rotation3d())));
          tags.add(new AprilTag(27, new Pose3d(new Translation3d(-4.003, -0.604, 1.124), new Rotation3d())));
          tags.add(new AprilTag(28, new Pose3d(new Translation3d(-3.685, -3.390, 0.889), new Rotation3d())));
          tags.add(new AprilTag(29, new Pose3d(new Translation3d(-8.245, -3.371, 0.552), new Rotation3d())));
          tags.add(new AprilTag(30, new Pose3d(new Translation3d(-8.245, -2.939, 0.552), new Rotation3d())));
          tags.add(new AprilTag(31, new Pose3d(new Translation3d(-8.245, -0.291, 0.552), new Rotation3d())));
          tags.add(new AprilTag(32, new Pose3d(new Translation3d(-8.245,  0.140, 0.552), new Rotation3d())));
          // Field dimensions: ~16.54m x 8.13m (Reefscape 2026)
          return new AprilTagFieldLayout(tags, 16.54, 8.13);
      }

      // PhotonVision cameras (names must match PhotonVision UI config)
      public static final String PHOTON_RIGHT = "photon-right";
      public static final String PHOTON_LEFT  = "photon-left";

      // Camera mounting transforms (robot-to-camera) — TUNE AFTER CAD
      public static final Transform3d RIGHT_CAMERA_TRANSFORM = new Transform3d(
          new Translation3d(
              Units.inchesToMeters(0),   // X: forward/back from robot center
              Units.inchesToMeters(-6),  // Y: left/right (negative = right side)
              Units.inchesToMeters(21)   // Z: height from floor
          ),
          new Rotation3d(0, 0, 0)       // Roll, Pitch, Yaw (facing forward)
      );

      public static final Transform3d LEFT_CAMERA_TRANSFORM = new Transform3d(
          new Translation3d(
              Units.inchesToMeters(0),
              Units.inchesToMeters(6),   // Y: positive = left side
              Units.inchesToMeters(21)
          ),
          new Rotation3d(0, 0, 0)       // Roll, Pitch, Yaw (facing forward)
      );

      // Alliance Hub positions (separate for Red and Blue)
      // Red hub: center of tags {10, 11} at x≈4.12, y≈0.30
      // Blue hub: center of tags {26, 27} at x≈-4.13, y≈-0.30
      public static final Translation2d HUB_POSITION_RED  = new Translation2d(4.122, 0.302);
      public static final Translation2d HUB_POSITION_BLUE = new Translation2d(-4.127, -0.302);
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
      public static final double kShootWheelSpeed = 0.5;
  }

  public static final class ShooterLookupConstants {
      // Horizontal distance (meters) -> Pivot Angle (degrees) lookup
      // These are initial placeholders - TUNE ON ROBOT
      public static final double[] DISTANCE_TO_PIVOT_DISTANCE = {1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0};
      public static final double[] DISTANCE_TO_PIVOT_ANGLE =   {15, 20, 23, 30, 35, 35, 35};
      
      // Horizontal distance (meters) -> Flywheel RPM lookup (for future use)
      public static final double[] DISTANCE_TO_RPM_DISTANCE = {1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0};
      public static final double[] DISTANCE_TO_RPM_RPM = {1500, 1800.0, 2000.0, 2600.0, 3000.0, 3400.0, 3800.0};
  }

  public static final class IntakeConstants {
      public static final int DEPLOY_LEFT_ID = 30;
      public static final int DEPLOY_RIGHT_ID = 31;
      public static final int ROLLER_MASTER_ID = 32;
      public static final int ROLLER_FOLLOWER_ID = 33;
      public static final String CAN_BUS = "canivore";
      public static final double GEAR_RATIO = 4.714;
      public static final double FULL_DEPLOY_DEGREES = -4 * 360; // -1440
      public static final double INTAKE_SPEED = -1.0;
      public static final double SYNC_TOLERANCE_DEGREES = 5.0;
      public static final double DEPLOY_KP = 1.1;
      public static final double DEPLOY_KD = 0.2;
      public static final double STATOR_CURRENT_LIMIT = 35.0;
  }

  public static final class PassShootConstants {
      public static final double PASS_RPM = 2000.0;
      public static final double PASS_PIVOT_ANGLE = 30.0;
      public static final double PASS_DELAY_SECONDS = 0.3;
      public static final double MAX_TURRET_HEADING_OFFSET = 90.0;
  }
}
