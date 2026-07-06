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

    // Vision alignment
    public static final boolean VISION_DRIVE_ENABLED = true; // Enable PhotonVision pose estimation
    public static final double  AIM_P_GAIN           = 0.05; // P-gain for vision target aiming
  }

  public static final class FlywheelConstants {
        public static final int FRONT_RIGHT_ID = 10; 
        public static final int BACK_LEFT_ID = 11;
        public static final int FRONT_LEFT_ID = 12;
        public static final int BACK_RIGHT_ID = 13;
        
        // You can also put your current limits and PID values here later!
        public static final double STATOR_CURRENT_LIMIT = 60.0;
  }

  public static final class PivotConstants {
      public static final int PIVOT_ID = 20; // TODO: Update ID
      public static final double GEAR_RATIO = 320.0; // 320:1
      
      // Soft limits in DEGREES
      public static final float MAX_ANGLE_DEG = 60.0f; 
      public static final float MIN_ANGLE_DEG = 0.0f;  // Stowed position
  }

  public static final class TurretConstants {
      public static final int TURRET_ID = 21; // TODO: Update ID
      public static final double GEAR_RATIO = 400.0 / 14.0; 
      
      // Soft limits in ROTATIONS (Phoenix 6 uses rotations natively)
      // 90 degrees = 0.25 rotations
      public static final double MAX_ROTATIONS = 0.25; 
      public static final double MIN_ROTATIONS = -0.25; 
  }
}
