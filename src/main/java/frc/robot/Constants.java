// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }



public static final class ModuleConstants {
  // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
  // This changes the drive speed of the module (a pinion gear with more teeth will result in a
  // robot that drives faster).
  public static final int kDrivingMotorPinionTeeth = 14;

  // Invert the turning encoder, since the output shaft rotates in the opposite direction of
  // the steering motor in the MAXSwerve Module.
  public static final boolean kTurningEncoderInverted = true;

  // Calculations required for driving motor conversion factors and feed forward
  public static final double kNeoFreeSpeedRpm = 5676;
  public static final double kDrivingMotorFreeSpeedRps = kNeoFreeSpeedRpm / 60;
  public static final double kWheelDiameterMeters = 0.0762;
  public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
  // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
  public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
  public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
      / kDrivingMotorReduction;

  public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
      / kDrivingMotorReduction; // meters
  public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
      / kDrivingMotorReduction) / 60.0; // meters per second

  public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
  public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

  public static final double kTurningEncoderPositionPIDMinInput = -Math.PI; // radians
  public static final double kTurningEncoderPositionPIDMaxInput = Math.PI; // radians

  public static final double kDrivingP = 0.04;
  public static final double kDrivingI = 0;
  public static final double kDrivingD = 0;
  public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
  public static final double kDrivingMinOutput = -1;
  public static final double kDrivingMaxOutput = 1;

  public static final double kTurningP = 1;
  public static final double kTurningI = 0;
  public static final double kTurningD = 0;
  public static final double kTurningFF = 0;
  public static final double kTurningMinOutput = -1;
  public static final double kTurningMaxOutput = 1;

  public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
  public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

  public static final int kDrivingMotorCurrentLimit = 50; // amps
  public static final int kTurningMotorCurrentLimit = 40; // amps
  }
  public static class MechConstants{

    
    public static final int ENCODER_TICKS = 8192; //Counts per Revolution

    //climber encoders
    // public static final double RIGHT_CLIMB_OFFSET = 0.340;
    // public static final double LEFT_CLIMB_OFFSET = 0.30926;


    //Climber Heights
    //public static final double MAX_CLIMB_RIGHT = 42.0;
    //public static final double BASE_CLIMB_RIGHT = 0.0;
    //public static final double MAX_CLIMB_LEFT = 42.0;
    //public static final double BASE_CLIMB_LEFT = 0.0;

    //Mech Motor Speeds for Buttons
    public static double INTAKE_SPEED = 1.0;
    public static double TELE_INTAKE_SPEED = 1.0;
    public static double AUTO_INTAKE_SPEED = 0.5;
    //public static final double LAUNCHER_SPEED = 1.0;
    //public static final double ARM_PIVOT_SPEED = 1.0;
   // public static final double CLIMBER_SPEED = 0.3;

    //Arm Angles    
    //public static final double START_ANGLE = 94;
    //public static final double FLOOR_ANGLE = 0.0;
    //public static final double LAUNCH_ANGLE = 23.7;
    //public static final double AMP_ANGLE = 106;
    //public static final double ARM_POSITION_TOLERANCE = 1.0;
    //public static final double ARM_OFFSET = 357.2335615;
  }

}