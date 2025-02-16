// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

// Check out the 2025 Info Flow Chart for the updated port values:
// https://docs.google.com/spreadsheets/d/1KrIdetRF-3CVOMhvNwwubiZ93xbC6NNtv0gVvd5Vkgo/edit?gid=822794348#gid=822794348

public class Ports {


    //JOYSTICKS
    public static final int DRIVER_CONTROLLER = 0;
    public static final int OPERATOR_CONTROLLER = 1;   


    //CAN NETWORK
    public static final int SWERVE_DRIVE_FL = 1;
    public static final int SWERVE_TURN_FL = 2;

    public static final int SWERVE_DRIVE_FR = 3;
    public static final int SWERVE_TURN_FR = 4;

    public static final int SWERVE_DRIVE_BL = 5;
    public static final int SWERVE_TURN_BL = 6;

    public static final int SWERVE_DRIVE_BR = 7;
    public static final int SWERVE_TURN_BR = 8;

    public static final int ELEVATOR_LEFT_MOTOR_PORT = 9;
    public static final int ELEVATOR_RIGHT_MOTOR_PORT = 10;
  
    public static final int LEFT_INTAKE_MOTOR_PORT = 11;
    public static final int RIGHT_INTAKE_MOTOR_PORT = 12;

    public static final int CORALSCORER_MOTOR_PORT = 13;
    public static final int ALGAE_JAW_MOTOR_PORT = 14;
    public static final int ALGAE_TONGUE_MOTOR_PORT = 15;

    public static final int LEFT_CLIMBER_MOTOR_PORT = 16;
    public static final int RIGHT_CLIMBER_MOTOR_PORT = 17;


    //DIGITAL I/O PORTS
    public static final int DIGITAL_TOP_LIMIT_PORT = 1;
    public static final int DIGITAL_BOTTOM_LIMIT_PORT = 0;
    public static final int DIGITAL_ALGAEHANDLER_PORT = 9;


    //ANALOG IN PORTS
    public static final int ANALOG_GAP_SENSOR_PORT = 0;
    public static final int ANALOG_CORAL_SENSOR_PORT = 1;
  
    public static final int BLINKIN_PORT = 24;


    //PWM PORTS
}
