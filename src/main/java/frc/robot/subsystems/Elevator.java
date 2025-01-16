// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.utils.Ports;
import frc.robot.Constants;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SerialPort;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private SparkMax elevatorMotor;
  private SparkMax elevatorDownMotor;
  private RelativeEncoder encoder;
  private SparkMaxConfig topMotorConfig;
  private SparkMaxConfig downMotorConfig;
  private static Elevator instance;


  private Elevator() {
    elevatorMotor = new SparkMax(Ports.ELEVATOR_MOTOR_PORT, MotorType.kBrushless);
    elevatorDownMotor = new SparkMax(Ports.ELEVATOR_MOTOR_DOWN_PORT, MotorType.kBrushless);
    topMotorConfig = new SparkMaxConfig();
    downMotorConfig = new SparkMaxConfig();
    encoder = elevatorMotor.getEncoder();
    
  }

  public static Elevator getInstance() {
    if (instance == null) {
        instance = new Elevator();
    }
    return instance;
  }

  public double getPosition() {
    return encoder.getPosition();
  }

  public void elevate(){
    elevatorMotor.set(0.6);
  }
  
  public void descend(){
    elevatorDownMotor.set(-0.6);
  }
  
  public void stop(){
    elevatorMotor.set(0);
    elevatorDownMotor.set(0);
  }

  public void resetPosition(double pos){
    encoder.setPosition(pos);
  }

  public void resetPosition(){
    encoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("elevator position", getPosition());
  }
}

//jb ms