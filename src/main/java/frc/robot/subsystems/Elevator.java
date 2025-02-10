// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.utils.Ports;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SerialPort;


public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private SparkMax elevatorLeftMotor;
  private SparkMax elevatorRightMotor;
  private RelativeEncoder encoder;
  private SparkMaxConfig leftMotorConfig;
  private SparkMaxConfig rightMotorConfig;
  private static Elevator instance;
  private PIDController controller;
  private DigitalInput topLimitSwitch;
  private DigitalInput bottomLimitSwitch;

  



  private Elevator() {
    elevatorLeftMotor = new SparkMax(Ports.ELEVATOR_LEFT_MOTOR_PORT, MotorType.kBrushless);
    elevatorRightMotor = new SparkMax(Ports.ELEVATOR_RIGHT_MOTOR_PORT, MotorType.kBrushless);
    leftMotorConfig = new SparkMaxConfig();
    rightMotorConfig = new SparkMaxConfig();
    encoder = elevatorLeftMotor.getEncoder();
    topLimitSwitch = new DigitalInput(1);
    bottomLimitSwitch = new DigitalInput(2);
    

    controller = new PIDController(1, 0, 0);

    


    rightMotorConfig.inverted(ElevatorConstants.RIGHT_ELEVATOR_INVERTED);

    rightMotorConfig.idleMode(IdleMode.kBrake);
    leftMotorConfig.idleMode(IdleMode.kBrake);
    //leftMotorConfig.encoder.inverted();
    elevatorRightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevatorLeftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    
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

  public void elevate(double speed){

    if (encoder.getPosition() >= ElevatorConstants.ELEVATOR_MAX || topLimitSwitch.get()) {
      elevatorRightMotor.set(speed);
      elevatorLeftMotor.set(-speed);
    }

    else {
      elevatorLeftMotor.set(speed);
      elevatorRightMotor.set(-speed);
    }
  }
  
  public void descend(double speed){

    if (encoder.getPosition() <= ElevatorConstants.ELEVATOR_MIN || bottomLimitSwitch.get()) {
      elevatorLeftMotor.set(speed);
      elevatorRightMotor.set(-speed);
    }

    else {
      elevatorRightMotor.set(speed);
      elevatorLeftMotor.set(-speed);
    }
  }

  public void move(double speed){

    if(speed >0){
      elevate(-speed);
    }
    else if(speed <0){
      descend(speed);
    }
    else{
      stop();
    }
  }
  

  public void stop(){
    elevatorLeftMotor.set(0);
    elevatorRightMotor.set(0);
  }

  public void coralGapStop(){
    if(CoralIntake.getInstance().isGapBlocked()){
      elevatorLeftMotor.set(0);
      elevatorRightMotor.set(0);
    }
    else{

    }
  }

  public void resetPosition(double pos){
    encoder.setPosition(pos);
  }

  public void resetPosition(){
    encoder.setPosition(0);
  }

  public PIDController getController(){
    return controller;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("elevator position", getPosition());
  }
}

//jb ms