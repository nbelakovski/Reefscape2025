//Created by Gabriel R
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.utils.Ports;
import frc.robot.Constants;
import frc.robot.Constants.MechConstants;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SerialPort;

public class CoralOutake extends SubsystemBase {
  /** Creates a new Intake. */
  private SparkMax CoralOutakeMotor;
  private SparkMaxConfig motorConfig;
  private static CoralOutake instance;
  private Timer timer;
  

  private CoralOutake() {
    CoralOutakeMotor = new SparkMax(10, MotorType.kBrushless);
    motorConfig = new SparkMaxConfig();
  }
  public static CoralOutake getInstance(){
    if(instance == null) {
      instance = new CoralOutake();
    }
    return instance;
  }
  //
  public void in(){
    CoralOutakeMotor.set(-MechConstants.INTAKE_SPEED);
  }
  public void out(){
    CoralOutakeMotor.set(MechConstants.INTAKE_SPEED);
  }
  public void stop(){
    CoralOutakeMotor.set(0);
  }

  public void changeSpeed(double newSpeed){
    MechConstants.INTAKE_SPEED = newSpeed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
