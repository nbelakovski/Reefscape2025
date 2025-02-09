//Created by Gabriel R
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.utils.Ports;
import frc.robot.Constants;
import frc.robot.Constants.MechConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.DigitalInput;

public class AlgaeHandler extends SubsystemBase {
  /** Creates a new Intake. */
  private SparkMax tongueMotor;
  private SparkMaxConfig motorConfig;
  private static AlgaeHandler instance;
  private Timer timer;
  private DigitalInput touchSensor;
  private boolean haveAlgae = false;
  

  private AlgaeHandler() {
    tongueMotor = new SparkMax(Ports.ALGAE_TONGUE_MOTOR_PORT, MotorType.kBrushless);
    motorConfig = new SparkMaxConfig();

    touchSensor = new DigitalInput(Ports.DIGITAL_ALGAEHANDLER_PORT);
  }
  public static AlgaeHandler getInstance(){
    if(instance == null) {
      instance = new AlgaeHandler();
    }
    return instance;
  }
  //
  public void out(){
    tongueMotor.set(-MechConstants.INTAKE_SPEED);
  }
  public void in(){
    tongueMotor.set(MechConstants.INTAKE_SPEED);
  }
  public void stop(){
    tongueMotor.set(0);
  }

  public void changeSpeed(double newSpeed){
    MechConstants.INTAKE_SPEED = newSpeed;
  }

  public boolean checkAlgae() {
    if (touchSensor.get()) {
      haveAlgae = true;
    }

    else {
      haveAlgae = false;
    }

    return haveAlgae;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Touching Algae", haveAlgae);
  }
}
