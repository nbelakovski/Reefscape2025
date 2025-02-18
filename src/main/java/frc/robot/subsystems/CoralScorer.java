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
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SerialPort;

public class CoralScorer extends SubsystemBase {
  /** Creates a new Intake. */
  private SparkMax CoralScorerMotor;
  private SparkMaxConfig motorConfig;
  private static CoralScorer instance;
  private Timer timer;
  private boolean coralInScorer = false;
  private AnalogInput scorerSensor;
  private double distance;
  

  private CoralScorer() {
    CoralScorerMotor = new SparkMax(Ports.CORALSCORER_MOTOR_PORT, MotorType.kBrushless);
    scorerSensor = new AnalogInput(Ports.ANALOG_CORAL_SENSOR_PORT);
    motorConfig = new SparkMaxConfig();

    motorConfig.idleMode(IdleMode.kBrake);
    CoralScorerMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  public static CoralScorer getInstance(){
    if(instance == null) {
      instance = new CoralScorer();
    }
    return instance;
  }
  //
  public void spitSlow(){
    CoralScorerMotor.set(-MechConstants.INTAKE_SPEED);
  }

  public void spitFast(){
    CoralScorerMotor.set(-MechConstants.SCORE_SPEED);
  }

  public void backward(){
    CoralScorerMotor.set(MechConstants.INTAKE_SPEED);
  }
  public void stop(){
    CoralScorerMotor.set(0);
  }

  public void changeSpeed(double newSpeed){
    MechConstants.INTAKE_SPEED = newSpeed;
  }

  public boolean hasCoral(){
    if(getDistance() > 2000){
      coralInScorer = true;
    }
    else{
      coralInScorer = false;
    }
    return coralInScorer;
  }

  public double getDistance(){
    distance = scorerSensor.getValue();
    return distance;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Coral in Scorer", hasCoral());
    SmartDashboard.putNumber("coral anolog distance", getDistance());
  }
}
