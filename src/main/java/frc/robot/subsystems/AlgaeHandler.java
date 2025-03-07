//Created by Gabriel R
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.utils.Ports;
import frc.robot.Constants;
import frc.robot.Constants.MechConstants;

import com.revrobotics.spark.SparkMax;

import static edu.wpi.first.units.Units.Degree;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.DigitalInput;

public class AlgaeHandler extends SubsystemBase {
/** Creates a new Intake. */
private SparkMax tongueMotor;
private SparkMaxConfig motorConfig;
private SparkMax jawMotor;
private SparkMaxConfig jawConfig;
private static AlgaeHandler instance;
private Timer timer;
private DigitalInput touchSensor;
private boolean haveAlgae = false;
//private AbsoluteEncoder encoder;
private AbsoluteEncoder encoder;

private AlgaeHandler() {
tongueMotor = new SparkMax(Ports.ALGAE_TONGUE_MOTOR_PORT, MotorType.kBrushless);
motorConfig = new SparkMaxConfig();
jawMotor = new SparkMax(Ports.ALGAE_JAW_MOTOR_PORT, MotorType.kBrushless);
jawConfig = new SparkMaxConfig();

motorConfig.idleMode(IdleMode.kBrake);
jawConfig.inverted(true);

encoder = jawMotor.getAbsoluteEncoder();
jawConfig.absoluteEncoder.positionConversionFactor(360);
jawConfig.absoluteEncoder.zeroOffset(0.94);

// encoder.setPositionConversionFactor(360); tried making it to 360 but method wont work

touchSensor = new DigitalInput(Ports.DIGITAL_ALGAEHANDLER_PORT);

tongueMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
jawMotor.configure(jawConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
}
public static AlgaeHandler getInstance(){
if(instance == null) {
instance = new AlgaeHandler();
}
return instance;
}
//
public void spit(){
tongueMotor.set(MechConstants.ALGAE_INTAKE_SPEED);
}
public void eat(){
tongueMotor.set(-MechConstants.ALGAE_INTAKE_SPEED);
}
public void stop(){
tongueMotor.set(0);
}
// public void zeroAngle() {
//     encoder.setPosition(0);
// }

public void pivot(double speed){
speed = MathUtil.clamp(speed, -0.5, 0.5);
jawMotor.set(speed);
}
public void stopPivot(){
jawMotor.set(0);
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

public double getAngle(){
//return encoder.getPosition()*90/32-2.3;
return encoder.getPosition();
}

@Override
public void periodic() {
// This method will be called once per scheduler run
SmartDashboard.putBoolean("Touching Algae", haveAlgae);
SmartDashboard.putNumber("Angle of Jaw", getAngle());
}
}