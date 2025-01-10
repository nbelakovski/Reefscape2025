// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ModuleConfig;

public class SwerveModule extends SubsystemBase {

    private final SparkMax driveMotor;
    private final SparkMax turnMotor;

    private final RelativeEncoder driveEncoder;
    private final SparkAbsoluteEncoder turnEncoder;
    
    private final SparkClosedLoopController driveController;
    private final SparkClosedLoopController turnController;

    private final SparkMaxConfig motorConfig;
  
  /** Creates a new SwerveModule. */
  public SwerveModule(ModuleConfig config) {

    //This intitilizes the Drive and Turn motors.
    driveMotor = new SparkMax(config.DRIVE_PORT, MotorType.kBrushless);
    turnMotor = new SparkMax(config.TURN_PORT, MotorType.kBrushless);

    driveEncoder = driveMotor.getEncoder();
    turnEncoder = turnMotor.getAbsoluteEncoder();

    //PID Controllers 
    driveController = driveMotor.getClosedLoopController();
    turnController = turnMotor.getClosedLoopController();

    motorConfig = new SparkMaxConfig();

    //

    //driveMotor.configure()
    driveMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    



  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
