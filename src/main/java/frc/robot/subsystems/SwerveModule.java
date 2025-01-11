// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;
import frc.robot.utils.ModuleConfig;

public class SwerveModule extends SubsystemBase {

    private final SparkMax driveMotor;
    private final SparkMax turnMotor;

    private final RelativeEncoder driveEncoder;
    private final SparkAbsoluteEncoder turnEncoder;
    
    private final SparkClosedLoopController driveController;
    private final SparkClosedLoopController turnController;

    private final SparkMaxConfig driveMotorConfig;
    private final SparkMaxConfig turnMotorConfig;

    private final ClosedLoopConfig driveControllerConfig;
    private final ClosedLoopConfig turnControllerConfig;

    private SwerveModuleState desiredState; 

    public final ModuleConfig config;
  
  /** Creates a new SwerveModule. */
  public SwerveModule(ModuleConfig config) {
    this.config = config;

    //This intitilizes the Drive and Turn motors.
    driveMotor = new SparkMax(config.DRIVE_PORT, MotorType.kBrushless);
    turnMotor = new SparkMax(config.TURN_PORT, MotorType.kBrushless);

    driveEncoder = driveMotor.getEncoder();
    turnEncoder = turnMotor.getAbsoluteEncoder();

    //PID Controllers 
    driveController = driveMotor.getClosedLoopController();
    turnController = turnMotor.getClosedLoopController();


    desiredState = new SwerveModuleState(0.0, new Rotation2d() );

    //Istablished the Motor configations with the new Spark Configs
    driveMotorConfig = new SparkMaxConfig();
    turnMotorConfig = new SparkMaxConfig();

    driveControllerConfig = new ClosedLoopConfig();
    turnControllerConfig = new ClosedLoopConfig();

    driveMotorConfig.encoder.positionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
    driveMotorConfig.encoder.velocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

    turnMotorConfig.encoder.positionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
    turnMotorConfig.encoder.velocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);
    
    
    //References the established constants and and tells the Drive SparkMax what to set its PID to
    driveControllerConfig.p(ModuleConstants.kTurningP);
    driveControllerConfig.i(ModuleConstants.kTurningI);
    driveControllerConfig.d(ModuleConstants.kTurningD);
    driveControllerConfig.velocityFF(ModuleConstants.kDrivingFF);
    driveControllerConfig.outputRange(ModuleConstants.kDrivingMinOutput, ModuleConstants.kDrivingMaxOutput);

    //References the established constants and and tells the Turn SparkMax what to set its PID to
    turnControllerConfig.p(ModuleConstants.kTurningP);
    turnControllerConfig.i(ModuleConstants.kTurningI);
    turnControllerConfig.d(ModuleConstants.kTurningD);
    turnControllerConfig.velocityFF(ModuleConstants.kTurningFF);
    turnControllerConfig.outputRange(ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput);

    driveMotorConfig.idleMode(ModuleConstants.kDrivingMotorIdleMode);
    turnMotorConfig.idleMode(ModuleConstants.kTurningMotorIdleMode);

    driveMotorConfig.smartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    turnMotorConfig.smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);


    //flips the drive motor 
    driveMotorConfig.inverted(config.DRIVE_INVERSION);

    //Resets motors back to original
    driveMotor.configure(driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turnMotor.configure(turnMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // give the motors current controller configuration
    driveMotorConfig.apply(driveControllerConfig);
    turnMotorConfig.apply(turnControllerConfig);
    
    desiredState.angle = new Rotation2d(turnEncoder.getPosition());
    driveEncoder.setPosition(0);
  }

  public SwerveModuleState getState(){
    return new SwerveModuleState(driveEncoder.getVelocity(),
     new Rotation2d(turnEncoder.getPosition() - config.ANGULAR_OFFSET)); 
  }

  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(driveEncoder.getPosition(),
     new Rotation2d(turnEncoder.getPosition() - config.ANGULAR_OFFSET));
  }

  public double getTurnRadians(){
    return getPosition().angle.getRadians();
  }

  public void setDesiredState(SwerveModuleState desiredState){
    
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(config.ANGULAR_OFFSET));


    //Maybe come back and fix, potentially not what we want
    correctedDesiredState.optimize(new Rotation2d(turnEncoder.getPosition()));

    driveController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    turnController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

    this.desiredState = correctedDesiredState;
     
  }

   public void resetEncoders() {
    driveEncoder.setPosition(0);
  }

  public void updateTelemetry() {
    SmartDashboard.putNumber(config.NAME + " Angle Degrees", getPosition().angle.getDegrees());
    SmartDashboard.putNumber(config.NAME + " Angle Radians", getTurnRadians());
    SmartDashboard.putNumber(config.NAME + " Drive Position", getPosition().distanceMeters);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
