// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwerveDrive extends Command {
  private Drivetrain drivetrain;
  private Supplier<Double> xSpeed;
  private Supplier<Double> ySpeed;
  private Supplier<Double> rotSpeed;
  private SlewRateLimiter xFilter;
  private SlewRateLimiter yFilter;
  private Supplier<Boolean> fieldReset;

  
  
  /** Creates a new SwerveDrive. */
  public SwerveDrive(Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double> rotSpeed, Supplier<Boolean> fieldReset) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.rotSpeed = rotSpeed;
    this.fieldReset = fieldReset;
    xFilter = new SlewRateLimiter(1.2);
    yFilter = new SlewRateLimiter(1.2);

    drivetrain = Drivetrain.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  public SwerveDrive(Supplier<Double> supplier, Supplier<Double> supplier2, Supplier<Double> supplier3,
      Supplier<Boolean> supplier4, Object object) {
    //TODO Auto-generated constructor stub
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.stopDrive();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.setXSpeed(MathUtil.applyDeadband(xSpeed.get(), 0.1));
    drivetrain.setYSpeed(MathUtil.applyDeadband(ySpeed.get(), 0.1));

      // xFilter.calculate(MathUtil.applyDeadband(xSpeed.get(), 0.1)), 
      // yFilter.calculate(MathUtil.applyDeadband(ySpeed.get(), 0.1)),
    
      if(fieldReset.get()) {
        drivetrain.resetIMU();
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      drivetrain.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
