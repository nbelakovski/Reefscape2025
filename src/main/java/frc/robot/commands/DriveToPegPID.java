// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPegPID extends Command {

  private Drivetrain drivetrain;
  private Camera cam;
  private int tagID;

  private double startDistanceX;
  private double startDistanceY;

  private double setpointX;
  private double setpointY;


  private PIDController controllerX;
  private PIDController controllerY;

  
  /** Creates a new DriveToPegPID. */
  public DriveToPegPID(int tagID) {
    this.tagID = tagID;

    drivetrain = Drivetrain.getInstance(); 
    cam = Camera.getInstance();

    setpointX = cam.getTagPose(tagID).getX() - 0.3;
    setpointY = cam.getTagPose(tagID).getY();

    controllerX = new PIDController(0.9, 0.0, 0.0);
    startDistanceX = drivetrain.getPose().getX();
    controllerX.setSetpoint(setpointX);
    controllerX.setTolerance(0.05);

    controllerY = new PIDController(0.7, 0.0, 0.0);
    startDistanceY = drivetrain.getPose().getY();
    controllerY.setSetpoint(setpointY);
    controllerY.setTolerance(0.05);

    
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.stopDrive(); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //calculating the distance
    double currentDistanceX = drivetrain.getPose().getX();
    double xSpeed = controllerX.calculate(currentDistanceX);

    //TODO: calculating the strafe
    double currentDistanceY = drivetrain.getPose().getY();
    double ySpeed = controllerY.calculate(currentDistanceY);

    drivetrain.setDrive(xSpeed, ySpeed, 0.0);

    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("ySpeed", ySpeed);
    SmartDashboard.putNumber("SetpointX", setpointX);
    SmartDashboard.putNumber("SetpointY", setpointY);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopDrive(); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controllerX.atSetpoint() && controllerY.atSetpoint();
  }
}
