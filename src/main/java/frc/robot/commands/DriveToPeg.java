// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPeg extends Command {

  Drivetrain drivetrain;
  Camera cam;
  int tagID;
  private double xSpeed;
  private double ySpeed;
  private double rotSpeed;
  private double tagX;
  private double tagY;

  /** Creates a new DriveToPeg. */
  public DriveToPeg(int tagID) {
    drivetrain = Drivetrain.getInstance();
    cam = Camera.getInstance();
    this.tagID = tagID;
    this.xSpeed = 0.4;
    this.ySpeed = -0.3;
    this.rotSpeed = 0.7;

    tagX = cam.getTagPose(10).getX() -0.7;
    tagY = cam.getTagPose(10).getY();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain, cam);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.stopDrive();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(drivetrain.getPose().getY() > tagY && drivetrain.getPose().getX() < tagX){
      drivetrain.setDrive(xSpeed, ySpeed, 0.0);
    } else if ((drivetrain.getPose().getY() <= tagY + 0.2 || drivetrain.getPose().getY() >= tagY - 0.2) && drivetrain.getPose().getX() < tagX) {
      drivetrain.setDrive(xSpeed, 0.0, 0.0);
    } else {
      drivetrain.stopDrive();
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
    return drivetrain.getPose().getY() <= cam.getTagPose(10).getY() && drivetrain.getPose().getX() >= cam.getTagPose(10).getX() -0.7 ;
  }
}
