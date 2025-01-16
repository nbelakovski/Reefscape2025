// Created by Gabriel R
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralOutake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralOut extends Command {

  private CoralOutake coralOutake;
  /** Creates a new CoralOut. */
  public CoralOut() {
    coralOutake = CoralOutake.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(coralOutake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    coralOutake.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    coralOutake.out();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralOutake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
