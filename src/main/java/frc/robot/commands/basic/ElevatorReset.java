// Created by Gabriel & Mansour
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.basic;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorReset extends Command {
  /** Creates a new ElevatorReset. */

  private Elevator elevator;


  public ElevatorReset() {
    // Use addRequirements() here to declare subsystem dependencies.
    elevator = Elevator.getInstance();
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.zeroPosition();
  }
}
