// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.basic;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.AlgaeHandler;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ZeroAlgae extends InstantCommand {
  private AlgaeHandler algaeHandler;
  public ZeroAlgae() {
    algaeHandler = AlgaeHandler.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(algaeHandler);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    algaeHandler.zeroAngle();
  }
}
