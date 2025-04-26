// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.combos;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto2Piece {
  /** Creates a new Auto2Piece. */
  public static SequentialCommandGroup cmd(String startSide) {
    if(startSide.equals("LEFT")){
      return
        AutoPathToBranchScore.cmd( "Start-E-Auto", "RIGHT", ElevatorConstants.L4).andThen(
        new AutoPathToStation("E-to-Station-Auto")).andThen(
        AutoPathToBranchScore.cmd("Station-to-F-Auto", "LEFT", ElevatorConstants.L4));

    } else { // startSide.equals("RIGHT")
      return
        AutoPathToBranchScore.cmd("Start-to-C-Auto", "LEFT", ElevatorConstants.L4).andThen(
        new AutoPathToStation("C-to-Station-Auto")).andThen(
        AutoPathToBranchScore.cmd("Station-to-B-Auto", "RIGHT", ElevatorConstants.L4));
    }
  }
}
