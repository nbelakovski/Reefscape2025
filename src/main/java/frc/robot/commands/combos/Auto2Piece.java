// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.combos;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto2Piece extends SequentialCommandGroup {
  /** Creates a new Auto2Piece. */
  public Auto2Piece(String startSide) {

    Command scoreFirstCoral = null;
    Command getSecondCoral = null;
    Command scoreSecondCoral = null;


    Optional<Alliance> allianceOptional = DriverStation.getAlliance();
     Alliance alliance = null;
    if (allianceOptional.isPresent()) {
         alliance = allianceOptional.get();
    }    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    if(startSide.equals("LEFT")){
      
      scoreFirstCoral = new AutoPathToBranchScore( "Start-E-Auto", "RIGHT", 4);
      getSecondCoral = new AutoPathToStation("E-to-Station-Auto");
      scoreSecondCoral = new AutoPathToBranchScore("Station-to-F-Auto", "LEFT", 4);
      } else if(startSide.equals("RIGHT")){

      scoreFirstCoral = new AutoPathToBranchScore("Start-to-C-Auto", "LEFT", 4);
      getSecondCoral = new AutoPathToStation("C-to-Station-Auto");
      scoreSecondCoral = new AutoPathToBranchScore("Station-to-B-Auto", "RIGHT", 4);

      }
    addCommands(
      scoreFirstCoral,
      getSecondCoral,
      scoreSecondCoral

    );
  }
}
