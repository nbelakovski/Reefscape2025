// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.combos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CF;
import frc.robot.Constants.MechConstants;
import frc.robot.commands.closed.DriveToClosestBranch;
import frc.robot.subsystems.Elevator;


public class AutoPathToBranchScore  {
    public static SequentialCommandGroup cmd(String ppAutoName, String branchDirection, double level) {
      // Table for AprilTag IDs
      // 6	Red Reef F --> (Blue 19)
      // 7	Red Reef A --> (Blue 18)
      // 8	Red Reef B --> (Blue 17)
      // 9	Red Reef C --> (Blue 22)
      // 10	Red Reef D --> (Blue 21)
      // 11	Red Reef E --> (Blue 20)
      // 17	Blue Reef B --> (Red 8)
      // 18	Blue Reef A --> (Red 7)
      // 19	Blue Reef F --> (Red 6)
      // 20	Blue Reef E --> (Red 11)
      // 21	Blue Reef D --> (Red 10)
      // 22	Blue Reef C --> (Red 9)

      return
        // Move coral forward a tiny bit to avoid elevator jamming
        CF.scoreCommand().withDeadline(Commands.waitSeconds(0.1))
          .andThen(  // Use Vision to align to a specific tag
            new DriveToClosestBranch(branchDirection).alongWith(
              CF.jawAngleCommand(MechConstants.JAW_UP_ANGLE).andThen(
                // Move elevator up to desired elevator position
                Elevator.getInstance().setPosition(level).repeatedly()
              )
            ).withDeadline(Commands.waitSeconds(6)))
          .andThen(  // For 2 seconds, spin the coral and maintain the elevator's position
            CF.scoreCommand().alongWith(
              Elevator.getInstance().setPosition(level).repeatedly()
            ).withDeadline(Commands.waitSeconds(2)))
          .andThen(  // Bring the Elevator down
            Elevator.getInstance().setIntake().withDeadline(Commands.waitSeconds(2))
          );
  }
}
