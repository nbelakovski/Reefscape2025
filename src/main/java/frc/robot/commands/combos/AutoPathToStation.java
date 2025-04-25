// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.combos;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.CF;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;


public class AutoPathToStation extends SequentialCommandGroup {

  public static Drivetrain drivetrain;
  

  public AutoPathToStation(String ppAutoName) {
    
    drivetrain = Drivetrain.getInstance();
    
    addCommands(

     
      // 1. Use PathPlanner to drive first route to Reef Face
      new ParallelRaceGroup(
        new WaitCommand(3),
        new PathPlannerAuto(ppAutoName)
      ),
 

      // //2. Use Vision to align to a specific tag
      // new ParallelDeadlineGroup(
      //   new WaitCommand(3),
      //   new DriveToClosestBranch(branchDirection),
      //   new SetJawAngle(MechConstants.JAW_UP_ANGLE)
      // ),

      // 3. Bump up against the Reef Face manually
      // new ParallelDeadlineGroup(
      //   new WaitCommand(.5),
      //   new SwerveDrive(
      //     () -> -0.3,  //speed to drive forward
      //     () -> 0.0,
      //     () -> 0.0,   
      //     () -> false)
      // ),


      // 4. Move elevator up to intake //need it to trigger the end
      new ParallelDeadlineGroup(
        new WaitCommand(3),
        Elevator.getInstance().setIntake(),
        CF.coralInSafeCommand()
      )
      // // 5. Bring the Elevator down
      // new ParallelDeadlineGroup(
      //   new WaitCommand(2),
      //   new ElevatorSetPosition(ElevatorConstants.INTAKE_HEIGHT)
      // )
      

    );
  }
}
