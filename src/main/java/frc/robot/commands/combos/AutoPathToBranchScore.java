// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.combos;

import java.util.function.Supplier;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.FieldConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.MechConstants;
import frc.robot.commands.basic.CoralRetract;
import frc.robot.commands.basic.CoralScore;
import frc.robot.commands.closed.DriveToBranchPID;
import frc.robot.commands.closed.DriveToClosestBranch;
import frc.robot.commands.closed.ElevatorSetPosition;
import frc.robot.commands.closed.SetJawAngle;
import frc.robot.commands.complex.SafeAlgaeJoystick;
import frc.robot.commands.complex.SwerveDrive;
import frc.robot.subsystems.Drivetrain;


public class AutoPathToBranchScore extends SequentialCommandGroup {

  public static Drivetrain drivetrain;
  

  public AutoPathToBranchScore(Alliance alliance, String ppAutoName, String reefFace, String branchDirection, double level) {
    
    drivetrain = Drivetrain.getInstance();
    int tagId = FieldConstants.getTagFromReef(alliance, reefFace);

    // Set the desired elevator height
    double elevatorLevelHeight = ElevatorConstants.ELEVATOR_L2;
    if(level == 3){
      elevatorLevelHeight = ElevatorConstants.ELEVATOR_L3;
    } else if(level == 4){
      elevatorLevelHeight = ElevatorConstants.ELEVATOR_L4;
    }

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

    
    addCommands(

      // 1. Wait for 2 seconds
      new WaitCommand(2),
      
      // 2. Move coral forward a tiny bit to avoid elevator jamming
      new ParallelRaceGroup(
        //new SafeAlgaeJoystick(() -> 0.5),
        new CoralScore(),
        new WaitCommand(0.1)  
      ),

      // 3. Move jaw up out of the way
      new ParallelRaceGroup(
        new SetJawAngle(MechConstants.JAW_UP_ANGLE),
        new WaitCommand(2)
      ),

      // 4. Use PathPlanner to drive first route to Reef Face
      new ParallelRaceGroup(
        new WaitCommand(3),
        new PathPlannerAuto(ppAutoName)
      ),
 

      // 5. Use Vision to align to a specific tag
      new ParallelDeadlineGroup(
        new WaitCommand(2),
        new DriveToBranchPID(tagId, branchDirection),
        new SetJawAngle(MechConstants.JAW_UP_ANGLE)
      ),

      // 6. Bump up against the Reef Face manually
      // new ParallelDeadlineGroup(
      //   new WaitCommand(.5),
      //   new SwerveDrive(
      //     () -> -0.3,  //speed to drive forward
      //     () -> 0.0,
      //     () -> 0.0,   
      //     () -> false)
      // ),


      // 7. Move elevator up to desired elevator position
      new ParallelRaceGroup(
        new WaitCommand(3),
        new ElevatorSetPosition(elevatorLevelHeight),
        new DriveToClosestBranch("LEFT")
      ),
 
      // 8. For 2 seconds, spin the coral and maintain the elevator's position
      new ParallelDeadlineGroup(
        new WaitCommand(2),
        new CoralScore(),
        new ElevatorSetPosition(elevatorLevelHeight)
      ),

      // 9. Bring the Elevator down
      new ParallelDeadlineGroup(
        new WaitCommand(2),
        new ElevatorSetPosition(ElevatorConstants.INTAKE_HEIGHT)
      )
      

    );
  }
}
