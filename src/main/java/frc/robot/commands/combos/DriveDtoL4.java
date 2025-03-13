// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.combos;

import java.util.function.Supplier;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.MechConstants;
import frc.robot.commands.basic.CoralRetract;
import frc.robot.commands.basic.CoralScore;
import frc.robot.commands.closed.DriveToPeg;
import frc.robot.commands.closed.ElevatorSetPosition;
import frc.robot.commands.closed.SetJawAngle;
import frc.robot.commands.complex.SwerveDrive;
import frc.robot.subsystems.Drivetrain;


public class DriveDtoL4 extends SequentialCommandGroup {

  public static Drivetrain drivetrain;
  

  public DriveDtoL4() {
    
    drivetrain = Drivetrain.getInstance();

    addCommands(
    
      // Move coral forward a tiny bit to avoid elevator jamming
      new ParallelRaceGroup(
        new CoralScore(),
        new WaitCommand(0.1)  
      ),

      // Move jaw up out of the way
      new ParallelRaceGroup(
        new SetJawAngle(MechConstants.JAW_AUTO_ANGLE),
        new WaitCommand(3)
      ),

      // Use AprilCam to drive to a specific ID
      new ParallelDeadlineGroup(
        new WaitCommand(2),
        new DriveToPeg(10) //b=10,r=21
        //new PathPlannerAuto("Auto2"),
        //new SetJawAngle(MechConstants.JAW_AUTO_ANGLE)
      ),

      // Drive forward a little bit more (6") to hit bumper on Reef
      new ParallelRaceGroup(
        //new PathPlannerAuto("Auto2"),
        new SwerveDrive(
          () -> 0.3,  //speed to drive forward
          () -> 0.1,  //adjust the y-offset to align to peg
          () -> 0.0,
          () -> false),
        new WaitCommand(0.5)
      ),

      // Move elevator up to L4 position
      new ParallelRaceGroup(
        new ElevatorSetPosition(ElevatorConstants.ELEVATOR_L4),
        new WaitCommand(3)
      ),

      //For 3 seconds, spin the coral and maintain the L4 position
      new ParallelDeadlineGroup(
        new WaitCommand(3),
        new CoralScore(),
        new ElevatorSetPosition(ElevatorConstants.ELEVATOR_L4)
      )
    
    );

  }
}
