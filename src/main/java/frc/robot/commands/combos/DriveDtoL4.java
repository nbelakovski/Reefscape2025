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
import frc.robot.commands.DriveToPeg;
import frc.robot.commands.basic.CoralRetract;
import frc.robot.commands.basic.CoralScore;
import frc.robot.commands.closed.ElevatorSetPosition;
import frc.robot.commands.closed.SetJawAngle;
import frc.robot.commands.complex.SwerveDrive;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveDtoL4 extends SequentialCommandGroup {

  public Drivetrain drivetrain;
  

  /** Creates a new DriveDtoL4. */
  public DriveDtoL4() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    drivetrain = Drivetrain.getInstance();
    addCommands(
        new ParallelRaceGroup(
        new CoralScore(),
        new WaitCommand(0.1)  
        ),
        new ParallelRaceGroup(
          new SetJawAngle(MechConstants.JAW_AUTO_ANGLE),
          new WaitCommand(3)
        ),
        new ParallelRaceGroup(
          //new DriveToPeg(21), //b=10,r=21
          new PathPlannerAuto("Auto2"),
          new WaitCommand(2)
        ),
        new ParallelRaceGroup(
          new ElevatorSetPosition(ElevatorConstants.ELEVATOR_L2),
          new WaitCommand(3)
        ),
        new ParallelDeadlineGroup(
          new WaitCommand(3),
          new CoralScore(),
          new ElevatorSetPosition(ElevatorConstants.ELEVATOR_L2)
         
        )
      
    );
  }
}
