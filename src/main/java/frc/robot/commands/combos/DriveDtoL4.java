// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.combos;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.MechConstants;
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
    addCommands(
      
        new ParallelCommandGroup(
          new SetJawAngle(MechConstants.JAW_UP_ANGLE),
          new WaitCommand(3)
        ),
        new ParallelCommandGroup(
          //new SwerveDrive(1.0, 0.0, 0.0),
          new WaitCommand(2)
        ),
        new ParallelCommandGroup(
          new ElevatorSetPosition(ElevatorConstants.ELEVATOR_L2),
          new WaitCommand(3)
        ),
        new ParallelCommandGroup(
          new CoralScore(),
          new WaitCommand(1)
        )
       
      

      
      
    );
  }
}
