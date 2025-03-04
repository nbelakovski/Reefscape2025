// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.combos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.closed.ElevatorSetPosition;
import frc.robot.commands.complex.CoralInSafe;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ElevatorIntakeCombo extends ParallelCommandGroup {
  /** Creates a new ElevatorIntakeCombo. */
  public ElevatorIntakeCombo() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new CoralInSafe(),
      new ElevatorSetPosition(ElevatorConstants.INTAKE_HEIGHT)
    );
  }
}
