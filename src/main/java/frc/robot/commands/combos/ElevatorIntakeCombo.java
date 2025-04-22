package frc.robot.commands.combos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.complex.CoralInSafe;
import frc.robot.subsystems.Elevator;


public class ElevatorIntakeCombo extends ParallelCommandGroup {

  /** Creates a new ElevatorIntakeCombo. */
  public ElevatorIntakeCombo() {

    addCommands(
      Elevator.getInstance().setIntake(),
      new CoralInSafe()
    );

  }
}
