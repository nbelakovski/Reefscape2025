package frc.robot.commands.combos;


import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.basic.CoralRetract;
import frc.robot.commands.closed.ElevatorSetPosition;


public class ElevatorScoreCombo extends SequentialCommandGroup {

  /** Creates a new ElevatorScoreCombo. */
  public ElevatorScoreCombo(double position) {

    addCommands(
      new ElevatorSetPosition(position),
      new ParallelRaceGroup(
        new CoralRetract(),
        new WaitCommand(1)
      )
    );

  }
}
