package frc.robot.commands.combos;


import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.closed.ElevatorSetPosition;
import frc.robot.subsystems.CoralScorer;


public class ElevatorScoreCombo extends SequentialCommandGroup {

  /** Creates a new ElevatorScoreCombo. */
  public ElevatorScoreCombo(double position) {

    addCommands(
      new ElevatorSetPosition(position),
      new ParallelRaceGroup(
        CoralScorer.getInstance().retractCommand(),
        new WaitCommand(1)
      )
    );

  }
}
