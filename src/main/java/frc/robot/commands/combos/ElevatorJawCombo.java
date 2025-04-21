package frc.robot.commands.combos;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.MechConstants;
import frc.robot.commands.closed.SetJawAngle;
import frc.robot.subsystems.AlgaeHandler;

public class ElevatorJawCombo extends ParallelCommandGroup {
  public ElevatorJawCombo() {
    addCommands(
      new SetJawAngle(MechConstants.JAW_INTAKE_ANGLE),
      AlgaeHandler.getInstance().spitCommand()
    );

  }
}
