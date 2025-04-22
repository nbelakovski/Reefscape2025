package frc.robot.commands.combos;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.MechConstants;
import frc.robot.subsystems.AlgaeHandler;

public class ElevatorJawCombo extends ParallelCommandGroup {
  public ElevatorJawCombo() {
    addCommands(
      AlgaeHandler.getInstance().jawAngleCommand(MechConstants.JAW_INTAKE_ANGLE),
      AlgaeHandler.getInstance().spitCommand()
    );

  }
}
