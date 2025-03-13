package frc.robot.commands.combos;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.MechConstants;
import frc.robot.commands.closed.ElevatorSetPosition;
import frc.robot.commands.closed.SetJawAngle;
import frc.robot.commands.complex.CoralInSafe;
import frc.robot.subsystems.LEDStrip;
import frc.robot.subsystems.LEDStrip.SubsystemPriority;


public class ElevatorIntakeCombo extends ParallelCommandGroup {

  /** Creates a new ElevatorIntakeCombo. */
  public ElevatorIntakeCombo() {

    addCommands(
      new ElevatorSetPosition(ElevatorConstants.INTAKE_HEIGHT),
      new CoralInSafe()
      //new SetJawAngle(MechConstants.JAW_CORAL_STOP_ANGLE)
    );

  }
}
