//GR
package frc.robot.commands.combos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.MechConstants;
import frc.robot.commands.closed.ElevatorSetPosition;
import frc.robot.commands.closed.SetJawAngle;
import frc.robot.subsystems.AlgaeHandler;
//untested
/** Add your docs here. */
public class ElevatorSpitCombo extends ParallelCommandGroup {
    /** Creates a new ElevatorSputCombo */

    public ElevatorSpitCombo(double position) {

        addCommands(
            new ElevatorSetPosition(position),
            new SetJawAngle(MechConstants.JAW_INTAKE_ANGLE),
            AlgaeHandler.getInstance().spitCommand()
        );

    }
}
