//GR
package frc.robot.commands.combos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.MechConstants;
import frc.robot.subsystems.AlgaeHandler;
import frc.robot.subsystems.Elevator;
//untested
/** Add your docs here. */
public class ElevatorSpitCombo extends ParallelCommandGroup {
    /** Creates a new ElevatorSputCombo */

    public ElevatorSpitCombo(double position) {

        addCommands(
            Elevator.getInstance().setPosition(position),
            AlgaeHandler.getInstance().jawAngleCommand(MechConstants.JAW_INTAKE_ANGLE),
            AlgaeHandler.getInstance().spitCommand()
        );

    }
}
