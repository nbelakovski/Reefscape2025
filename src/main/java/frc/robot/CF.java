package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.MechConstants;

// CommandFactory
public class CF {
    public static Command retractCommand() {
        return Commands.startEnd(
            () -> MTR.coralScorer.set(MechConstants.CORAL_RETRACT_SPEED),
            () -> MTR.coralScorer.set(0),
            (Subsystem)null
        );
    }

    public static Command scoreCommand() {
        return Commands.startEnd(
            () -> MTR.coralScorer.set(MechConstants.CORAL_SCORE_SPEED),
            () -> MTR.coralScorer.set(0),
            (Subsystem)null
        );
    }
}
