package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.MechConstants;

// CommandFactory
public class CF {
    public static Command retractCommand() {
        return Commands.startEnd(
            () -> Motors.coralScorer(MechConstants.CORAL_RETRACT_SPEED),
            () -> Motors.coralScorer(0),
            (Subsystem)null
        );
    }

    public static Command scoreCommand() {
        return Commands.startEnd(
            () -> Motors.coralScorer(MechConstants.CORAL_SCORE_SPEED),
            () -> Motors.coralScorer(0),
            (Subsystem)null
        );
    }
}
