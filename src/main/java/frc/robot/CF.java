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

    public static Command coralInSafe() {
        return Commands.runEnd(
            () -> {
                Motors.coralIntake(MechConstants.CORAL_INTAKE_SPEED);
                Motors.coralScorer(-MechConstants.CORAL_INTAKE_SPEED);
            },
            () -> {
                Motors.coralIntake(0);
                Motors.coralScorer(0);
            }, (Subsystem)null).until(() -> Robot.isGapBlocked() && Robot.hasCoral()
        );       
    }

    public static Command spitAlgaeCommand() {
        return Commands.startEnd(
            () -> Motors.tongueMotor(MechConstants.ALGAE_INTAKE_SPEED),
            () -> Motors.tongueMotor(0),
            (Subsystem)null);
    }

    public static Command eatAlgaeCommand() {
        return Commands.startEnd(
            () -> Motors.tongueMotor(-MechConstants.ALGAE_INTAKE_SPEED),
            () -> Motors.tongueMotor(0),
            (Subsystem)null);
    }
}
