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

    public static Command coralInSafe() {
        return Commands.runEnd(
            () -> {
                MTR.coralIntake(MechConstants.CORAL_INTAKE_SPEED);
                MTR.coralScorer.set(-MechConstants.CORAL_INTAKE_SPEED);
            },
            () -> {
                MTR.coralIntake(0);
                MTR.coralScorer.set(0);
            }, (Subsystem)null).until(() -> Robot.isGapBlocked() && Robot.hasCoral()
        );       
    }

    public static Command spitAlgaeCommand() {
        return Commands.startEnd(
            () -> MTR.tongueMotor(MechConstants.ALGAE_INTAKE_SPEED),
            () -> MTR.tongueMotor(0),
            (Subsystem)null);
    }

    public static Command eatAlgaeCommand() {
        return Commands.startEnd(
            () -> MTR.tongueMotor(-MechConstants.ALGAE_INTAKE_SPEED),
            () -> MTR.tongueMotor(0),
            (Subsystem)null);
    }
}
