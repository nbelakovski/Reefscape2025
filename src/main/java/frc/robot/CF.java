package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
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

    public static Command coralInSafeCommand() {
        return Commands.startEnd(
            () -> {
                MTR.coralIntake(MechConstants.CORAL_INTAKE_SPEED);
                MTR.coralScorer.set(-MechConstants.CORAL_INTAKE_SPEED);
            },
            () -> {
                MTR.coralIntake(0);
                MTR.coralScorer.set(0);
            },
            (Subsystem)null
        ).until(() -> Robot.isGapBlocked() && Robot.hasCoral());
    }

    public static Command algaeEatCommand() {
        return Commands.startEnd(
            () -> MTR.tongueMotor.set(-MechConstants.ALGAE_INTAKE_SPEED),
            () -> MTR.tongueMotor.stopMotor(),
            (Subsystem)null
        );
    }

    public static Command algaeSpitCommand() {
        return Commands.startEnd(
            () -> MTR.tongueMotor.set(MechConstants.ALGAE_INTAKE_SPEED),
            () -> MTR.tongueMotor.stopMotor(),
            (Subsystem)null
        );
    }

    private static PIDController jawAngleController = new PIDController(0.15, 0, 0.02);
    public static Command jawAngleCommand(double desiredAngle) {
        // TODO: I believe all of this can be removed in favor of using
        // the built-in PID controller in the SparkMax. However I don't want to make
        // this change without testing it on the robot, and validating both that it
        // works but also the calling set after setReference (i.e. from
        // safeAlgaeJoystick) works.
        jawAngleController.setTolerance(2);
        return new FunctionalCommand(
            () -> {
                jawAngleController.reset();
                jawAngleController.setSetpoint(desiredAngle);
            },
            () -> {
                double speed = jawAngleController.calculate(SNSR.jawEncoder.getPosition());
                MTR.jawMotor.set(speed);
            },
            (interrupted) -> {
                MTR.jawMotor.stopMotor();
            },
            () -> jawAngleController.atSetpoint(),
            (Subsystem)null
        );
    }

    public static Command safeAlgaeJoystick(Supplier<Double> speedSupplier) {
        return Commands.runEnd(
            () -> {
                double speed = speedSupplier.get();
                speed = MathUtil.clamp(speed, -0.8, 0.8);
                speed = MathUtil.applyDeadband(speed, 0.2);
                MTR.jawMotor.set(speed);
            },
            () -> MTR.jawMotor.stopMotor(),
            (Subsystem)null
        );
    }
}
