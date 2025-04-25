// Created by Gabriel R

package frc.robot.commands.complex;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.MTR;
import frc.robot.Robot;
import frc.robot.Constants.MechConstants;
import frc.robot.subsystems.CoralIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralInSafe extends Command {
  private static CoralIntake coralIntake;

  /** Creates a new CoralIn. */
  public CoralInSafe() {
    coralIntake = CoralIntake.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(coralIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    coralIntake.eat();
    MTR.coralScorer.set(-MechConstants.CORAL_INTAKE_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralIntake.stop();
    MTR.coralScorer.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //If coral is completely inside of the scorer then were done intaking 
    return coralIntake.isGapBlocked() && Robot.hasCoral();
  }
}
