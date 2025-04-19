// Created by Gabriel & Mansour

package frc.robot.commands.complex;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SafeDescend extends Command {
  /** Creates a new ElevatorDescend. */

  private static Elevator elevator;


  public SafeDescend() {
    // Use addRequirements() here to declare subsystem dependencies.
    elevator = Elevator.getInstance();
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setSpeed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(CoralIntake.getInstance().isGapBlocked() && Elevator.getInstance().getPosition() < 15){
      elevator.setSpeed(0);
    }
    else{
      elevator.setSpeed(-0.3);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
