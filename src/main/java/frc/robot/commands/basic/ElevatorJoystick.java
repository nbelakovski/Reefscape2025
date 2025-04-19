package frc.robot.commands.basic;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;


public class ElevatorJoystick extends Command {

  private Elevator elevator;
  private Supplier<Double> speed;
  

  public ElevatorJoystick( Supplier<Double> speed) {
    this.speed = speed;
    elevator = Elevator.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.zeroPosition();
    elevator.setSpeed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.setSpeed(MathUtil.applyDeadband(speed.get(), 0.1));
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