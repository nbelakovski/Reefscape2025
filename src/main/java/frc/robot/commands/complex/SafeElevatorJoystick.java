package frc.robot.commands.complex;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;


public class SafeElevatorJoystick extends Command {

  private Elevator elevator;
  private Supplier<Double> speed;
  

  public SafeElevatorJoystick( Supplier<Double> speed) {
    this.speed = speed;
    elevator = Elevator.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(CoralIntake.getInstance().isGapBlocked() && Elevator.getInstance().getPosition() < 15){
      elevator.stop();
    }
    else{
      elevator.move(MathUtil.applyDeadband(speed.get(), 0.2));
    }   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    elevator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  
}