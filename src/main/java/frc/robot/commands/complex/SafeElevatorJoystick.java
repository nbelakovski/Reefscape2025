package frc.robot.commands.complex;


import java.util.function.Supplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;


public class SafeElevatorJoystick extends Command {

  private static Elevator elevator;
  private Supplier<Double> speedSupplier;
  

  public SafeElevatorJoystick( Supplier<Double> speedSupplier) {
    this.speedSupplier = speedSupplier;
    elevator = Elevator.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
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
    elevator.setSpeed(MathUtil.applyDeadband(-speedSupplier.get(), 0.2)); 
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