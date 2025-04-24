package frc.robot.commands.complex;


import java.util.function.Supplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeHandler;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SafeAlgaeJoystick extends Command {

  private static AlgaeHandler algaeHandler;
  private Supplier<Double> speed;


  /** Creates a new SafeAlgaeJoystick. */
  public SafeAlgaeJoystick( Supplier<Double> speed) {
    this.speed = speed;
    algaeHandler = AlgaeHandler.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(algaeHandler);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    algaeHandler.stopPivot();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speedVal = speed.get();

    speedVal = MathUtil.clamp(speedVal, -0.8, 0.8);
    speedVal = MathUtil.applyDeadband(speedVal, 0.2);
    
    algaeHandler.pivot(speedVal);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algaeHandler.stopPivot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
