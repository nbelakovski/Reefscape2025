package frc.robot.commands.complex;

import java.util.function.Supplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Drivetrain;


public class SwerveDrive extends Command {

  private static Drivetrain drivetrain;
  private Supplier<Double> xSpeedSupplier;
  private Supplier<Double> ySpeedSupplier;
  private Supplier<Double> rotSpeedSupplier;
  private Supplier<Boolean> fieldResetSupplier;

  
  /** Creates a new SwerveDrive. */
  public SwerveDrive(Supplier<Double> xSpeedSupplier, Supplier<Double> ySpeedSupplier,
    Supplier<Double> rotSpeedSupplier, Supplier<Boolean> fieldResetSupplier) {
    this.xSpeedSupplier = xSpeedSupplier;
    this.ySpeedSupplier = ySpeedSupplier;
    this.rotSpeedSupplier = rotSpeedSupplier;
    this.fieldResetSupplier = fieldResetSupplier;


    drivetrain = Drivetrain.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.stopDrive();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = xSpeedSupplier.get();
    double xSpeedDeadbanded = MathUtil.applyDeadband(xSpeed, 0.1);
    double xSpeedScaled = xSpeedDeadbanded * SwerveConstants.TOP_SPEED;

    double ySpeed = ySpeedSupplier.get();
    double ySpeedDeadbanded = MathUtil.applyDeadband(ySpeed, 0.1);
    double ySpeedScaled = ySpeedDeadbanded * SwerveConstants.TOP_SPEED;

    double rotSpeed = rotSpeedSupplier.get();
    double rotSpeedDeadbanded = MathUtil.applyDeadband(rotSpeed, 0.1);
    double rotSpeedScaled = rotSpeedDeadbanded * SwerveConstants.TOP_ANGULAR_SPEED;

    drivetrain.setDrive(xSpeedScaled, ySpeedScaled, rotSpeedScaled, true, true);
    
    if(fieldResetSupplier.get()) {
      drivetrain.zeroRobotHeading();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      drivetrain.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
