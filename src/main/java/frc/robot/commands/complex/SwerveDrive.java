package frc.robot.commands.complex;

import java.util.function.Supplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SNSR;
import frc.robot.subsystems.Drivetrain;


public class SwerveDrive extends Command {

  private static Drivetrain drivetrain;
  private Supplier<Double> xSpeedSupplier;
  private Supplier<Double> ySpeedSupplier;
  private Supplier<Double> rotSpeedSupplier;
  private Supplier<Boolean> fieldResetSupplier;
  private Supplier<Boolean> toggleFieldCentricSupplier;
  boolean fieldCentric = true;

  
  /** Creates a new SwerveDrive. */
  public SwerveDrive(Supplier<Double> xSpeedSupplier, Supplier<Double> ySpeedSupplier,
    Supplier<Double> rotSpeedSupplier, Supplier<Boolean> fieldResetSupplier,
    Supplier<Boolean> toggleFieldCentricSupplier) {
    this.xSpeedSupplier = xSpeedSupplier;
    this.ySpeedSupplier = ySpeedSupplier;
    this.rotSpeedSupplier = rotSpeedSupplier;
    this.fieldResetSupplier = fieldResetSupplier;
    this.toggleFieldCentricSupplier = toggleFieldCentricSupplier;


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
    double xSpeedScaled = xSpeedDeadbanded * Drivetrain.SwerveConstants.TOP_SPEED;

    double ySpeed = ySpeedSupplier.get();
    double ySpeedDeadbanded = MathUtil.applyDeadband(ySpeed, 0.1);
    double ySpeedScaled = ySpeedDeadbanded * Drivetrain.SwerveConstants.TOP_SPEED;

    double rotSpeed = rotSpeedSupplier.get();
    double rotSpeedDeadbanded = MathUtil.applyDeadband(rotSpeed, 0.1);
    double rotSpeedScaled = rotSpeedDeadbanded * Drivetrain.SwerveConstants.TOP_ANGULAR_SPEED;

    drivetrain.move(xSpeedScaled, ySpeedScaled, rotSpeedScaled, fieldCentric, true);
    
    if(fieldResetSupplier.get()) {
      SNSR.navX.reset();;
    }
    if(toggleFieldCentricSupplier.get()) {
      fieldCentric = !fieldCentric;
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
