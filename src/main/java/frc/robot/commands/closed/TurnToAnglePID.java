package frc.robot.commands.closed;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveAutoConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ETechPoseEstimator;


public class TurnToAnglePID extends Command {

    private static Drivetrain drivetrain;
    private PIDController controllerTurn;
    private double setpoint;

    
  /** Creates a new TurnToAnglePID. */
  public TurnToAnglePID(double angle) {

    drivetrain = Drivetrain.getInstance();
    controllerTurn = new PIDController(SwerveAutoConstants.TURN_P, SwerveAutoConstants.TURN_I, SwerveAutoConstants.TURN_D);
    this.setpoint = angle;

    controllerTurn.setSetpoint(setpoint);
    controllerTurn.setTolerance(SwerveAutoConstants.TURN_TOL, SwerveAutoConstants.TURN_DERIV_TOL); //<--values from 2022
    controllerTurn.enableContinuousInput(-180, 180);

    addRequirements(drivetrain);
    
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controllerTurn.reset();
    // drivetrain.stopDrive();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //measure current angle
    double measurement = ETechPoseEstimator.getInstance().getPose().getRotation().getDegrees();
    
    //calculate new speed needed to turn to correct angle
    double newRotSpeed = controllerTurn.calculate(measurement);

    //SD stuff
    SmartDashboard.putNumber("TTA targetAngle", setpoint);
    SmartDashboard.putNumber("TTA newRotSpeed", newRotSpeed);
    SmartDashboard.putNumber("TTA measurement",measurement);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopDrive();
    controllerTurn.close();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controllerTurn.atSetpoint();
  }


}