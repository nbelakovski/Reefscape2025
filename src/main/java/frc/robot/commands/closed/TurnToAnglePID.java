package frc.robot.commands.closed;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;


public class TurnToAnglePID extends Command {

    private static Drivetrain drivetrain;
    private PIDController controllerTurn;
    private double setpoint;

  /** Creates a new TurnToAnglePID. */
  public TurnToAnglePID(double angle) {

    drivetrain = Drivetrain.getInstance();
    controllerTurn = new PIDController(Constants.SwerveAutoConstants.DRIVE_TURN_P, Constants.SwerveAutoConstants.DRIVE_TURN_I, Constants.SwerveAutoConstants.DRIVE_TURN_D);
    this.setpoint = angle;

    controllerTurn.setSetpoint(setpoint);
    controllerTurn.setTolerance(30,1); //<--values from 2022
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
    double measurement = drivetrain.getFieldAngleDegrees();
    
    //calculate new speed needed to turn to correct angle
    double newRotSpeed = controllerTurn.calculate(measurement);
    
    // OPTION 1: Turning Only in place
    boolean isFieldCentric = true;
    drivetrain.move(0, 0 , newRotSpeed, isFieldCentric); 

    //OPTION 2: //<--allows translation to also happen
    // drivetrain.setRotSpeed(newRotSpeed); 
    // drivetrain.drive();

    //SD stuff
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