package frc.robot.commands.closed;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveAutoConstants;
import frc.robot.Field;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Drivetrain;


public class DriveToBranchPID extends Command {

  private static Drivetrain drivetrain;
  private static Camera cam;
  private int tagID;
  private String branchDirection;

  private double startDistanceX;
  private double startDistanceY;

  private PIDController controllerX;
  private PIDController controllerY;
  private double setpointX;
  private double setpointY;
  

  
  /** Creates a new DriveToBranchPID. */
  public DriveToBranchPID(int tagID, String branchDirection) {

    drivetrain = Drivetrain.getInstance();
    this.tagID = tagID;
    this.branchDirection = branchDirection;
    SmartDashboard.putString("DTBPID Branch", branchDirection);
    SmartDashboard.putNumber("DTBPID Tag", tagID);

    // Get desired pose at a specific branch
    Pose3d targetPose = Field.getBranchPose(tagID, branchDirection);

    //Record starting X & Y values
    startDistanceX = drivetrain.getPose().getX();
    startDistanceY = drivetrain.getPose().getY();

    // Record the setpoints for X & Y Coordinates
    setpointX = targetPose.getX();
    setpointY = targetPose.getY();

    // Setup PID controllers for X & Y distances
    controllerX = new PIDController(SwerveAutoConstants.DRIVE_X_D, SwerveAutoConstants.DRIVE_X_I, SwerveAutoConstants.DRIVE_X_D); //<--old values of 0.9, 0,0
    controllerY = new PIDController(SwerveAutoConstants.DRIVE_Y_P, SwerveAutoConstants.DRIVE_Y_I, SwerveAutoConstants.DRIVE_Y_D); //<--old values of 0.7, 0,0
    
    // Set setpoints for X & Y controllers
    controllerX.setSetpoint(setpointX);
    controllerY.setSetpoint(setpointY);

    // Set tolerances for X & Y controllers
    controllerX.setTolerance(0.05); //0.05m = 2 inches
    controllerY.setTolerance(0.01);


    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controllerX.reset();
    controllerY.reset();
    drivetrain.stopDrive(); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //measure current coordinats
    double currentDistanceX = drivetrain.getPose().getX();
    double currentDistanceY = drivetrain.getPose().getY();
    
    //calculating the X & Y speeds needed to strafe (field-centrically)
    double xSpeed = -controllerX.calculate(currentDistanceX);
    double ySpeed = -controllerY.calculate(currentDistanceY);

    //make robot strafe
    drivetrain.setDrive(xSpeed, ySpeed, 0.0, true);

    //SD stuff
    SmartDashboard.putNumber("DTBPID xSpeed", xSpeed);
    SmartDashboard.putNumber("DTBPID ySpeed", ySpeed);
    SmartDashboard.putNumber("DTBPID startX", startDistanceX);
    SmartDashboard.putNumber("DTBPID startY", startDistanceY);
    SmartDashboard.putNumber("DTBPID SetpointX", setpointX);
    SmartDashboard.putNumber("DTBPID SetpointY", setpointY);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopDrive();
    controllerX.close();
    controllerY.close();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controllerX.atSetpoint() && controllerY.atSetpoint();
  }
}
