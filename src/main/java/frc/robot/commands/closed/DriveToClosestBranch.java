package frc.robot.commands.closed;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveAutoConstants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ETechPoseEstimator;


public class DriveToClosestBranch extends Command {

  private static Drivetrain drivetrain;

  private double startX;
  private double startY;
  private double startAngle;

  private PIDController controllerX;
  private PIDController controllerY;
  private PIDController controllerTurn;

  private double setpointX;
  private double setpointY;
  private double setpointTurn;
  int tagId;
  Pose3d targetPose;
  String branchDirection;
  
  
  /** DriveToBranchPID Constructor #1 - takes in a specific AprilTag & Branch direction  */
  public DriveToClosestBranch(String branchDirection) {
    this.branchDirection = branchDirection;
    drivetrain = Drivetrain.getInstance();
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.stopDrive(); 


    tagId = FieldConstants.getNearestReefTag(new Pose3d(ETechPoseEstimator.getInstance().getPose()));
    targetPose = FieldConstants.getRobotPoseToBranch(tagId, branchDirection);

    //Record starting values for X (m), Y (m), Angle (deg)
    startX = ETechPoseEstimator.getInstance().getPose().getX();
    startY = ETechPoseEstimator.getInstance().getPose().getY();
    startAngle = ETechPoseEstimator.getInstance().getPose().getRotation().getDegrees();

    // Record the setpoints for X (m), Y (m), Angle (deg)
    setpointX = targetPose.getX();
    setpointY = targetPose.getY();
    setpointTurn = Math.toDegrees(targetPose.getRotation().getZ());

    // Setup PID controllers for X & Y distances
    controllerX = new PIDController(SwerveAutoConstants.X_P, SwerveAutoConstants.X_I, SwerveAutoConstants.X_D);
    controllerY = new PIDController(SwerveAutoConstants.Y_P, SwerveAutoConstants.Y_I, SwerveAutoConstants.Y_D); 
    controllerTurn = new PIDController(SwerveAutoConstants.TURN_P, SwerveAutoConstants.TURN_I, SwerveAutoConstants.TURN_D);

    // Set setpoints for X & Y controllers
    controllerX.setSetpoint(setpointX);
    controllerY.setSetpoint(setpointY);
    controllerTurn.setSetpoint(setpointTurn);

    // Set tolerances for X & Y controllers
    controllerX.setTolerance(SwerveAutoConstants.X_TOL);
    controllerY.setTolerance(SwerveAutoConstants.Y_TOL);
    controllerTurn.setTolerance(SwerveAutoConstants.TURN_TOL,SwerveAutoConstants.TURN_DERIV_TOL);
    controllerTurn.enableContinuousInput(-180, 180);

    SmartDashboard.putString("DTB Branch", branchDirection);
    SmartDashboard.putNumber("DTB Tag", tagId);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //measure current X (m), Y (m), Angle (deg)
    double currentX = ETechPoseEstimator.getInstance().getPose().getX();
    double currentY = ETechPoseEstimator.getInstance().getPose().getY();
    double currentAngle = ETechPoseEstimator.getInstance().getPose().getRotation().getDegrees();
    
    //calculating the X, Y, TURN speeds needed to strafe (field-centrically)
    double xSpeed = -controllerX.calculate(currentX);
    double ySpeed = controllerY.calculate(currentY);
    double turnSpeed = controllerTurn.calculate(currentAngle);

    // Set to 0 for isolation testing
    // xSpeed=0;
    // ySpeed=0;
     //turnSpeed=0;

    //make robot move
    if(controllerX.getError() < 0.1 && controllerY.getError() < 0.1) {
      controllerX.setP(SwerveAutoConstants.X_P * 2);
      controllerY.setP(SwerveAutoConstants.Y_P * 2);
    }
    Rotation2d rotation = ETechPoseEstimator.getInstance().getPose().getRotation();
    drivetrain.move(xSpeed, ySpeed, turnSpeed, true, false, rotation);

    //SD stuff
    SmartDashboard.putNumber("DTB xSpeed", xSpeed);
    SmartDashboard.putNumber("DTB ySpeed", ySpeed);
    SmartDashboard.putNumber("DTB turnSpeed", turnSpeed);

    SmartDashboard.putNumber("DTB startX", startX);
    SmartDashboard.putNumber("DTB startY", startY);
    SmartDashboard.putNumber("DTB startTurn", startAngle);

    SmartDashboard.putNumber("DTB SetpointX", setpointX);
    SmartDashboard.putNumber("DTB SetpointY", setpointY);
    SmartDashboard.putNumber("DTB SetpointAngle", setpointTurn);

    SmartDashboard.putBoolean("DTB Xatsp",controllerX.atSetpoint());
    SmartDashboard.putBoolean("DTB Yatsp",controllerY.atSetpoint());
    SmartDashboard.putBoolean("DTB Turnatsp",controllerTurn.atSetpoint());
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopDrive();
    controllerX.close();
    controllerY.close();
    controllerTurn.close();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return controllerX.atSetpoint() && controllerY.atSetpoint() && controllerTurn.atSetpoint();
    return false;
  }
}
