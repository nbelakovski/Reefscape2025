package frc.robot.subsystems;


import frc.robot.Constants.VisionConstants;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.SNSR;
import frc.robot.utils.AprilCam;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ETechPoseEstimator extends SubsystemBase {

  private static ETechPoseEstimator instance;
  public AprilCam cam1;
  public AprilCam cam2;
  public boolean doubleCam = false;
  Drivetrain drivetrain = Drivetrain.getInstance();
  private final Field2d field2d = new Field2d();

  // Vision Constructor
  private ETechPoseEstimator() {
    
    // Construct each AprilCam
    this.cam1 = new AprilCam(
      VisionConstants.CAM1_NAME, 
      VisionConstants.CAM1_POSITION_OFFSET, 
      VisionConstants.CAM1_ANGLE_OFFSET
    );
    
    // Option to add 2nd camera
    if(doubleCam){
      this.cam2 = new AprilCam(
        VisionConstants.CAM2_NAME, 
        VisionConstants.CAM2_POSITION_OFFSET, 
        VisionConstants.CAM2_ANGLE_OFFSET
      );
    }

  }

  // Camera Singleton - ensures only one Camera instance is constructed
  public static ETechPoseEstimator getInstance(){
    if(instance == null){
      instance = new ETechPoseEstimator();
    }
    return instance;
  }

  public Pose2d getPose() {
    return drivetrain.poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d newPose) {
    drivetrain.poseEstimator.resetPosition(SNSR.navX.getRotation2d(), drivetrain.getSwerveModulePos(), newPose);
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    drivetrain.poseEstimator.update(SNSR.navX.getRotation2d(), drivetrain.getSwerveModulePos());

    // Correct pose estimate with vision measurements
    var visionEst1 = cam1.getEstimatedGlobalPose(getPose());
    visionEst1.ifPresent(
      est -> {
        // Change our trust in the measurement based on the tags we can see
        var estimatedSDs = cam1.getEstimationSDs();

        drivetrain.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estimatedSDs);
      }
    );
    if(visionEst1.isPresent()) {
      SmartDashboard.putNumber("cam1 poseX", visionEst1.get().estimatedPose.getX());
      SmartDashboard.putNumber("cam1 poseY", visionEst1.get().estimatedPose.getY());
      SmartDashboard.putNumber("cam1 poseRot", visionEst1.get().estimatedPose.getRotation().getAngle());
    }

    if(doubleCam){
      var visionEst2 = cam2.getEstimatedGlobalPose(getPose());
      visionEst2.ifPresent(
        est -> {
          var estimatedSDs = cam2.getEstimationSDs();
          drivetrain.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estimatedSDs);
        }
      );
      if(visionEst2.isPresent()) {
        SmartDashboard.putNumber("cam2 poseX", visionEst2.get().estimatedPose.getX());
        SmartDashboard.putNumber("cam2 poseY", visionEst2.get().estimatedPose.getY());
        SmartDashboard.putNumber("cam2 poseRot", visionEst2.get().estimatedPose.getRotation().getAngle());
      }
    }

    int tagId = FieldConstants.getNearestReefTag(new Pose3d(getPose()));

    if(tagId > 0) {
      SmartDashboard.putNumber("tag " + tagId + " pose x", FieldConstants.aprilTagFieldLayout.getTagPose(tagId).get().getX());
      SmartDashboard.putNumber("tag " + tagId + " pose y", FieldConstants.aprilTagFieldLayout.getTagPose(tagId).get().getY());
      SmartDashboard.putNumber("tag " + tagId + " angle", FieldConstants.aprilTagFieldLayout.getTagPose(tagId).get().getRotation().getAngle());
    }

    SmartDashboard.putNumber("CAM1 X offset to Front", Constants.VisionConstants.CAM1_X_OFFSET_TO_FRONT);
    SmartDashboard.putNumber("CAM1 X offset to Center", Constants.VisionConstants.CAM1_X_OFFSET_TO_CENTER);
    SmartDashboard.putNumber("CAM1 Bumper to Center Dist", Constants.RobotConstants.BUMPER_TO_ROBOT_CENTER_DISTANCE);

    SmartDashboard.putNumber("Robot Angle Degrees", SNSR.navX.getRotation2d().getDegrees());
    SmartDashboard.putNumber("Robot Angle Radians", SNSR.navX.getRotation2d().getRadians());

    SmartDashboard.putNumber("PoseX", getPose().getX());
    SmartDashboard.putNumber("PoseY", getPose().getY());
    SmartDashboard.putNumber("PoseAngle Degrees", getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("PoseAngle Radians", getPose().getRotation().getRadians());
    field2d.setRobotPose(getPose());
    SmartDashboard.putData("PoseEstimator Field", field2d);
  }
}
