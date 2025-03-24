package frc.robot.subsystems;


import frc.robot.Constants.VisionConstants;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.utils.AprilCam;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Vision extends SubsystemBase {

  private static Vision instance;
  public AprilCam cam1;
  public int closestId;
  Drivetrain drivetrain = Drivetrain.getInstance();

  // Camera Constructor
  private Vision() {
    
    this.cam1 = new AprilCam(
      VisionConstants.CAM1_NAME, 
      new Translation3d(), 
      new Rotation3d(), 
      VisionConstants.CAM1_X_OFFSET_TO_CENTER, 
      VisionConstants.CAM1_Y_OFFSET_TO_CENTER
    );
    cam1.update();

  }

  // Camera Singleton - ensures only one Camera instance is constructed
  public static Vision getInstance(){
    if(instance == null){
      instance = new Vision();
    }
      return instance;
  }
  
  public PhotonTrackedTarget getDesiredTarget(int target) {
    return cam1.getDesiredTarget(target);
  }

  public double getXDesired(PhotonTrackedTarget target) {
    return cam1.getXDesired(target);
  }

  public  Pose3d getTagPose(int tagId){
    return FieldConstants.aprilTagFieldLayout.getTagPose(tagId).get();
  }
  
  public int getClosestId(){
    // return cam1.closestId;
    closestId = FieldConstants.getNearestReefTag(new Pose3d(drivetrain.getPose()));
    return closestId;
  }  


  // This method will be called once per scheduler run
  @Override
  public void periodic() {

    cam1.update();

    // Correct pose estimate with vision measurements
    var visionEst = cam1.getEstimatedGlobalPose(drivetrain.getPose());
    visionEst.ifPresent(
              est -> {
                // Change our trust in the measurement based on the tags we can see
                var estStdDevs = cam1.getEstimationStdDevs();

                drivetrain.addVisionMeasurement(
                        est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
            });

    if(visionEst.isPresent()) {
      SmartDashboard.putNumber("pose X", visionEst.get().estimatedPose.getX());
      SmartDashboard.putNumber("pose Y", visionEst.get().estimatedPose.getY());
      SmartDashboard.putNumber("rot", visionEst.get().estimatedPose.getRotation().getAngle());
      SmartDashboard.putNumber("tag x", getXDesired(getDesiredTarget(closestId)));
    }
    
    SmartDashboard.putNumber("tag 21 pose x", FieldConstants.aprilTagFieldLayout.getTagPose(21).get().getX());
    SmartDashboard.putNumber("tag 21 pose y", FieldConstants.aprilTagFieldLayout.getTagPose(21).get().getY());
    SmartDashboard.putNumber("tag 21 angle", FieldConstants.aprilTagFieldLayout.getTagPose(21).get().getRotation().getAngle());

    SmartDashboard.putNumber("closest Id", getClosestId());
    SmartDashboard.putNumber("CAM1 X offset to Front", Constants.VisionConstants.CAM1_X_OFFSET_TO_FRONT);
    SmartDashboard.putNumber("CAM1 X offset to Center", Constants.VisionConstants.CAM1_X_OFFSET_TO_CENTER);
    SmartDashboard.putNumber("CAM1 Bumper to Center Dist", Constants.RobotConstants.BUMPER_TO_ROBOT_CENTER_DISTANCE);
    
    
  }
}
