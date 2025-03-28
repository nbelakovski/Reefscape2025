package frc.robot.subsystems;


import frc.robot.Constants.VisionConstants;
import frc.robot.FieldConstants;
import frc.robot.utils.AprilCam;

import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Vision extends SubsystemBase {

  private static Vision instance;
  public AprilCam cam1;
  public AprilCam cam2;
  public boolean doubleCam = false;
  public int closestID;
  Drivetrain drivetrain = Drivetrain.getInstance();

  // Vision Constructor
  private Vision() {
    
    // Construct each AprilCam
    this.cam1 = new AprilCam(
      VisionConstants.CAM1_NAME, 
      VisionConstants.CAM1_POSITION_OFFSET, 
      VisionConstants.CAM1_ANGLE_OFFSET
    );
    
    // Update the cameras
    cam1.update();
    
    // Option to add 2nd camera
    if(doubleCam){
      this.cam2 = new AprilCam(
        VisionConstants.CAM2_NAME, 
        VisionConstants.CAM2_POSITION_OFFSET, 
        VisionConstants.CAM2_ANGLE_OFFSET
      );
      cam2.update();  
    }
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

  public  Pose3d getTagPose(int tagID){
    return FieldConstants.aprilTagFieldLayout.getTagPose(tagID).get();
  }
  
  public int getClosestID(){
    return cam1.closestID;
  }  


  // This method will be called once per scheduler run
  @Override
  public void periodic() {

    cam1.update();
    // Correct pose estimate with vision measurements
    var visionEst1 = cam1.getEstimatedGlobalPose(drivetrain.getPose());
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
      cam2.update();
      var visionEst2 = cam2.getEstimatedGlobalPose(drivetrain.getPose());
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

    SmartDashboard.putNumber("tag x", getXDesired(getDesiredTarget(closestID)));
    SmartDashboard.putNumber("tag 21 pose x", FieldConstants.aprilTagFieldLayout.getTagPose(21).get().getX());
    SmartDashboard.putNumber("tag 21 pose y", FieldConstants.aprilTagFieldLayout.getTagPose(21).get().getY());
    SmartDashboard.putNumber("tag 21 angle", FieldConstants.aprilTagFieldLayout.getTagPose(21).get().getRotation().getAngle());
    SmartDashboard.putNumber("closest ID", closestID);
    
  }
}
