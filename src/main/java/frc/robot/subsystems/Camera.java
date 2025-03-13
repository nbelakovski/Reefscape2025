package frc.robot.subsystems;


import frc.robot.Constants.VisionConstants;
import frc.robot.utils.AprilCam;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Camera extends SubsystemBase {

  private static Camera instance;
  public AprilCam cam;
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  public int closestID;
  Transform3d robotToCam;
  PhotonPoseEstimator photonPoseEstimator;
  Drivetrain drivetrain = Drivetrain.getInstance();

  // Camera Constructor
  private Camera() {
    this.cam = new AprilCam(VisionConstants.FRONT_CAM_NAME);
    // this.robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    // this.photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
    //cam.update();
  }

  // Camera Singleton - ensures only one Camera instance is constructed
  public static Camera getInstance(){
    if(instance == null){
      instance = new Camera();
    }
      return instance;
  }
  
  public PhotonTrackedTarget getDesiredTarget(int target) {
    return cam.getDesiredTarget(target);
  }

  public double getXDesired(PhotonTrackedTarget target) {
    return cam.getXDesired(target);
  }

  public  Pose3d getTagPose(int tagID){
    return aprilTagFieldLayout.getTagPose(tagID).get();
  }

  // public double getX(){
  //   return cam.getXBest();
  // }

  // public double getYDesired(PhotonTrackedTarget target) {
  //   return cam.getYDesired(target);
  // }

  // public double getY(){
  //   return cam.getYBest();
  // }

  // public double getZ(){
  //   return cam.getZBest();
  // }
  // public boolean hasTarget() {
  //   return cam.hasTarget();
  // }
  
  public int getClosestID(){
    return cam.closestID;
  }
    

  @Override
  public void periodic() {

    cam.update();

  
    // This method will be called once per scheduler run
    // Correct pose estimate with vision measurements
    var visionEst = cam.getEstimatedGlobalPose(drivetrain.getPose());
    visionEst.ifPresent(
            est -> {
                // Change our trust in the measurement based on the tags we can see
                var estStdDevs = cam.getEstimationStdDevs();

                drivetrain.addVisionMeasurement(
                        est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
            });

    // SmartDashboard.putNumber("X", getX());
    // SmartDashboard.putNumber("Y", getY());
    // SmartDashboard.putNumber("Z", getZ());
    // for (int i = 0; i < cam.getTargets().size(); i++) {
    //   SmartDashboard.putString("id" + i, cam.getTargets().get(i).toString());
    // }

    if(visionEst.isPresent()) {
      SmartDashboard.putNumber("pose X", visionEst.get().estimatedPose.getX());
      SmartDashboard.putNumber("pose Y", visionEst.get().estimatedPose.getY());
      SmartDashboard.putNumber("rot", visionEst.get().estimatedPose.getRotation().getAngle());
      SmartDashboard.putNumber("tag x", getXDesired(getDesiredTarget(closestID)));
    }
    // else {
    //   SmartDashboard.putNumber("pose X", 0);
    //   SmartDashboard.putNumber("pose Y", 0);
    //   SmartDashboard.putNumber("rot", 0);
    // }
    
    SmartDashboard.putNumber("tag 21 pose x", aprilTagFieldLayout.getTagPose(21).get().getX());
    SmartDashboard.putNumber("tag 21 pose y", aprilTagFieldLayout.getTagPose(21).get().getY());
    SmartDashboard.putNumber("closest ID", closestID);
    
   
    
  }
}
