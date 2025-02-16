// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.VisionConstants;
import frc.robot.utils.AprilCam;
import frc.robot.utils.AprilCamDep;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class Camera extends SubsystemBase {

  public AprilCam cam;
  private static Camera instance;
  // The field from AprilTagFields will be different depending on the game.
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();

  public int closestID;

  //Forward Camera
  // cam = new PhotonCamera("testCamera");
  Transform3d robotToCam;
  // Construct PhotonPoseEstimator
  PhotonPoseEstimator photonPoseEstimator;

  // Construct Drivetrain Instance
  Drivetrain drivetrain = Drivetrain.getInstance();

  // Constructor
  private Camera() {
    this.cam = new AprilCam(VisionConstants.FRONT_CAM_NAME);
    // this.robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    // this.photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);

    //cam.update();
  }

  // Singleton Constructor
  public static Camera getInstance(){
    if(instance == null){
      instance = new Camera();
    }
      return instance;
  }
  
  // public AprilCam getCam() {
  //   return cam;
  // }

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
    return cam.getClosestID();
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
    for (int i = 0; i < cam.getTargets().size(); i++) {
      SmartDashboard.putString("id" + i, cam.getTargets().get(i).toString());
    }
    closestID = getClosestID();

    if(visionEst.isPresent()) {
      SmartDashboard.putNumber("pose X", visionEst.get().estimatedPose.getX());
      SmartDashboard.putNumber("pose Y", visionEst.get().estimatedPose.getY());
      SmartDashboard.putNumber("rot", visionEst.get().estimatedPose.getRotation().getAngle());
    }
    // else {
    //   SmartDashboard.putNumber("pose X", 0);
    //   SmartDashboard.putNumber("pose Y", 0);
    //   SmartDashboard.putNumber("rot", 0);
    // }
    
    SmartDashboard.putNumber("tag 10 pose x", aprilTagFieldLayout.getTagPose(10).get().getX());
    SmartDashboard.putNumber("tag 10 pose y", aprilTagFieldLayout.getTagPose(10).get().getY());
    SmartDashboard.putNumber("closest ID", closestID);
    
   
    
  }
}
