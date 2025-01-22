// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import frc.robot.Constants.VisionConstants;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;


/** Add your docs here. */
public class AprilCam {

    private PhotonCamera camera;
    private PhotonPipelineResult result;



    private PhotonTrackedTarget desiredTarget;
    private AprilTagFieldLayout fieldLayout;
    private Transform3d camOffset;
    private PhotonPoseEstimator photonPoseEstimator;
    //private static AprilCam instance;
    
    
    // Constructor 1
    public AprilCam(String name, Translation3d position, Rotation3d angle){
        this.camera = new PhotonCamera(name);
     }

     // Constructor 2: simple version
    public AprilCam(String name) {
        this(name,new Translation3d(), new Rotation3d());
    }

    // //Singleton not needed if we want multiple Camera objects!
    // public static AprilCam getInstance() {
    //     if (instance == null) {
    //       instance = new AprilCam(VisionConstants.FRONT_CAM_NAME);
    //     }
    //     return instance;
    //   }

    // Updates the camera with the latest results (Needs to be called periodically!)
    public void update() {
        //var result = camera.getLatestResult();  // Query the latest result from PhotonVision //photonvision code  
        // this.result = camera.getAllUnreadResults().get(0); //old code
        this.result = camera.getLatestResult();
    }


    // --------------------- GETTING TARGETS ------------------------------- //

    // Checks if the latest result has any targets
    public boolean hasTarget() {
        return result.hasTargets();
    }

    //Gets all the AprilTag targets the camera can currently see
    public List<PhotonTrackedTarget> getTargets(){
        return result.getTargets();
    }

    // Gets the current "best" target
    public PhotonTrackedTarget getBestTarget(){
        return result.getBestTarget();
    }

    // Gets a target object for a specific AprilTag
    public PhotonTrackedTarget getDesiredTarget(int desiredTargetID){

        //look at each target in the arraylist of targets
        for (PhotonTrackedTarget t: getTargets())
        {
            //look for the target with the desired ID
            if (t.getFiducialId() == desiredTargetID)
            {
                return t;
            }
        }
        //return null if you can't find the desiredTarget
        return null;
    }

    // Checks if a desired AprilTag is visible
    public boolean hasDesiredTarget(int desiredTargetID) {
        ///use the getDesiredTarget method to see if it returns null (not correct target) or not
        if (getDesiredTarget(desiredTargetID)!= null)
        {
            return true;
        }
        return false;
    }

    // --------------------- GETTING DATA FROM A TARGET ------------------------------- //
    // https://docs.photonvision.org/en/v2025.1.1/docs/programming/photonlib/getting-target-data.html#getting-data-from-a-target
    // double yaw = target.getYaw();
    // double pitch = target.getPitch();
    // double area = target.getArea();
    // double skew = target.getSkew(); //not available for AprilTags
    // Transform2d pose = target.getCameraToTarget();
    // List<TargetCorner> corners = target.getCorners();
    // int targetID = target.getFiducialId();
    // double poseAmbiguity = target.getPoseAmbiguity();
    // Transform3d bestCameraToTarget = target.getBestCameraToTarget();
    // Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();

    // Gets the Transform3d object of a specific AprilTag object
    private Transform3d getTargetTransform(PhotonTrackedTarget target){
        if(target == null) {
            return null;
        }
        return target.getBestCameraToTarget();
    }

    // Gets the X value of a desired target
    public double getXDesired(PhotonTrackedTarget target){
        if(target == null) { return Float.NaN; }
        return getTargetTransform(target).getX();
    }

    // Gets the X value of the "Best" target
    public double getXBest(){
        return getXDesired( result.getBestTarget() );
    }
    
    // Gets the Y value of a desired target
    public double getYDesired(PhotonTrackedTarget target){
        if(target == null) { return Float.NaN; }
        return getTargetTransform(target).getY();
    }

    // Gets the Y value of the "Best" target
    public double getYBest(){
        return getYDesired( result.getBestTarget() );
    }

    // Gets the Z value of a desired target
    public double getZDesired(PhotonTrackedTarget target){
        if(target == null) { return Float.NaN; }
        return getTargetTransform(target).getZ();
    }

    // Gets the Z value of the "Best" target
    public double getZBest(){
        return getZDesired( result.getBestTarget() );
    }


    // --------------------- POSE ESTIMATION ------------------------------- //

    // Check out this page: https://docs.photonvision.org/en/latest/docs/examples/poseest.html
    // Here is the example Vision class from PhotonVision: https://github.com/PhotonVision/photonvision/blob/main/photonlib-java-examples/poseest/src/main/java/frc/robot/Vision.java
 
    // Note: Line 68 
    // Gets the AprilTag Locations:
    // visionSim.addAprilTags(kTagLayout);


    // Note: Lines 64-73
    // During periodic execution, we read back camera results. If we see AprilTags in the image, we calculate the camera-measured pose of the robot and pass it to the Drivetrain.

    // // Correct pose estimate with vision measurements
    // var visionEst = vision.getEstimatedGlobalPose();
    // visionEst.ifPresent(
    //         est -> {
    //             // Change our trust in the measurement based on the tags we can see
    //             var estStdDevs = vision.getEstimationStdDevs();

    //             drivetrain.addVisionMeasurement(
    //                     est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
    //         });




    //OLD CODE 
    // public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {

    //     return photonPoseEstimator.update(result);
    // }

    // public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {

    //     // only matters for CLOSEST_TO_REFERENCE_POSE strategy
    //     photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);

    //     return photonPoseEstimator.update(result);
    // }









    
    //method that gets the ID of the "best" target, generally not used by our robot
    // public String getAllTargets(){

    //     Optional<MultiTargetPNPResult> target = result.getMultiTagResult();

    //     return target.get().fiducialIDsUsed.toString();
    // }

   

}
