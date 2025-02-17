// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/** Add your docs here. */
public class AprilCam {

    private PhotonCamera camera;
    private List<PhotonPipelineResult> results;

    private PhotonTrackedTarget desiredTarget;
    private PhotonPoseEstimator photonPoseEstimator;
    private List<PhotonTrackedTarget> targets;

    //private static AprilCam instance;
    Transform3d camOofset;
    AprilTagFieldLayout aprilTagFieldLayout;

    // Simulation
    private PhotonCameraSim cameraSim;
    private VisionSystemSim visionSim;

    private Matrix<N3, N1> curStdDevs;
    
    
    // Constructor 1
    public AprilCam(String name, Translation3d position, Rotation3d angle){
        this.camera = new PhotonCamera(name);
         //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        this.camOofset = new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0,0,0));
        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
        this.photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camOofset);
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
    // public void update() {
    //     result = new PhotonPipelineResult();
        // SmartDashboard.putBoolean("photon result", camera.getAllUnreadResults().isEmpty());
        // SmartDashboard.putNumber("photon result size", camera.getAllUnreadResults().size());
        //SmartDashboard.putString("pipeline", camera.getAllUnreadResults().get(0).toString());
    
        // if(camera.getAllUnreadResults().size() != 0) {
        //     SmartDashboard.putBoolean("is run", true);

        //     //this.result = camera.getAllUnreadResults().get(0);
        // }
        //var result = camera.getLatestResult();  // Query the latest result from PhotonVision //photonvision code  
        //this.result = camera.getAllUnreadResults().get(0); //old code
        //this.result = camera.getLatestResult();
    //}

    public void update() {
        this.results = camera.getAllUnreadResults();
    }

    // --------------------- GETTING TARGETS -------------------)------------ //

    // Checks if the latest result has any targets
    // public boolean hasTarget() {
    //     return result.hasTargets();
    // }

    //Gets all the AprilTag targets the camera can currently see
    public List<PhotonTrackedTarget> getTargets(){
        return targets;
    }

    public int getClosestID() {
        int closestID = -1;
        double closestDistance = 100;
        for(PhotonTrackedTarget t: targets) {
            double currentDistance = Math.sqrt(Math.pow(getTargetTransform(t).getX(), 2) + Math.pow(getTargetTransform(t).getY(), 2));
            
            if(currentDistance < closestDistance) {
                closestDistance = currentDistance;
                closestID = t.fiducialId;
            }

        }
        return closestID;
    }

    // Gets the current "best" target
    // public PhotonTrackedTarget getBestTarget(){
    //     return result.getBestTarget();
    // }

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
    // public boolean hasDesiredTarget(int desiredTargetID) {
    //     ///use the getDesiredTarget method to see if it returns null (not correct target) or not
    //     if (getDesiredTarget(desiredTargetID)!= null)
    //     {
    //         return true;
    //     }
    //     return false;
    // }

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

    // // Gets the X value of a desired target
    public double getXDesired(PhotonTrackedTarget target){
        if(target == null) { return Float.NaN; }
        return getTargetTransform(target).getX();
    }

    // // Gets the X value of the "Best" target
    // public double getXBest(){
    //     return getXDesired( results.getBestTarget() );
    // }
    
    // // Gets the Y value of a desired target
    // public double getYDesired(PhotonTrackedTarget target){
    //     if(target == null) { return Float.NaN; }
    //     return getTargetTransform(target).getY();
    // }

    // // Gets the Y value of the "Best" target
    // public double getYBest(){
    //     return getYDesired( results.getBestTarget() );
    // }

    // // Gets the Z value of a desired target
    // public double getZDesired(PhotonTrackedTarget target){
    //     if(target == null) { return Float.NaN; }
    //     return getTargetTransform(target).getZ();
    // }

    // // Gets the Z value of the "Best" target
    // public double getZBest(){
    //     return getZDesired( results.getBestTarget() );
    // }


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

    // public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    //     photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    //     return photonPoseEstimator.update(result);
    // }

    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
     * {@link getEstimationStdDevs}
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     *     used for estimation.
     */

    //https://github.com/PhotonVision/photonvision/blob/main/photonlib-java-examples/poseest/src/main/java/frc/robot/Vision.java 
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstPose) {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();

        //photonPoseEstimator.setReferencePose(prevEstPose);

        for (var change : this.results) {
            visionEst = photonPoseEstimator.update(change);
            updateEstimationStdDevs(visionEst, change.getTargets());
            targets = change.getTargets();
            

        }
        return visionEst;
    }
    /*
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
     private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = VisionConstants.kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = VisionConstants.kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = photonPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = VisionConstants.kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }
     

    /*
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }
    /** A Field2d for visualizing our robot and objects on the field. */
    public Field2d getSimDebugField() {
        if (!Robot.isSimulation()) return null;
        return visionSim.getDebugField();
    }

    
    //method that gets the ID of the "best" target, generally not used by our robot
    // public String getAllTargets(){

    //     Optional<MultiTargetPNPResult> target = result.getMultiTagResult();

    //     return target.get().fiducialIDsUsed.toString();
    // }

   

}
