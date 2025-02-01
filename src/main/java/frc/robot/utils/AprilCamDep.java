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
public class AprilCamDep {

    private PhotonCamera camera;
    private PhotonPipelineResult result;

    private PhotonTrackedTarget desiredTarget;
    private PhotonPoseEstimator photonPoseEstimator;

    //private static AprilCam instance;
    Transform3d camOofset;
    AprilTagFieldLayout aprilTagFieldLayout;

    // Simulation
    private PhotonCameraSim cameraSim;
    private VisionSystemSim visionSim;

    private Matrix<N3, N1> curStdDevs;
    
    
    // Constructor 1
    public AprilCamDep(String name, Translation3d position, Rotation3d angle){
        this.camera = new PhotonCamera(name);
         //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        this.camOofset = new Transform3d(new Translation3d(0.3683, 0.0, 0.0), new Rotation3d(0,0,0));
        aprilTagFieldLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();
        this.photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camOofset);
     }

     public AprilCamDep(String name) {
        this(name,new Translation3d(), new Rotation3d());
    }

     // Updates the camera with the latest results (Needs to be called periodically!)
    public void update() {
        this.result = camera.getLatestResult();
    }

     public boolean hasTarget() {
        return result.hasTargets();
     }

     //Gets all the AprilTag targets the camera can currently see
    public List<PhotonTrackedTarget> getTargets(){
        return result.getTargets();
    }

     //Gets the current "best" target
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

    // // Gets the X value of a desired target
    public double getXDesired(PhotonTrackedTarget target){
        if(target == null) { return Float.NaN; }
        return getTargetTransform(target).getX();
    }

    // // Gets the X value of the "Best" target
    public double getXBest(){
        return getXDesired( result.getBestTarget() );
    }
    
    // // Gets the Y value of a desired target
    public double getYDesired(PhotonTrackedTarget target){
        if(target == null) { return Float.NaN; }
        return getTargetTransform(target).getY();
    }

    // // Gets the Y value of the "Best" target
    public double getYBest(){
        return getYDesired( result.getBestTarget() );
    }

    // // Gets the Z value of a desired target
    public double getZDesired(PhotonTrackedTarget target){
        if(target == null) { return Float.NaN; }
        return getTargetTransform(target).getZ();
    }

    // // Gets the Z value of the "Best" target
    public double getZBest(){
        return getZDesired( result.getBestTarget() );
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstPose) {

        Optional<EstimatedRobotPose> visionEst = Optional.empty();

        photonPoseEstimator.setReferencePose(prevEstPose);
        visionEst = photonPoseEstimator.update(result);
        updateEstimationStdDevs(visionEst, getTargets());

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

    }