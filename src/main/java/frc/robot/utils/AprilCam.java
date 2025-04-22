package frc.robot.utils;


import frc.robot.Constants.VisionConstants;
import frc.robot.FieldConstants;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;


/** Add your docs here. */
public class AprilCam {

    private PhotonCamera camera;
    private Transform3d camOffset;
    private PhotonPoseEstimator photonPoseEstimator;
    private List<PhotonPipelineResult> results;


    // Simulation
    private Matrix<N3, N1> currentSDs;
    
    
    // Constructor 1
    public AprilCam(String name, Translation3d pos, Rotation3d angle){
        this.camera = new PhotonCamera(name);
        this.camOffset = new Transform3d(pos, angle);
        this.photonPoseEstimator = new PhotonPoseEstimator(FieldConstants.aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camOffset);
     }


    public void update() {
        this.results = camera.getAllUnreadResults();
    }

    // --------------------- POSE ESTIMATION ------------------------------- //
    // Check out this page: https://docs.photonvision.org/en/latest/docs/examples/poseest.html
    // Here is the example Vision class from PhotonVision: https://github.com/PhotonVision/photonvision/blob/main/photonlib-java-examples/poseest/src/main/java/frc/robot/Vision.java
 
   
    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
     * {@link getEstimationSDs}
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     *     used for estimation.
     */

    //https://github.com/PhotonVision/photonvision/blob/main/photonlib-java-examples/poseest/src/main/java/frc/robot/Vision.java 
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstPose) {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (PhotonPipelineResult result : this.results) {
            visionEst = photonPoseEstimator.update(result);
            updateEstimationSDs(visionEst, result.getTargets());
        }
        return visionEst;
    }


    /*
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     * Uses higher weights for reef tags!
     * This should only be used when there are targets visible
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
    private void updateEstimationSDs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {

        // No pose input. Default to single-tag std devs
        if (estimatedPose.isEmpty()) {
            currentSDs = VisionConstants.SINGLE_TAG_SD;
        } 
        
        // Pose present. Start running Heuristic
        else {
            var estimatedSDs = VisionConstants.SINGLE_TAG_SD;
            int numTags = 0;
            double totalDistance = 0;
            double totalWeight = 0;

            // Loop through all the targets to find difference in distance from current pose & how important that tag is
            for (var target : targets) {
                // Pose3d tagPose = FieldConstants.getTagPose(target.getFiducialId());
                // if(tagPose != null){
                var tagPose = photonPoseEstimator.getFieldTags().getTagPose(target.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                totalDistance += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
                totalWeight += FieldConstants.TAG_WEIGHTS[target.getFiducialId() - 1]; 
            }

            // Use the single tag standard deviations if no tags visible
            if (numTags == 0) {
                currentSDs = VisionConstants.SINGLE_TAG_SD;
            } 
            
            // One or more tags visible, run the full heuristic.
            else {

                // Calculate the average distance changes were
                double avgDist = totalDistance /numTags;
                double avgWeight = totalWeight /numTags;

                // Decrease standard deviations if multiple targets are visible
                if (numTags > 1) estimatedSDs = VisionConstants.MULTI_TAG_SD;
                
                // Increase standard deviations based on average distance
                if (numTags == 1 && avgDist > 4){
                    estimatedSDs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                } else {
                    estimatedSDs = estimatedSDs.times(1 + (avgDist * avgDist / 30));
                }

                // Weight reef tags higher
                estimatedSDs = estimatedSDs.times(avgWeight);
                currentSDs = estimatedSDs;
            }
        }
    }
     

    /*
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationSDs() {
        return currentSDs;
    }
}
