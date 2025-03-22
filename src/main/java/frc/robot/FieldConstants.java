package frc.robot;

import java.util.Arrays;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;

public class FieldConstants {

    public static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    public static Pose3d getTagPose(int tagID){
        return aprilTagFieldLayout.getTagPose(tagID).get();
    }

    public static Pose3d getBranchPose(int tagID, String branchDirection){

        // System.out.println("Branch\t" + branchDirection);
        // System.out.println("Tag\t" + tagID);

        AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

        Pose3d tagPose = aprilTagFieldLayout.getTagPose(tagID).get();
        // System.out.println("tagX\t"+tagPose.getX());
        // System.out.println("tagY\t"+tagPose.getY());
        

        //Get the coordinate of tag on field
        Translation3d tagTranslation = tagPose.getTranslation();
        double tagAngle = tagPose.getRotation().getAngle();
        // System.out.println("tagAngle\t" + tagAngle);

        //Create offset from tag to desired branch
        Translation3d branchOffset = new Translation3d();
        double branchOffsetDistanceInches = 6.5;
        double branchOffsetDistance = branchOffsetDistanceInches * 0.0254;
        // System.out.println("BranchOffsetDistanceinMeters\t" + branchOffsetDistance);

        if (branchDirection.equals("LEFT")){
        branchOffset = new Translation3d(Math.sin(tagAngle) * branchOffsetDistance, -Math.cos(tagAngle) * branchOffsetDistance, 0);
        }  else  if (branchDirection.equals("RIGHT")){
        branchOffset = new Translation3d(-Math.sin(tagAngle) * branchOffsetDistance, Math.cos(tagAngle) * branchOffsetDistance, 0);
        }

        // System.out.println("BO X\t"+ branchOffset.getX());
        // System.out.println("BO Y\t"+ branchOffset.getY());

        // Add tag and branch offset to get desired target coordinate --> target "Pose"
        Translation3d targetCoordinate = branchOffset.plus(tagTranslation);
        Rotation3d targetAngle = new Rotation3d(0,0,tagAngle).plus(new Rotation3d(0,0,Math.PI));  //targetAngle should be 180degrees from tag angle
        Pose3d targetPose = new Pose3d(targetCoordinate, targetAngle);
        // System.out.println("targetAngle\t"+targetAngle.getZ());
        
        return targetPose;
   

    }



    public static Pose2d getInitialPose(){

        var initialPose = new Pose2d();
        // Set our initial location and orientation based on alliance/location
        // Location 1 is Left Side (Same color barge)
        // location 2 is in the Middle (either alliance)
        // location 3 is Right Side/Processor (Opposite color barge)
        var locationOptional = DriverStation.getLocation();
        var allianceOptional = DriverStation.getAlliance();
        
        if (locationOptional.isPresent() && allianceOptional.isPresent()) {

            int location = locationOptional.getAsInt();
            var alliance = allianceOptional.get();

            // See dimensions in CAD model: https://cad.onshape.com/documents/73436e28519cd6ef4a2eaa1e/w/658277bde6080a5805de078e/e/42ebe4fcf42b8b1c992b38c2
            double FIELD_LENGTH_X = 17.548;
            double FIELD_WIDTH_Y = 8.052;
            double BLUE_CENTER_CAGE_Y = 6.169;
            double RED_CENTER_CAGE_Y = 1.883;
            double FIELD_CENTER_Y = FIELD_WIDTH_Y/2; //4.026
            double BLUE_STARTING_LINE = 6.742; //front of bumper when back edge of bumper is on back edge of starting line
            double RED_STARTING_LINE = FIELD_LENGTH_X - BLUE_STARTING_LINE;
            double BLUE_AUTO_ANGLE = 180;
            double RED_AUTO_ANGLE = 0;


            if (location == 1 && alliance == DriverStation.Alliance.Blue) {
                initialPose = new Pose2d(BLUE_STARTING_LINE, BLUE_CENTER_CAGE_Y, Rotation2d.fromDegrees(BLUE_AUTO_ANGLE));
            }
            else if (location == 2 && alliance == DriverStation.Alliance.Blue) {
                initialPose = new Pose2d(BLUE_STARTING_LINE, FIELD_CENTER_Y, Rotation2d.fromDegrees(BLUE_AUTO_ANGLE));
            }
            else if (location == 3 && alliance == DriverStation.Alliance.Blue) {
                initialPose = new Pose2d(BLUE_STARTING_LINE, RED_CENTER_CAGE_Y, Rotation2d.fromDegrees(BLUE_AUTO_ANGLE));
            }
            else if (location == 1 && alliance == DriverStation.Alliance.Red) {
                initialPose = new Pose2d(RED_STARTING_LINE, RED_CENTER_CAGE_Y, Rotation2d.fromDegrees(RED_AUTO_ANGLE));
            }
            else if (location == 2 && alliance == DriverStation.Alliance.Red) {
                initialPose = new Pose2d(RED_STARTING_LINE, FIELD_CENTER_Y, Rotation2d.fromDegrees(RED_AUTO_ANGLE));
            }
            else if (location == 3 && alliance == DriverStation.Alliance.Red) {
                initialPose = new Pose2d(RED_STARTING_LINE, BLUE_CENTER_CAGE_Y, Rotation2d.fromDegrees(RED_AUTO_ANGLE));
            }
        }

        return initialPose;
    }

    // Returns the nearest reef apriltag from an input pose
    // public static pose2d nearestReefApriltag(Pose2d pose) {
    //   return Arrays.stream(Face.values())
    //       .filter(
    //           face -> face.pose().minus(pose.nearest(poseList())).getTranslation().getNorm() < 1e-4)
    //       .findFirst()
    //       .orElse(AB);
    // }

    
}
