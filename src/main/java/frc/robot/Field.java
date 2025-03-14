package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public class Field {

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
    
}
