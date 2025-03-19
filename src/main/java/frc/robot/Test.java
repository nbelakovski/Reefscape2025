package frc.robot;


import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;


public class Test {
    
    public static void main(String[] args){

        System.out.println("Testing the getBranchPose method!");

        int tagID = 21;
        String branchDirection = "LEFT";
        Pose3d targetPose = Field.getBranchPose(tagID, branchDirection);

        // Record the setpoints for X & Y Coordinates
        System.out.println("tagX\t" + Field.getTagPose(tagID).getX());
        System.out.println("tagY\t" + Field.getTagPose(tagID).getY());
        System.out.println("tagAngle\t" + Math.toDegrees(Field.getTagPose(tagID).getRotation().getZ()));
        
        System.out.println("setpointX\t" + targetPose.getX());
        System.out.println("setpointY\t" + targetPose.getY());
        System.out.println("setpointAngle\t" + Math.toDegrees(targetPose.getRotation().getZ()));

        System.out.println("3 inches in meters: "+Units.inchesToMeters(3));
        System.out.println("Gear Ratio: " + Constants.SwerveModuleConstants.DRIVE_GEAR_RATIO);
        System.out.println("Gear Reduction: " + Constants.SwerveModuleConstants.DRIVE_GEAR_REDUCTION);
        

    }


}
