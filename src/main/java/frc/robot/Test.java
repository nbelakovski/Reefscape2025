package frc.robot;


import edu.wpi.first.math.geometry.Pose3d;


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

    }


}
