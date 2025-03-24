package frc.robot;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;


public class TestCalculations {
    
    public static void main(String[] args){

        System.out.println("\nTesting for Poses!\n------------------------------");


        // Testing Variables
        Alliance alliance = Alliance.Blue;
        int startLocation = 2;
        String reefFace = "D";  //Blue D = 21
        String branchDirection = "LEFT";
        String stationDirection = "LEFT";
        

        // Print the starting point Pose coordinates (center of robot)
        Pose3d startPose = FieldConstants.getRobotPoseInitial(alliance, startLocation);
        System.out.printf("\nROBOT START POSE for " + alliance + " "+startLocation+":");
        FieldConstants.printPose3d(startPose);

        // Print the AprilTag Pose coordinates
        int tagID = FieldConstants.getTagFromReef(reefFace, alliance);
        Pose3d reefTagPose = FieldConstants.getTagPose(tagID);
        System.out.println("\n\nAPRIL TAG POSE for Reef tagID #" + tagID +":");
        FieldConstants.printPose3d(reefTagPose);

        // Print the Branch pose coordinates (center of robot)
        Pose3d targetPose = FieldConstants.getRobotPoseToBranch(tagID, branchDirection);
        System.out.println("\n\nROBOT POSE FACING BRANCH for "+ alliance + " - Face " + reefFace + " - " + branchDirection + " branch:");
        FieldConstants.printPose3d(targetPose);

        // Print the AprilTag Pose coordinates
        int csTagID = FieldConstants.getTagFromCoralStation(alliance, stationDirection);
        Pose3d csTagPose = FieldConstants.getTagPose(csTagID);
        System.out.println("\n\nAPRIL TAG POSE for Coral Station tagID #" + csTagID +":");
        FieldConstants.printPose3d(csTagPose);

        // Print the starting point Pose coordinates (center of robot)
        Pose3d coralStationPose = FieldConstants.getRobotPoseToCoralStation(alliance, stationDirection);
        System.out.println("\n\nROBOT POSE backing up to a Coral Station " + alliance + " "+ stationDirection+":");
        FieldConstants.printPose3d(coralStationPose);
    
        // Print the value of the closest tag
        double currentX = 2.0;
        double currentY = 4.0;
        Pose3d currentPose = new Pose3d( new Pose2d(currentX, currentY, new Rotation2d(0)));
        // currentPose = targetPose; 
        int closestTag = FieldConstants.getNearestReefTag(currentPose);
        String branchDirFromButton = "LEFT"; 
        System.out.println("Closest Tag is: " + closestTag);
        Pose3d closestBranchPose = FieldConstants.getRobotPoseToBranch(closestTag, branchDirFromButton);
        System.out.println("\n\nROBOT POSE FACING CLOSEST BRANCH from (" + currentX + "," + currentY + ") is at "+ alliance + " - tag " + closestTag + " - " + branchDirFromButton + " branch:");
        FieldConstants.printPose3d(closestBranchPose);


        System.out.println("\n");

        // System.out.println("3 inches in meters: "+Units.inchesToMeters(3));
        // System.out.println("Gear Ratio: " + Constants.SwerveModuleConstants.DRIVE_GEAR_RATIO);
        // System.out.println("Gear Reduction: " + Constants.SwerveModuleConstants.DRIVE_GEAR_REDUCTION);
    

    }


}
