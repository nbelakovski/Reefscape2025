package frc.robot;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;


public class TestCalculations {
    
    public static void main(String[] args){

        System.out.println("\nTesting for Poses!\n------------------------------");


        // Testing Variables
        Alliance alliance = Alliance.Blue;
        int startLocation = 2;
        String reefFace = "E";  //Blue D = 21
        String branchDirection = "RIGHT";
        String stationDirection = "LEFT";
        

        // Print the starting point Pose coordinates (center of robot)
        Pose2d startPose = FieldConstants.getRobotPoseInitial(alliance, startLocation);
        System.out.printf("\nROBOT START POSE for " + alliance + " "+startLocation+":");
        System.out.printf("\nstartX\t%.3f",startPose.getX());
        System.out.printf("\nstartY\t%.3f", startPose.getY());
        System.out.printf("\nstartAngle\t%.1f",startPose.getRotation().getDegrees());

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
    
        System.out.println("\n");

        // System.out.println("3 inches in meters: "+Units.inchesToMeters(3));
        // System.out.println("Gear Ratio: " + Constants.SwerveModuleConstants.DRIVE_GEAR_RATIO);
        // System.out.println("Gear Reduction: " + Constants.SwerveModuleConstants.DRIVE_GEAR_REDUCTION);
    

    }


}
