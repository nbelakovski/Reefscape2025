package frc.robot;


import java.util.Optional;
import java.util.OptionalInt;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants;


public class FieldConstants {

    public static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    public static final int[] REEF_TAGS = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
    public static final double[] TAG_WEIGHTS = { 0.25, 0.25, 0.25, 0.25, 0.25, 1, 1, 1, 1, 1, 1, 0.25, 0.25, 0.25, 0.25, 0.25, 1, 1, 1, 1, 1, 1};

    // See dimensions in CAD model: https://cad.onshape.com/documents/73436e28519cd6ef4a2eaa1e/w/658277bde6080a5805de078e/e/42ebe4fcf42b8b1c992b38c2
    public static final double FIELD_LENGTH_X = 17.548;
    public static final double FIELD_WIDTH_Y = 8.052;
    public static final double BLUE_CENTER_CAGE_Y = 6.169;
    public static final double RED_CENTER_CAGE_Y = 1.883;
    public static final double FIELD_CENTER_Y = FIELD_WIDTH_Y/2; //4.026
    public static final double BLUE_STARTING_LINE = 7.606; //"back" edge of 2" starting line
    public static final double BLUE_STARTING_X = BLUE_STARTING_LINE - RobotConstants.BUMPER_TO_ROBOT_CENTER_DISTANCE;  //7.174 Center of Robot when Starting
    public static final double RED_STARTING_X = FIELD_LENGTH_X - BLUE_STARTING_X;
    public static final double BLUE_AUTO_ANGLE = 180;
    public static final double RED_AUTO_ANGLE = 0;
    public static final double FIELD_LENGTH_X = 17.548;
    public static final double FIELD_WIDTH_Y = 8.052;
    public static final double BLUE_CENTER_CAGE_Y = 6.169;
    public static final double RED_CENTER_CAGE_Y = 1.883;
    public static final double FIELD_CENTER_Y = FIELD_WIDTH_Y/2; //4.026
    public static final double BLUE_STARTING_LINE = 7.606; //"back" edge of 2" starting line
    public static final double BLUE_STARTING_X = BLUE_STARTING_LINE - RobotConstants.BUMPER_TO_ROBOT_CENTER_DISTANCE;  //7.174 Center of Robot when Starting
    public static final double RED_STARTING_X = FIELD_LENGTH_X - BLUE_STARTING_X;
    public static final double BLUE_AUTO_ANGLE = 180;
    public static final double RED_AUTO_ANGLE = 0;

    public static final Translation2d fieldCenter = new Translation2d(FIELD_LENGTH_X/2, FIELD_WIDTH_Y/2);
    public static final Pose2d blueCoralStationLeft = new Pose2d(Inches.of(33.526), Inches.of(291.176), Rotation2d.fromDegrees(90 - 144.011));
    public static final Pose2d blueCoralStationRight = new Pose2d(Inches.of(33.526), Inches.of(25.824), Rotation2d.fromDegrees(144.011 - 90));
    public static final Pose2d blueBargeSideAuto = new Pose2d(BLUE_STARTING_LINE, BLUE_CENTER_CAGE_Y, Rotation2d.fromDegrees(BLUE_AUTO_ANGLE));
    public static final Pose2d blueCenterAuto = new Pose2d(BLUE_STARTING_LINE, FIELD_CENTER_Y, Rotation2d.fromDegrees(BLUE_AUTO_ANGLE));
    public static final Pose2d blueProcessorSideAuto = new Pose2d(BLUE_STARTING_LINE, RED_CENTER_CAGE_Y, Rotation2d.fromDegrees(BLUE_AUTO_ANGLE));
    public static final Pose2d redCoralStationLeft = blueCoralStationLeft.rotateAround(fieldCenter, new Rotation2d(Degrees.of(180)) );
    public static final Pose2d redCoralStationRight = blueCoralStationRight.rotateAround(fieldCenter, new Rotation2d(Degrees.of(180)) );
    public static final Pose2d redBargeSideAuto = new Pose2d(RED_STARTING_LINE, RED_CENTER_CAGE_Y, Rotation2d.fromDegrees(RED_AUTO_ANGLE));
    public static final Pose2d redCenterAuto = new Pose2d(RED_STARTING_LINE, FIELD_CENTER_Y, Rotation2d.fromDegrees(RED_AUTO_ANGLE));
    public static final Pose2d redProcessorSideAuto = new Pose2d(RED_STARTING_LINE, BLUE_CENTER_CAGE_Y, Rotation2d.fromDegrees(RED_AUTO_ANGLE));


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


    // Set our initial location and orientation based on alliance/location
    // Location 1 is Left Side (Barge)
    // location 2 is in the Middle
    // location 3 is Right Side (Processor)
    public static Pose2d getInitialPose(){

        Optional<Alliance> allianceOptional = DriverStation.getAlliance();
        Alliance alliance = null;
        if (allianceOptional.isPresent()) {
            alliance = allianceOptional.get();
        }    

        OptionalInt locationOptional = DriverStation.getLocation();
        int location = -1;        
        if(locationOptional.isPresent()){
            location = locationOptional.getAsInt();
        }

        return getRobotPoseInitial(alliance, location);
        return getRobotPoseInitial(alliance, location);
    }
    
    public static Pose2d getInitialPose(Alliance alliance, int location){
        
        if (location == 1 && alliance == DriverStation.Alliance.Blue) { return blueBargeSideAutoPose; }
        else if (location == 2 && alliance == DriverStation.Alliance.Blue) { return blueCenterAutoPose; }
        else if (location == 3 && alliance == DriverStation.Alliance.Blue) { return blueProcessorSideAutoPose; }
        else if (location == 1 && alliance == DriverStation.Alliance.Red) { return redBargeSideAutoPose; }
        else if (location == 2 && alliance == DriverStation.Alliance.Red) { return redCenterAutoPose; }
        else if (location == 3 && alliance == DriverStation.Alliance.Red) { return redProcessorSideAutoPose; }
        if (location == 1 && alliance == DriverStation.Alliance.Blue) { return blueBargeSideAutoPose; }
        else if (location == 2 && alliance == DriverStation.Alliance.Blue) { return blueCenterAutoPose; }
        else if (location == 3 && alliance == DriverStation.Alliance.Blue) { return blueProcessorSideAutoPose; }
        else if (location == 1 && alliance == DriverStation.Alliance.Red) { return redBargeSideAutoPose; }
        else if (location == 2 && alliance == DriverStation.Alliance.Red) { return redCenterAutoPose; }
        else if (location == 3 && alliance == DriverStation.Alliance.Red) { return redProcessorSideAutoPose; }

        return new Pose2d();
    }

    // Returns the nearest reef apriltag from an input pose
    public static int getNearestReefTag(Pose3d currentRobotPose) {

        double minDistance = 3.0;
        int closestTag = -1;

        for(int reefTag: REEF_TAGS){
            
            Pose3d reefFacePose = aprilTagFieldLayout.getTagPose(reefTag).get();
            double dx = reefFacePose.getX() - currentRobotPose.getX();
            double dy = reefFacePose.getY() - currentRobotPose.getY();
            double distance = Math.sqrt(Math.pow(dx,2)+Math.pow(dy,2));

            if(distance < minDistance){
                minDistance = distance;
                closestTag = reefTag;
            }
        }
        return closestTag;
    }

    // See AprilTag field map: https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/Apriltag_Images_and_User_Guide.pdf
    public static int getTagFromReef(Alliance alliance, String reefFace){
        
        if(reefFace.equals("A") && alliance == Alliance.Blue) return 18;
        else if(reefFace.equals("B") & alliance == Alliance.Blue) return 17;
        else if(reefFace.equals("C") & alliance == Alliance.Blue) return 22;
        else if(reefFace.equals("D") & alliance == Alliance.Blue) return 21;
        else if(reefFace.equals("E") & alliance == Alliance.Blue) return 20;
        else if(reefFace.equals("F") & alliance == Alliance.Blue) return 19;
        else if(reefFace.equals("A") && alliance == Alliance.Red) return 7;
        else if(reefFace.equals("B") & alliance == Alliance.Red) return 8;
        else if(reefFace.equals("C") & alliance == Alliance.Red) return 9;
        else if(reefFace.equals("D") & alliance == Alliance.Red) return 10;
        else if(reefFace.equals("E") & alliance == Alliance.Red) return 11;
        else if(reefFace.equals("F") & alliance == Alliance.Red) return 6;
        else return -1;
    }

    // Determine the tag at the desired Coral Station
    public static int getTagFromCoralStation(Alliance alliance, String stationDirection){

        if (alliance == Alliance.Blue && stationDirection.equals("LEFT")) return 13;
        else if (alliance == Alliance.Blue && stationDirection.equals("RIGHT")) return 12;
        else if (alliance == Alliance.Red && stationDirection.equals("LEFT")) return 1;
        else if (alliance == Alliance.Red && stationDirection.equals("RIGHT")) return 2;
        else return -1;
    }

    public static void printPose3d(Pose3d p3d){
        System.out.printf("\tX: \t%.3f", p3d.getX());
        System.out.printf("\tY: \t%.3f", p3d.getY());
        System.out.printf("\tAngle: \t%.1f", Math.toDegrees(p3d.getRotation().getZ()));
    }

    // See AprilTag field map: https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/Apriltag_Images_and_User_Guide.pdf
    public static int getTagFromReef(String reefFace, Alliance alliance){
        
        if(reefFace.equals("A") && alliance == Alliance.Blue) return 18;
        else if(reefFace.equals("B") & alliance == Alliance.Blue) return 17;
        else if(reefFace.equals("C") & alliance == Alliance.Blue) return 22;
        else if(reefFace.equals("D") & alliance == Alliance.Blue) return 21;
        else if(reefFace.equals("E") & alliance == Alliance.Blue) return 20;
        else if(reefFace.equals("F") & alliance == Alliance.Blue) return 19;
        else if(reefFace.equals("A") && alliance == Alliance.Red) return 7;
        else if(reefFace.equals("B") & alliance == Alliance.Red) return 8;
        else if(reefFace.equals("C") & alliance == Alliance.Red) return 9;
        else if(reefFace.equals("D") & alliance == Alliance.Red) return 10;
        else if(reefFace.equals("E") & alliance == Alliance.Red) return 11;
        else if(reefFace.equals("F") & alliance == Alliance.Red) return 6;
        else return -1;
    }

    // Determine the tag at the desired Coral Station
    public static int getTagFromCoralStation(Alliance alliance, String stationDirection){

        if (alliance == Alliance.Blue && stationDirection.equals("LEFT")) return 13;
        else if (alliance == Alliance.Blue && stationDirection.equals("RIGHT")) return 12;
        else if (alliance == Alliance.Red && stationDirection.equals("LEFT")) return 1;
        else if (alliance == Alliance.Red && stationDirection.equals("RIGHT")) return 2;
        else return -1;
    }

    public static void printPose3d(Pose3d p3d){
        System.out.printf("\tX: \t%.3f", p3d.getX());
        System.out.printf("\tY: \t%.3f", p3d.getY());
        System.out.printf("\tAngle: \t%.1f", Math.toDegrees(p3d.getRotation().getZ()));
    }

    
}
