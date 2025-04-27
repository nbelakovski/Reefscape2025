package frc.robot;


import frc.robot.Constants.*;
import frc.robot.utils.Ports;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Drivetrain.SwerveConstants;
import frc.robot.commands.closed.*;
import frc.robot.commands.complex.*;
import frc.robot.commands.combos.*;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private static final CommandXboxController driverController = new CommandXboxController(Ports.DRIVER_CONTROLLER);
  private static final CommandXboxController operatorController = new CommandXboxController(Ports.OPERATOR_CONTROLLER);

  private SendableChooser<Command> autoChooser;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureBindings();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
      () -> ETechPoseEstimator.getInstance().getPose(),
      (Pose2d newPose) -> ETechPoseEstimator.getInstance().resetPose(newPose),
      () -> Drivetrain.getInstance().getSpeeds(),
      (ChassisSpeeds speeds) -> Drivetrain.getInstance().driveRobotRelative(speeds),
      new PPHolonomicDriveController(
        new PIDConstants(1, 0, 0), // Translation constants
        new PIDConstants(SwerveAutoConstants.TURN_P, SwerveAutoConstants.TURN_I, SwerveAutoConstants.TURN_D) // Rotation constants
      ),
      new RobotConfig(
        RobotConstants.MASS,
        RobotConstants.MOI,
        new ModuleConfig(
          SwerveModuleConstants.WHEEL_DIAMETER_METERS/2,
          SwerveConstants.TOP_SPEED,
          SwerveModuleConstants.WHEEL_COEFFICIENT_OF_FRICTION,
          DCMotor.getNEO(1).withReduction(SwerveModuleConstants.DRIVE_GEAR_REDUCTION),
          SwerveModuleConstants.kDrivingMotorCurrentLimit,
          1),
        Drivetrain.getInstance().driveKinematics.getModules()
      ),
      () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
      },
      Drivetrain.getInstance()
    );

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("AutoChooser", autoChooser);
    
    // Configure the trigger bindings
    autoChooserInit();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // Link for joystick doc: https://docs.google.com/presentation/d/1cis5OrQfkU9m38LwgAMIfmPpJAZxnIC-KnAzi0JsRao/edit#slide=id.g18d2b75b637cb431_3


    //---------- DRIVETRAIN ----------//

   //Driver - LX & LY joysticks for Translation, RX joystick for Strafing, A to reset Robot NavX Heading
    Drivetrain.getInstance().setDefaultCommand(new SwerveDrive(
      () -> driverController.getRawAxis(1),
      () -> -driverController.getRawAxis(0),
      () -> -driverController.getRawAxis(4), //negative joystick values make a positive CCW turn
      () -> driverController.getHID().getAButton(),
      () -> driverController.getHID().getXButton()
    ));

    // Driver - DPAD - Align to AprilTag Branch LEFT or RIGHT
    driverController.povLeft().whileTrue(new DriveToClosestBranch("LEFT"));
    driverController.povDown().whileTrue(new DriveToClosestBranch("CENTER"));
    driverController.povRight().whileTrue(new DriveToClosestBranch("RIGHT"));

    driverController.leftBumper().whileTrue(new DriveToClosestBranch("LEFT"));
    driverController.rightBumper().whileTrue(new DriveToClosestBranch("RIGHT"));

    //---------- ELEVATOR ----------//

    //Driver - Y - Elevator to Intake Height
    driverController.y().whileTrue(Elevator.getInstance().setIntake());

    //Operator - RY joystick - Manually move Elevator
    Elevator.getInstance().setDefaultCommand(new SafeElevatorJoystick(
      () -> operatorController.getRawAxis(5)
    ));

    //Operator - DPAD - Elevator to L1, L2, L3, L4 heights
    operatorController.povDown().whileTrue(Elevator.getInstance().setL1());
    operatorController.povLeft().whileTrue(Elevator.getInstance().setL2());
    operatorController.povUp().whileTrue(Elevator.getInstance().setL3());
    operatorController.povRight().whileTrue(Elevator.getInstance().setL4());
    
    //---------- CORAL INTAKE/ SCORING ----------//

    // Driver - RT - Move Elevator in position to Intake + Spin Intake wheels
    Command elevatorIntakeCombo = Elevator.getInstance().setIntake().alongWith(CF.coralInSafeCommand());
    driverController.rightTrigger(0.7).whileTrue(elevatorIntakeCombo);
  
    // Operator - LT - Intake Coral with sensors
    operatorController.leftTrigger(0.7).whileTrue(CF.coralInSafeCommand());
    
    // Operator - LB - Retract Coral if hanging too far out
    operatorController.leftBumper().whileTrue(CF.retractCommand());
    
    //Operator - RT - Score Coral
    operatorController.rightTrigger(0.7).whileTrue(CF.scoreCommand());
    

    //---------- ALGAE JAW ----------//
    
    // Operator - LY joystick - manually move Jaw up & down
    RobotModeTriggers.teleop().whileTrue(CF.safeAlgaeJoystick(
      () -> operatorController.getRawAxis(1)
    ));


    // Operator - A - Rotate jaw to Intake Angle
    Command elevatorSpitCombo = Elevator.getInstance().setL4().repeatedly().alongWith(
      CF.jawAngleCommand(MechConstants.JAW_INTAKE_ANGLE),
      CF.algaeSpitCommand());
    operatorController.a().whileTrue(elevatorSpitCombo);
    
    //Operator - B - Go to L4, Algae score angle, and spit algae 
    
    
    // Operator - RB - Rotate jaw to bring Coral out of the way for a Supercycle
    // Command jawUpAngle = CF.jawAngleCommand(MechConstants.JAW_UP_ANGLE);
    // operatorController.leftBumper().whileTrue(jawUpAngle);

    // Operator - B - Rotate Jaw to Starting/Coral Stop Angle
    // new JoystickButton(operatorController, Button.kB.value).whileTrue(new SetJawAngle(MechConstants.JAW_STARTING_ANGLE).repeatedly());

    //new JoystickButton(operatorController, Button.kRightBumper.value).whileTrue(new SetJawAngle(MechConstants.JAW_UP_ANGLE)); //RB

    // Driver - B - Snap Elevator & Jaw to prep for Algae on L2
    //new JoystickButton(driverController, Button.kB.value).whileTrue(new ElevatorJawCombo(ElevatorConstants.ELEVATOR_ALGAE_L2));


    //---------- ALGAE TONGUE ----------//

    // Operator - Y - Eat the Algae
    Command elevatorJawCombo =
        CF.jawAngleCommand(MechConstants.JAW_INTAKE_ANGLE).alongWith(
        CF.algaeSpitCommand());
    operatorController.y().whileTrue(elevatorJawCombo);
    //new JoystickButton(operatorController, Button.kY.value).whileTrue(CF.algaeEatCommand());

    // Operator - X - Spit out the Algae
    operatorController.x().whileTrue(CF.algaeSpitCommand());
    operatorController.rightBumper().whileTrue(CF.algaeEatCommand());
  }

public void autoChooserInit() {

    autoChooser.setDefaultOption("PP-one meter", new PathPlannerAuto("one meter"));

    //autoChooser.addOption("one meter", new PathPlannerAuto("one meter"));

    autoChooser.addOption("L-onePieceAuto", AutoPathToBranchScore.cmd("testing", "LEFT", ElevatorConstants.L4));
    autoChooser.addOption("R-onePieceAuto", AutoPathToBranchScore.cmd("testing", "RIGHT", ElevatorConstants.L4));

    autoChooser.addOption("L-twoPieceAuto", Auto2Piece.cmd("LEFT"));
    autoChooser.addOption("R-twoPieceAuto", Auto2Piece.cmd("RIGHT"));
    autoChooser.addOption("driveToBranch-Closest-LEFT", new DriveToClosestBranch("LEFT"));
    autoChooser.addOption("driveToBranch-21-LEFT", new DriveToBranchPID(21, "LEFT"));
    // autoChooser.addOption("turntoangle", new TurnToAnglePID(90));


    // Command driveForward = new ParallelRaceGroup(
    //   new SwerveDrive( ()-> 0.5, ()-> 0.0, ()->0.0,()->false, () -> false),
    //   new WaitCommand(1)
    // );
    // autoChooser.addOption("DriveForward", driveForward);

    autoChooser.addOption("testing", new PathPlannerAuto("testing"));
    autoChooser.addOption("PP-Auto1", new PathPlannerAuto("Auto1"));
    autoChooser.addOption("PP-Auto2", new PathPlannerAuto("Auto2"));
    autoChooser.addOption("PP-ERComboPath", new PathPlannerAuto("ERComboPath"));
    autoChooser.addOption("CRComboPath", new PathPlannerAuto("CRComboPath"));



    // Table for AprilTag IDs
    // 9	Red Reef C --> (Blue 22)
    // 10	Red Reef D --> (Blue 21)
    // 11	Red Reef E --> (Blue 20)
    // 22	Blue Reef C --> (Red 9)
    // 21	Blue Reef D --> (Red 10)
    // 20	Blue Reef E --> (Red 11)

}

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
