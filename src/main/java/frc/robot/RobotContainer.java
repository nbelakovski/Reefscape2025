package frc.robot;


import frc.robot.Constants.*;
import frc.robot.utils.Ports;
import frc.robot.subsystems.*;
import frc.robot.commands.closed.*;
import frc.robot.commands.complex.*;
import frc.robot.commands.combos.*;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private static final XboxController driverController = new XboxController(Ports.DRIVER_CONTROLLER);
  private static final XboxController operatorController = new XboxController(Ports.OPERATOR_CONTROLLER);

  private SendableChooser<Command> autoChooser;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureBindings();

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
      () -> driverController.getAButton(),
      () -> driverController.getXButton()
    ));

    // Driver - DPAD - Align to AprilTag Branch LEFT or RIGHT
    new Trigger(() -> driverController.getPOV() == 270).whileTrue(new DriveToClosestBranch("LEFT"));
    new Trigger(() -> driverController.getPOV() == 180).whileTrue(new DriveToClosestBranch("CENTER"));
    new Trigger(() -> driverController.getPOV() == 90).whileTrue(new DriveToClosestBranch("RIGHT")); 

    new JoystickButton(driverController, Button.kLeftBumper.value).whileTrue(new DriveToClosestBranch("LEFT")); 
    new JoystickButton(driverController, Button.kRightBumper.value).whileTrue(new DriveToClosestBranch("RIGHT")); 

    //---------- ELEVATOR ----------//

    //Driver - Y - Elevator to Intake Height
    new JoystickButton(driverController, Button.kY.value).whileTrue(Elevator.getInstance().setIntake());

    //Operator - RY joystick - Manually move Elevator
    Elevator.getInstance().setDefaultCommand(new SafeElevatorJoystick(
      () -> operatorController.getRawAxis(5)
    ));
    // EventLoop m_loop;
    // operatorController.x(m_loop).rising()

    //Operator - DPAD - Elevator to L1, L2, L3, L4 heights
    new Trigger(() -> operatorController.getPOV() == 180).whileTrue(Elevator.getInstance().setL1());
    new Trigger(() -> operatorController.getPOV() == 270).whileTrue(Elevator.getInstance().setL2());
    new Trigger(() -> operatorController.getPOV() == 0).whileTrue(Elevator.getInstance().setL3());
    new Trigger(() -> operatorController.getPOV() == 90).whileTrue(Elevator.getInstance().setL4());
    
    //---------- CORAL INTAKE/ SCORING ----------//

    // Driver - RT - Move Elevator in position to Intake + Spin Intake wheels
    Command elevatorIntakeCombo = Elevator.getInstance().setIntake().alongWith(CF.coralInSafeCommand());
    new Trigger(() -> (driverController.getRawAxis(3) > 0.7)).whileTrue(elevatorIntakeCombo);        //RT 
  
    // Operator - LT - Intake Coral with sensors
    new Trigger(() -> (operatorController.getRawAxis(2) > 0.7)).whileTrue(CF.coralInSafeCommand());        //LT    
    
    // Operator - LB - Retract Coral if hanging too far out
    new JoystickButton(operatorController, Button.kLeftBumper.value).whileTrue(CF.retractCommand()); //LB
    
    //Operator - RT - Score Coral
    new Trigger(() -> (operatorController.getRawAxis(3) > 0.7)).whileTrue(CF.scoreCommand()); //RT
    

    //---------- ALGAE JAW ----------//
    
    // Operator - LY joystick - manually move Jaw up & down
    RobotModeTriggers.teleop().whileTrue(CF.safeAlgaeJoystick(
      () -> operatorController.getRawAxis(1)
    ));


    // Operator - A - Rotate jaw to Intake Angle
    Command jawIntakeAngle = CF.jawAngleCommand(MechConstants.JAW_INTAKE_ANGLE);
    new JoystickButton(operatorController, Button.kA.value).whileTrue(jawIntakeAngle);

    //Operator - B - Go to L4, Algae score angle, and spit algae 
    Command elevatorSpitCombo = Elevator.getInstance().setL4().alongWith(
      CF.jawAngleCommand(MechConstants.JAW_INTAKE_ANGLE),
      CF.algaeSpitCommand());
    new JoystickButton(operatorController, Button.kB.value).whileTrue(elevatorSpitCombo.repeatedly());
    
    // Operator - RB - Rotate jaw to bring Coral out of the way for a Supercycle
    Command jawUpAngle = CF.jawAngleCommand(MechConstants.JAW_UP_ANGLE);
    new JoystickButton(operatorController, Button.kLeftBumper.value).whileTrue(jawUpAngle); //RB

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
    new JoystickButton(operatorController, Button.kY.value).whileTrue(elevatorJawCombo); //11.7
    //new JoystickButton(operatorController, Button.kY.value).whileTrue(CF.algaeEatCommand());

    // Operator - X - Spit out the Algae
    new JoystickButton(operatorController, Button.kX.value).whileTrue(CF.algaeSpitCommand());
    new JoystickButton(operatorController, Button.kRightBumper.value).whileTrue(CF.algaeEatCommand());
  }

public void autoChooserInit() {

    autoChooser.setDefaultOption("PP-one meter", new PathPlannerAuto("one meter"));

    //autoChooser.addOption("one meter", new PathPlannerAuto("one meter"));

    autoChooser.addOption("L-onePieceAuto", new AutoPathToBranchScore("testing", "LEFT", 4));
    autoChooser.addOption("R-onePieceAuto", new AutoPathToBranchScore("testing", "RIGHT", 4));

    autoChooser.addOption("L-twoPieceAuto", new Auto2Piece("LEFT"));
    autoChooser.addOption("R-twoPieceAuto", new Auto2Piece("RIGHT"));
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
