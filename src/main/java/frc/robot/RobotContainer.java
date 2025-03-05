// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.MechConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.utils.DPad;
// import frc.robot.commands.auto.*;
import frc.robot.commands.basic.*;
import frc.robot.commands.closed.*;
import frc.robot.commands.complex.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.commands.basic.AlgaeEat;
import frc.robot.commands.basic.AlgaeSpit;
import frc.robot.commands.basic.CoralScore;
import frc.robot.commands.combos.ElevatorIntakeCombo;
import frc.robot.commands.combos.ElevatorJawCombo;
import frc.robot.commands.combos.ElevatorScoreCombo;
import frc.robot.commands.complex.CoralInSafe;
import frc.robot.commands.complex.SwerveDrive;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.LEDStrip;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveToPeg;
import frc.robot.commands.DriveToPegPID;
import frc.robot.subsystems.AlgaeHandler;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.utils.Ports;
import frc.robot.utils.TriggerButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private LEDStrip led;
  


  private static final XboxController driverController = new XboxController(Ports.DRIVER_CONTROLLER);
  private static final XboxController operatorController = new XboxController(Ports.OPERATOR_CONTROLLER);

  Drivetrain drivetrain = Drivetrain.getInstance();
  Camera cam = Camera.getInstance();

 private SendableChooser<Command> autoChooser;
 private Command auto1 = new PathPlannerAuto("Auto1");
 private Command oneMeter = new PathPlannerAuto("one meter");
 private Command testing = new PathPlannerAuto("testing");


  AlgaeHandler algae = AlgaeHandler.getInstance();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureBindings();

  
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("AutoChooser", autoChooser);
    // Configure the trigger bindings
    autoChooserInit();

    led = new LEDStrip();
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


    // DRIVETRAIN
    Drivetrain.getInstance().setDefaultCommand(new SwerveDrive(
      () -> -driverController.getRawAxis(1),
      () -> -driverController.getRawAxis(0),
      () -> -driverController.getRawAxis(4),
      () -> driverController.getAButton()
    ));
    // new JoystickButton(driverController,Button.kB.value).whileTrue(new DriveToPegPID(cam.closestID, "RIGHT"));
    // new JoystickButton(driverController,Button.kX.value).whileTrue(new DriveToPegPID(cam.closestID, "LEFT"));
    // new JoystickButton(driverController,Button.kY.value).whileTrue(new DriveToPegPID(cam.closestID, "STRAIGHT"));


    // CORAL INTAKE
    new JoystickButton(driverController, Button.kY.value).whileTrue(new ElevatorSetPosition(ElevatorConstants.INTAKE_HEIGHT));
    new JoystickButton(driverController, Button.kX.value).whileTrue(new ElevatorIntakeCombo());
    new TriggerButton(operatorController, 3).whileTrue(new ElevatorIntakeCombo());
    new TriggerButton(operatorController, 2).whileTrue(new CoralInSafe());
    // new JoystickButton(operatorController, Button.kY.value).whileTrue(new CoralInSafe());


    //ELEVATOR
    Elevator.getInstance().setDefaultCommand(new SafeElevatorJoystick(
      () -> operatorController.getRawAxis(1)
    ));
    new DPad(operatorController,180).whileTrue(new ElevatorSetPosition(ElevatorConstants.ELEVATOR_L1));
    new DPad(operatorController,270).whileTrue(new ElevatorSetPosition(ElevatorConstants.ELEVATOR_L2));
    new DPad(operatorController,0).whileTrue(new ElevatorSetPosition(ElevatorConstants.ELEVATOR_L3));
    new DPad(operatorController,90).whileTrue(new ElevatorSetPosition(ElevatorConstants.ELEVATOR_L4));
    // new JoystickButton(operatorController, Button.kB.value).whileTrue(new SafeElevate());
      

    //CORAL SCORE
    new TriggerButton(operatorController, 3).whileTrue(new CoralScore());
    // new JoystickButton(operatorController, Button.kX.value).whileTrue(new CoralScore());
    

    // ALGAE JAW
    AlgaeHandler.getInstance().setDefaultCommand(new SafeAlgaeJoystick(
      () -> operatorController.getRawAxis(5)
    ));
    new JoystickButton(operatorController, Button.kA.value).whileTrue(new SetJawAngle(MechConstants.JAW_INTAKE_ANGLE));
    new JoystickButton(operatorController, Button.kB.value).whileTrue(new SetJawAngle(MechConstants.JAW_CORAL_STOP_ANGLE));

    new JoystickButton(driverController, Button.kB.value).whileTrue(new ElevatorJawCombo(ElevatorConstants.ELEVATOR_ALGAE_L2));
    new DPad(driverController, 0).whileTrue(new SetJawAngle(MechConstants.JAW_INTAKE_ANGLE));
    new DPad(driverController, 180).whileTrue(new SetJawAngle(MechConstants.JAW_CORAL_STOP_ANGLE));
    //new JoystickButton(driverController, Button.kA.value).whileTrue(new SetJawAngle(MechConstants.JAW_INTAKE_ANGLE));


    //ALGAE TONGUE
    new JoystickButton(operatorController, Button.kX.value).whileTrue(new AlgaeSpit());
    new JoystickButton(operatorController, Button.kY.value).whileTrue(new AlgaeEat());


  }

public void autoChooserInit() {

    autoChooser.setDefaultOption("one meter", oneMeter);

    autoChooser.addOption("Auto 1", auto1);
    //autoChooser.addOption("one meter", oneMeter);
    autoChooser.addOption("testing", testing);
}
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}



