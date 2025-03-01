// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.utils.DPad;
// import frc.robot.commands.auto.*;
import frc.robot.commands.basic.*;
import frc.robot.commands.closed.*;
import frc.robot.commands.complex.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.commands.basic.AlgaeIn;
import frc.robot.commands.basic.AlgaeOut;
import frc.robot.commands.basic.CoralScore;
import frc.robot.commands.closed.ElevatorSetPosition;
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


  AlgaeHandler algae = AlgaeHandler.getInstance();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureBindings();

  
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("AutoChooser", autoChooser);
    SmartDashboard.putData("one meter", oneMeter);
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

    Drivetrain.getInstance().setDefaultCommand(new SwerveDrive(
      () -> -driverController.getRawAxis(1),
      () -> -driverController.getRawAxis(0),
      () -> -driverController.getRawAxis(4),
      () -> driverController.getAButton()
    ));
 

    // Driver Commands

    // new JoystickButton(driverController,Button.kB.value).whileTrue(new DriveToPegPID(cam.closestID, "RIGHT"));
    // new JoystickButton(driverController,Button.kX.value).whileTrue(new DriveToPegPID(cam.closestID, "LEFT"));
    // new JoystickButton(driverController,Button.kY.value).whileTrue(new DriveToPegPID(cam.closestID, "STRAIGHT"));
    //Operator commands
    // Link for joystick doc: https://docs.google.com/presentation/d/1cis5OrQfkU9m38LwgAMIfmPpJAZxnIC-KnAzi0JsRao/edit#slide=id.g18d2b75b637cb431_3

    //Manual Elevator on Operator Joystick
    Elevator.getInstance().setDefaultCommand(new SafeElevatorJoystick(
      () -> operatorController.getRawAxis(1)
    ));
    //Manual Algae on Operator Joystick
    AlgaeHandler.getInstance().setDefaultCommand(new SafeAlgaeJoystick(
      () -> -operatorController.getRawAxis(5)
    ));
    // Set Elevator Position for Operator on DPad
    new DPad(operatorController,180).whileTrue(new ElevatorSetPosition(ElevatorConstants.ELEVATOR_L1));
    new DPad(operatorController,270).whileTrue(new ElevatorSetPosition(ElevatorConstants.ELEVATOR_L2));
    new DPad(operatorController,0).whileTrue(new ElevatorSetPosition(ElevatorConstants.ELEVATOR_L3));
    new DPad(operatorController,90).whileTrue(new ElevatorSetPosition(ElevatorConstants.ELEVATOR_L4));

 //   new JoystickButton(operatorController,Button.kY.value).whileTrue(new ElevatorSetPosition(ElevatorConstants.ELEVATOR_L1));
    // Set Elevator Positions for Operator on Joystick Buttons
    // new JoystickButton(operatorController,Button.kY.value).whileTrue(new ElevatorSetPosition(ElevatorConstants.ELEVATOR_ALGAE_L3));
    // new JoystickButton(operatorController,Button.kX.value).whileTrue(new ElevatorSetPosition(ElevatorConstants.ELEVATOR_ALGAE_L2));
    // new JoystickButton(operatorController,Button.kA.value).whileTrue(new ElevatorSetPosition(ElevatorConstants.ELEVATOR_PROCESSOR));

    // new JoystickButton(operatorController, Button.kA.value).whileTrue(new SafeElevate());
    new JoystickButton(operatorController, Button.kA.value).whileTrue(new ElevatorSetPosition(1.1));
    new JoystickButton(operatorController, Button.kB.value).whileTrue(new SafeDescend());
    // new JoystickButton(operatorController, Button.kX.value).whileTrue(new CoralScore());
    // new JoystickButton(operatorController, Button.kY.value).whileTrue(new CoralInSafe());


    //Bumper buttons
    new JoystickButton(operatorController, Button.kX.value).whileTrue(new AlgaeIn());
    new JoystickButton(operatorController, Button.kY.value).whileTrue(new AlgaeOut());

    //Trigger buttons for operator
    new TriggerButton(operatorController, 2).whileTrue(new CoralInSafe());
    new TriggerButton(operatorController, 3).whileTrue(new CoralScore());
  }

public void autoChooserInit() {

    autoChooser.setDefaultOption("CoralIn", new CoralIn());

    autoChooser.addOption("CoralIn", new CoralIn());
    
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


