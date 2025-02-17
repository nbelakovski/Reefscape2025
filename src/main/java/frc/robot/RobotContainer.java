// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.CoralIn;
import frc.robot.commands.CoralOut;
import frc.robot.commands.ElevatorDescend;
import frc.robot.commands.ElevatorElevate;
import frc.robot.commands.ElevatorSetPosition;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.DPad;
import frc.robot.utils.Ports;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {


  private static final XboxController driverController = new XboxController(Ports.DRIVER_CONTROLLER);
  private static final XboxController operatorController = new XboxController(Ports.OPERATOR_CONTROLLER);

  Drivetrain drivetrain = Drivetrain.getInstance();
  Camera cam = Camera.getInstance();

private SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

  
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("AutoChooser", autoChooser);
    // Configure the trigger bindings
    configureBindings();
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

    Drivetrain.getInstance().setDefaultCommand(new SwerveDrive(
      () -> -driverController.getRawAxis(1),
      () -> -driverController.getRawAxis(0),
      () -> -driverController.getRawAxis(4),
      () -> driverController.getAButton()
    ));
    
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`



    // Elevator Elevate + Elevator Descend 

    new JoystickButton(operatorController,Button.kY.value).whileTrue(new ElevatorElevate());
    new JoystickButton(operatorController,Button.kA.value).whileTrue(new ElevatorDescend());

    // Set Elevator Position for Driver on DPad
    new DPad(driverController,90).whileTrue(new ElevatorSetPosition(ElevatorConstants.ELEVATOR_L2));
    new DPad(driverController,0).whileTrue(new ElevatorSetPosition(ElevatorConstants.ELEVATOR_L3));
  
  // Makes button Y/A Algae Intake/Outake
  // new JoystickButton(operatorController, Button.kY.value).whileTrue(new AlgaeIn());
  // new JoystickButton(operatorController, Button.kA.value).whileTrue(new AlgaeOut());

  }

  public void autoChooserInit() {

    autoChooser.setDefaultOption("CoralIn", new CoralIn());

    autoChooser.addOption("CoralIn", new CoralIn());
    autoChooser.addOption("CoralOut", new CoralOut());

    


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


