// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Motors.InnerMotors;
import frc.robot.subsystems.AlgaeHandler;
import frc.robot.subsystems.LEDStrip;
import frc.robot.subsystems.LEDStrip.SubsystemPriority;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DataLogManager;


/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private static Robot instance;

  private final RobotContainer m_robotContainer;
  private boolean coralInScorer = false;
  private boolean coralInGap = false;

  public static boolean hasCoral() {
    if (Sensors.scorer() > 2000) {
      instance.coralInScorer = true;
    } else if (Sensors.scorer() < 1800) {
      instance.coralInScorer = false;
    }
    return instance.coralInScorer;
  }

  public static boolean isGapBlocked() {
    if (Sensors.gap() > 1000 && Sensors.gap() < 1800) {
      instance.coralInGap = true;
    } else if (Sensors.gap() < 900 || Sensors.gap() > 1900) {
      instance.coralInGap = false;
    }
    return instance.coralInGap;
  }

  public static Robot get() {
    if (instance == null) {
      instance = new Robot();
    }
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  private Robot() {
    CameraServer.startAutomaticCapture();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  public void robotInit() {
    // If publishing to NetworkTables and DataLog
    DataLogManager.start();
  }
  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putBoolean("Coral in scorer", hasCoral());
    SmartDashboard.putNumber("Coral analog distance", Sensors.scorer());
    SmartDashboard.putBoolean("Coral in gap", isGapBlocked());
    SmartDashboard.putNumber("Coral gap distance", Sensors.gap());
    // TODO: Need to move LEDStrip to Motors
    if(coralInGap && !Robot.hasCoral()){
      LEDStrip.request(SubsystemPriority.CORAL, LEDStrip.IN_GAP);
    }
    else if (coralInGap && Robot.hasCoral()) {
      LEDStrip.request(SubsystemPriority.CORAL, LEDStrip.IN_GAP_SCORER);
    }
    else if(!coralInGap && Robot.hasCoral()){
      LEDStrip.request(SubsystemPriority.CORAL, LEDStrip.IN_SCORER);
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    Motors.setJawIdleMode(IdleMode.kCoast);
    InnerMotors.jawAngle();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    Motors.setJawIdleMode(IdleMode.kBrake);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
