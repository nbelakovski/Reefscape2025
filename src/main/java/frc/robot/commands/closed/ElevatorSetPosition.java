// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.closed;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorSetPosition extends Command {

    private static Elevator elevator;
    private PIDController controller;

    private boolean finished;
    private double setpoint;

  /** Creates a new ElevatorSetPositionNew. */
  public ElevatorSetPosition(double desiredPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
        elevator = Elevator.getInstance();

        controller = new PIDController(0.1, 0, 0);

        controller.setSetpoint(desiredPosition);
        controller.setTolerance(0.1);

        setpoint = desiredPosition;
        finished = false;
        addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.reset();
    elevator.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = controller.calculate(elevator.getPosition());
    elevator.move(speed);

    // if (setpoint == ElevatorConstants.ELEVATOR_L4 && (controller.getError() < 2 && controller.getError() > -2)) {
    //   finished = true;
    // }
    // else if (controller.getError() < 0.5 && controller.getError() > -0.5) {
    //   finished = true;
    // }
    // else {
    //   finished = false;
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
