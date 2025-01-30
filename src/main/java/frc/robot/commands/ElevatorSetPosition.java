/*package frc.robot.commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class ElevatorSetPosition extends PIDCommand {

    private static Elevator elevator;

    public ElevatorSetPosition(double desiredPosition){
        super(
            new PIDController(1, 0, 0),
            () -> elevator.getPosition(),
            () -> desiredPosition,
            output -> {
                elevator.elevate(output);
            });
    }
    
}
*/