//Created by Gabriel R

package frc.robot.subsystems;


import frc.robot.utils.Ports;
import frc.robot.MTR;
import frc.robot.Constants.MechConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeHandler extends SubsystemBase {

    private static AlgaeHandler instance;
    private SparkMax jawMotor;
    private SparkMaxConfig jawConfig;
    private RelativeEncoder jawEncoder;
    private PIDController controller = new PIDController(0.15, 0, 0.02);

    // AlgaeHandler Constructor
    private AlgaeHandler() {

        jawMotor = new SparkMax(Ports.ALGAE_JAW_MOTOR_PORT, MotorType.kBrushless);
        jawConfig = new SparkMaxConfig();
        jawConfig.inverted(true);
        jawEncoder = jawMotor.getEncoder();
        jawEncoder.setPosition(0);
        // jawEncoder = jawMotor.getAbsoluteEncoder();
        // jawConfig.absoluteEncoder.positionConversionFactor(360);
        // jawConfig.absoluteEncoder.zeroOffset(0.94);
        // jawEncoder.setPositionConversionFactor(360); tried making it to 360 but method wont work

        controller.setTolerance(2);
    }

    // AlgaeHandler Singleton - ensures AlgaeHandler is only constructed once
    public static AlgaeHandler getInstance(){
        if(instance == null) {
            instance = new AlgaeHandler();
        }
        return instance;
    }

    public Command jawAngleCommand(double desiredAngle) {
        /*
         * NOTES:
         * So as far as capturing this controller goes, Runnables can capture variables that
         * are on the heap. Ideally though I'd like for there to be a single instance of this
         * controller, i.e. I don't want it to be recreated every time someone makes the command,
         * because that risks having two controllers active at once. It's unlikely though, and there
         * are other ways of making sure that two aren't active at once (particularly if it's tied
         * to a button).
         * Then again, why should the controller continue to live when the command isn't running? Particularly
         * if the motors are in brake mode?
         * Of course, if they're not in brake mode.... if they're not in brake mode then the command
         * should continue running, but ideally there's a way to access the controller and
         * change its setpoint.
         * Sending voltages is all well and good, but when you want the robot to autonomously
         * send voltages now you need to maintain state somewhere.
         * Perhaps controllers should live in Motors, adjacent to the motor they control
         * And then CommandFactory can reference those controllers. CF would really just need to
         * set the setpoint on the controller. So then who calculates the error and sets the motor? Motors?
         * Does that mean we should make it into a Subsystem? Oof. Foiled.
         * 
         * OK so I think we should have long-lived controllers, and only one instance of a controller
         * for a particular thing, and then commands will handle the setting and running of controllers.
         * And actually CommandScheduler.getInstance().run() is already in robotPeriodic, so that
         * visibility that I so value is already partially there. My other thought was to basically
         * reimplement Command so that we have greater visibility on it, and maybe that's an option
         * but I think for draft 1 of this structure we can keep it.
         */
        return new FunctionalCommand(
            () -> {
                this.controller.reset();
                this.controller.setSetpoint(desiredAngle);
            },
            () -> {
                double speed = this.controller.calculate(this.getAngle());
                MTR.jawMotor.set(speed);
            },
            (interrupted) -> MTR.jawMotor.set(0),
            () -> this.controller.atSetpoint(),
            instance
        );
    }

    public double getAngle(){
        //return encoder.getPosition()*90/32-2.3;
        return jawEncoder.getPosition();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Angle of Jaw", getAngle());
    }
}
