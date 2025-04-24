//Created by Gabriel R

package frc.robot.subsystems;


import frc.robot.utils.Ports;
import frc.robot.Motors;
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
        jawMotor.configure(jawConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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
        return new FunctionalCommand(
            () -> {
                this.controller.reset();
                this.controller.setSetpoint(desiredAngle);
            },
            () -> {
                double speed = this.controller.calculate(this.getAngle());
                Motors.jawMotor(speed);
            },
            (interrupted) -> Motors.jawMotor(0),
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
