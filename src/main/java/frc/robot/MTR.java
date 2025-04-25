package frc.robot;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.utils.Ports;


public class MTR {
    public static SparkMax coralScorer       = new SparkMax(Ports.CORALSCORER_MOTOR_PORT  , MotorType.kBrushless);
    public static SparkMax coralIntakeMotorL = new SparkMax(Ports.LEFT_INTAKE_MOTOR_PORT  , MotorType.kBrushless);
    public static SparkMax coralIntakeMotorR = new SparkMax(Ports.RIGHT_INTAKE_MOTOR_PORT , MotorType.kBrushless);

    public static void configureMotors() {
        coralScorer      .configure(new SparkMaxConfig().idleMode(IdleMode.kBrake), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        coralIntakeMotorL.configure(new SparkMaxConfig(), null, null);
        coralIntakeMotorR.configure(new SparkMaxConfig(), null, null); // This one could be set up as a follower of L
    }

    public static void coralIntake(double speed) {
        coralIntakeMotorL.set(speed);
        coralIntakeMotorR.set(-speed); // should we invert this one?
    }
}
