package frc.robot;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.utils.Ports;


public class Motors {
    private static Motors instance;


    public SparkMax coralScorer;

    private Motors() {
        coralScorer = new SparkMax(Ports.CORALSCORER_MOTOR_PORT, MotorType.kBrushless);
        coralScorer.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // I may need to have Robot call this in robotInit to make sure someone calls it before
    // any mechanisms are used, so that all the motors are initialized properly.
    public static Motors get() {
        if (instance == null) {
            instance = new Motors();
        }
        return instance;
    }

    public static void coralScorer(double speed) {
        instance.coralScorer.set(speed);
    }
}
