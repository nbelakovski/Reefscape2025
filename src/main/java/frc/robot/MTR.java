package frc.robot;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.utils.Ports;


public class MTR {
    public static SparkMax coralScorer = new SparkMax(Ports.CORALSCORER_MOTOR_PORT, MotorType.kBrushless);

    public static void configureMotors() {
        coralScorer.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
