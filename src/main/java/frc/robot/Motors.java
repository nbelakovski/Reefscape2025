package frc.robot;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.utils.Ports;


public class Motors {
    private static Motors instance;

    public class InnerMotors {
        private RelativeEncoder jawEncoder;
        public static InnerMotors instance;

        private InnerMotors() {
            jawEncoder = Motors.get().jawMotor.getEncoder();
        }

        public static InnerMotors get() {
            if (instance == null) {
                instance = Motors.get().new InnerMotors();
            }
            return instance;
        }

        public static double jawAngle() {
            return instance.jawEncoder.getPosition();
        }
    
        
    }


    private SparkMax coralScorer;
    private SparkMax coralIntakeMotorL;
    private SparkMax coralIntakeMotorR;
    private SparkMax tongueMotor;
    private SparkMax jawMotor;

    private Motors() {
        coralScorer = new SparkMax(Ports.CORALSCORER_MOTOR_PORT, MotorType.kBrushless);
        coralScorer.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        coralIntakeMotorL = new SparkMax(Ports.LEFT_INTAKE_MOTOR_PORT, MotorType.kBrushless);
        coralIntakeMotorL.configure(new SparkMaxConfig(), null, null);

        coralIntakeMotorR = new SparkMax(Ports.RIGHT_INTAKE_MOTOR_PORT, MotorType.kBrushless);
        coralIntakeMotorR.configure(new SparkMaxConfig(), null, null);

        tongueMotor = new SparkMax(Ports.ALGAE_TONGUE_MOTOR_PORT, MotorType.kBrushless);
        tongueMotor.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        jawMotor = new SparkMax(Ports.ALGAE_JAW_MOTOR_PORT, MotorType.kBrushless);
        jawMotor.configure(new SparkMaxConfig().inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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

    public static void coralIntake(double speed) {
        instance.coralIntakeMotorL.set(speed);
        instance.coralIntakeMotorR.set(-speed); // should we invert this one?
        // instance.coralIntakeMotorR.isFollower()
    }

    public static void tongueMotor(double speed) {
        instance.tongueMotor.set(speed);
    }

    public static void jawMotor(double speed) {
        instance.jawMotor.set(speed);
    }

    public static void setJawIdleMode(IdleMode idleMode) {
        boolean inverted = instance.jawMotor.configAccessor.getInverted();
        instance.jawMotor.configure(new SparkMaxConfig().inverted(inverted).idleMode(idleMode),
            ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /*
     * Options for getting the encoder into the Sensors class
     * 
     * Option 1: Make the SparkMax public, access it via Motors.get().jawMotor.getEncoder
     *   Pros: Less getters - just provide direct access, no need to create extra getters
     *   Cons: Somewhat verbose access pattern
     * 
     * Option 2: Make a getter, access it like Motors.jawEncoder()
     *   Pros: Simple access method
     *   Cons: requires an extra getter, and also break the line between Motors and Sensors
     * 
     * Option 3: Could I make sensors a subclass of Motors?
     *   Pros: Sensors class should be able to access the motors easily
     *   Cons: Now I need to do Motors.Sensors which is weird, unless I import it right?
     * 
     * 
     */
}
