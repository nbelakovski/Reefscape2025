//Created by Gabriel R

package frc.robot.subsystems;


import frc.robot.utils.Ports;
import frc.robot.Constants.MechConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeHandler extends SubsystemBase {

    private static AlgaeHandler instance;
    private SparkMax tongueMotor;
    private SparkMaxConfig tongueConfig;
    private SparkMax jawMotor;
    private SparkMaxConfig jawConfig;
    private RelativeEncoder jawEncoder;

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

        tongueMotor = new SparkMax(Ports.ALGAE_TONGUE_MOTOR_PORT, MotorType.kBrushless);
        tongueConfig = new SparkMaxConfig();
        tongueConfig.idleMode(IdleMode.kBrake);
        tongueMotor.configure(tongueConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // AlgaeHandler Singleton - ensures AlgaeHandler is only constructed once
    public static AlgaeHandler getInstance(){
        if(instance == null) {
            instance = new AlgaeHandler();
        }
        return instance;
    }

    public Command eatCommand() {
        return this.runEnd(
            () -> this.eat(),
            () -> this.stop());
    }

    public Command spitCommand() {
        return this.runEnd(
            () -> this.spit(),
            () -> this.stop());
    }

    public void setCoast(){
        jawConfig.idleMode(IdleMode.kCoast);
        //re-configures the sparkmax after changing brake/coast
        jawMotor.configure(jawConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setBrake(){
        jawConfig.idleMode(IdleMode.kBrake);
        //re-configures the sparkmax after changing brake/coast
        jawMotor.configure(jawConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    
    public void spit(){
        tongueMotor.set(MechConstants.ALGAE_INTAKE_SPEED);
    }
    
    public void eat(){
        tongueMotor.set(-MechConstants.ALGAE_INTAKE_SPEED);
    }
    
    public void stop(){
        tongueMotor.set(0);
    }
    
    public void zeroAngle() {
        jawEncoder.setPosition(0);
    }

    public void pivot(double speed){
        jawMotor.set(speed);
    }

    public void stopPivot(){
        jawMotor.set(0);   
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
