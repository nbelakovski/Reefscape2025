// Created by Gabriel R

package frc.robot.subsystems;


import frc.robot.utils.Ports;
import frc.robot.Constants.MechConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;


public class CoralScorer extends SubsystemBase {

  private static CoralScorer instance;
  private SparkMax coralScorerMotor;
  private SparkMaxConfig motorConfig;
  private boolean coralInScorer = false;
  private AnalogInput scorerSensor;
  private double distance;
  

  // CoralScorer Constructor
  private CoralScorer() {
    coralScorerMotor = new SparkMax(Ports.CORALSCORER_MOTOR_PORT, MotorType.kBrushless);
    scorerSensor = new AnalogInput(Ports.ANALOG_CORAL_SENSOR_PORT);
    motorConfig = new SparkMaxConfig();
    motorConfig.idleMode(IdleMode.kBrake);
    coralScorerMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // CoralScorer Singleton - ensures only 1 CoralScorer object is constructed
  public static CoralScorer getInstance(){
    if(instance == null) {
      instance = new CoralScorer();
    }
    return instance;
  }

  // Spits out Coral slowly
  public void spitSlow(){
    coralScorerMotor.set(-MechConstants.CORAL_INTAKE_SPEED);
  }

  // Spits out Coral fast-ly
  public void spitFast(){
    coralScorerMotor.set(-MechConstants.CORAL_SCORE_SPEED);
  }

  // Pushes coral back in if hanging out too far
  public void backward(){
    coralScorerMotor.set(MechConstants.CORAL_RETRACT_SPEED);
  }

  // Stops the CS motor
  public void stop(){
    coralScorerMotor.set(0);
  }

  // Checks if Coral is inside scorer
  public boolean hasCoral(){
    if(getDistance() > 2000){
      coralInScorer = true;
    }
    else if (getDistance() < 1800) {
      coralInScorer = false;
    }
    return coralInScorer;
  }

  // Gets the distance the coral is from the sensor
  public double getDistance(){
    distance = scorerSensor.getValue();
    return distance;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Coral in Scorer", hasCoral());
    SmartDashboard.putNumber("coral anolog distance", getDistance());
  }
}
