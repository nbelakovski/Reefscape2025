// Created by Gabriel R

package frc.robot.subsystems;


import frc.robot.utils.Ports;
import frc.robot.Constants.MechConstants;
import frc.robot.subsystems.LEDStrip.SubsystemPriority;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;


public class CoralIntake extends SubsystemBase {

  private static CoralIntake instance;
  private SparkMax coralIntakeMotor;
  private SparkMaxConfig motorConfig;
  private boolean coralInGap = false;
  private AnalogInput gapSensor;
  private double distance;
  

  // CoralIntake Constructor
  private CoralIntake() {

    coralIntakeMotor = new SparkMax(Ports.LEFT_INTAKE_MOTOR_PORT, MotorType.kBrushless);
    motorConfig = new SparkMaxConfig();
    coralIntakeMotor.configure(motorConfig, null, null);
    
    gapSensor = new AnalogInput(Ports.ANALOG_GAP_SENSOR_PORT);
  }
  
  // CoralIntake Singleton - ensures only 1 CoralIntake instance is constructed
  public static CoralIntake getInstance(){
    if(instance == null) {
      instance = new CoralIntake();
    }
    return instance;
  }


  public void eat(){
    coralIntakeMotor.set(MechConstants.CORAL_INTAKE_SPEED);
  }
  public void spit(){
    coralIntakeMotor.set(-MechConstants.CORAL_INTAKE_SPEED);
  }
  public void stop(){
    coralIntakeMotor.set(0);
  }

  // Checks if coral is blocking the gap between intake & coral scorer
  //new coral range
  public boolean isGapBlocked(){
    if(getDistance() > 1156 && getDistance() < 1800){
      coralInGap = true;
    }
    else{
      coralInGap = false;
    }
    return coralInGap;
  }

  // Gets the distance from sensor to coral stuck in gap
  public double getDistance(){
    distance = gapSensor.getValue();
    return distance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Coral in Gap", isGapBlocked());
    SmartDashboard.putNumber("anolog distance", getDistance());
    
    // LEDs priority
    if(coralInGap && !CoralScorer.getInstance().hasCoral()){
       LEDStrip.request(SubsystemPriority.CORAL, LEDStrip.IN_GAP);
    }
    else if (coralInGap && CoralScorer.getInstance().hasCoral()) {
        LEDStrip.request(SubsystemPriority.CORAL, LEDStrip.IN_GAP_SCORER);
    }
    else if(!coralInGap && CoralScorer.getInstance().hasCoral()){
        LEDStrip.request(SubsystemPriority.CORAL, LEDStrip.IN_SCORER);
      }
    }


  }
