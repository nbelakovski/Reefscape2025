package frc.robot.subsystems;


import frc.robot.utils.Ports;
import frc.robot.Robot;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.LEDStrip.SubsystemPriority;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;


public class Elevator extends SubsystemBase {

  private static Elevator instance;

  private SparkMax elevatorLeftMotor;
  private SparkMax elevatorRightMotor;
  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;
  private SparkMaxConfig leftMotorConfig;
  private SparkMaxConfig rightMotorConfig;
  // I don't see us using the limit switches, are they real?
  private DigitalInput topLimitSwitch;
  private DigitalInput bottomLimitSwitch;
  private PIDController controller = new PIDController(0.1, 0, 0);


  // Elevator Constructor
  private Elevator() {

    elevatorLeftMotor = new SparkMax(Ports.ELEVATOR_LEFT_MOTOR_PORT, MotorType.kBrushless);
    elevatorRightMotor = new SparkMax(Ports.ELEVATOR_RIGHT_MOTOR_PORT, MotorType.kBrushless);
    leftMotorConfig = new SparkMaxConfig();
    rightMotorConfig = new SparkMaxConfig();
    leftEncoder = elevatorLeftMotor.getEncoder();
    rightEncoder = elevatorRightMotor.getEncoder();
    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);
    topLimitSwitch = new DigitalInput(Ports.DIGITAL_TOP_LIMIT_PORT);
    bottomLimitSwitch = new DigitalInput(Ports.DIGITAL_BOTTOM_LIMIT_PORT);
    
    rightMotorConfig.inverted(ElevatorConstants.RIGHT_ELEVATOR_INVERTED);
  
    rightMotorConfig.idleMode(IdleMode.kBrake);
    leftMotorConfig.idleMode(IdleMode.kBrake);
    //leftMotorConfig.encoder.inverted();
    elevatorRightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevatorLeftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    controller.setTolerance(0.1);
  }

  // Elevator Singleton - ensures only 1 instance of Elevator is constructed
  public static Elevator getInstance() {
    if (instance == null) {
        instance = new Elevator();
    }
    return instance;
  }

  // Primary method to move the elevator up & down
  public void setSpeed(double speed){
    if (getPosition() >= ElevatorConstants.ELEVATOR_MAX ||
        getPosition() <= ElevatorConstants.ELEVATOR_MIN ) {
      speed = 0;
    }
    if (CoralIntake.getInstance().isGapBlocked() && getPosition() < 3) {
      speed = 0;
    }

    speed = MathUtil.clamp(speed, -0.8, 0.8);
    elevatorLeftMotor.set(speed);
    elevatorRightMotor.set(-speed);
  }

  public Command setPosition(double desiredPosition) {
        return new FunctionalCommand(
            () -> {
                this.controller.reset();
                this.controller.setSetpoint(desiredPosition);
            },
            () -> {
                double speed = this.controller.calculate(this.getPosition());
                this.setSpeed(speed);
            },
            (interrupted) -> this.setSpeed(0),
            () -> this.controller.atSetpoint(),
            instance
        );
  }

  public Command setIntake() { return setPosition(ElevatorConstants.INTAKE_HEIGHT); }
  public Command setL1() { return setPosition(ElevatorConstants.ELEVATOR_L1); }
  public Command setL2() { return setPosition(ElevatorConstants.ELEVATOR_L2); }
  public Command setL3() { return setPosition(ElevatorConstants.ELEVATOR_L3); }
  public Command setL4() { return setPosition(ElevatorConstants.ELEVATOR_L4); }

  //----------------- SENSOR METHODS -----------------//

  // Gets the sensor position of the elevator
  public double getPosition() {
    double avg = (leftEncoder.getPosition() + -rightEncoder.getPosition()) / 2;
    return avg;
  }
  
  // Gets the value of the top limit switch
  public boolean getTopLimit() {
    return !topLimitSwitch.get();
  }

  // Gets the value of the bottom limit switch
  public boolean getBotLimit() {
    return !bottomLimitSwitch.get();
  }

  // Resets the values of both encoders
  public void resetPosition(double pos){
    leftEncoder.setPosition(pos);
    rightEncoder.setPosition(pos);
  }

  // Resets the values of both encoders to zero
  public void zeroPosition(){
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("elevator position", getPosition());
    SmartDashboard.putNumber("left elevator position", leftEncoder.getPosition());
    SmartDashboard.putNumber("right elevator position", -rightEncoder.getPosition());
    SmartDashboard.putBoolean("Top Limit", getTopLimit());
    SmartDashboard.putBoolean("Bottom Limit", getBotLimit());

    boolean hasCoral = Robot.hasCoral();
    boolean nearIntake = getPosition() > 2 && getPosition() < 3.5;
    boolean nearL1 = (getPosition() > ElevatorConstants.ELEVATOR_L1 - 1 && getPosition() < ElevatorConstants.ELEVATOR_L1 + 1);
    boolean nearL2 = (getPosition() > ElevatorConstants.ELEVATOR_L2 - 1 && getPosition() < ElevatorConstants.ELEVATOR_L2 + 1);
    boolean nearL3 = (getPosition() > ElevatorConstants.ELEVATOR_L3 - 1 && getPosition() < ElevatorConstants.ELEVATOR_L3 + 1);
    boolean nearL4 = (getPosition() > ElevatorConstants.ELEVATOR_L4 - 1 && getPosition() < ElevatorConstants.ELEVATOR_L4 + 1);
    boolean nearSomething = nearIntake || nearL1 || nearL2 || nearL3 || nearL4;
    if (nearSomething && hasCoral) {
      LEDStrip.request(SubsystemPriority.ELCORAL, LEDStrip.SCORE_READY);
    }
    
    if (nearIntake){
      LEDStrip.request(SubsystemPriority.ELEVATOR, LEDStrip.INTAKE_HEIGHT);
    }
    else if (nearL1) {
      LEDStrip.request(SubsystemPriority.ELEVATOR, LEDStrip.L1);
    }
    else if (nearL2) {
      LEDStrip.request(SubsystemPriority.ELEVATOR, LEDStrip.L2);
    }
    else if (nearL3) {
      LEDStrip.request(SubsystemPriority.ELEVATOR, LEDStrip.L3);
    }
    else if (nearL4) {
      LEDStrip.request(SubsystemPriority.ELEVATOR, LEDStrip.L4);
    }

  }
}

// Authors: jb ms
