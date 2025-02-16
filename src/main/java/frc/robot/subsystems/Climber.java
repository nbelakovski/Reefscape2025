// package frc.robot.subsystems;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.SparkMaxConfig;

// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import frc.robot.utils.Ports;



// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Climber extends SubsystemBase {
//     private SparkMax rightMotor;
//     private SparkMax leftMotor;
//     private RelativeEncoder leftEncoder;
//     private RelativeEncoder rightEncoder;
//     private static Climber instance;

//     private Climber() {
//         this.rightMotor = new SparkMax(frc.robot.utils.Ports.CLIMB_RIGHT_MOTOR_PORT, MotorType.kBrushless);
//         this.leftMotor = new SparkMax(frc.robot.utils.Ports.CLIMB_LEFT_MOTOR_PORT, MotorType.kBrushless);
        
//         rightEncoder = rightMotor.getEncoder();
//         leftEncoder = leftMotor.getEncoder();
//     }

//     public static Climber getInstance(){
//         if(instance == null) {
//             instance = new Climber();
//         }
//         return instance;
//     }

//     public void climberReach(double speed) {
//         SmartDashboard.putString("climb", "reach");

//         rightMotor.set(speed);
//         leftMotor.set(speed);
//     }

//     public void climberRetract(double speed) {
//         SmartDashboard.putString("climb", "retract");

//         rightMotor.set(-speed);
//         leftMotor.set(-speed);
//     }

//     public void move(double speed){
//         if(speed >0.1){
//             climberReach(Math.abs(speed));
//         }
//         else if(speed < -0.1){
//             climberRetract(Math.abs(speed));
//         }
//         else{
//             stop();
//         }
//     }

//     public void stop(){
//         rightMotor.set(0);
//         leftMotor.set(0);
//     }

//     public void resetRightEncoder(){
//         rightEncoder.setPosition(0);
//     }

//     public void resetLeftEncoder(){
//         leftEncoder.setPosition(0);
//     }
    
//     public double getRightHeight(){
//         return this.rightEncoder.getPosition();
//     }
//     public double getLeftHeight(){
//         return this.leftEncoder.getPosition();
//     }

//     @Override
//     public void periodic(){
//         SmartDashboard.putNumber("right climber encoder", getRightHeight());
//         SmartDashboard.putNumber("left climber encoder", getLeftHeight());
//     }

// }

// //Made by JB & SP