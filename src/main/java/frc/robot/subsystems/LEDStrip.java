package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LEDColors;
import frc.robot.utils.Ports;

public class LEDStrip extends SubsystemBase {
    private static Spark LED = new Spark(Ports.BLINKIN_PORT);

    public static final double HAVE_ALGAE = LEDColors.YELLOW; //random colors
    
   // public static final double IN_BLUE_ZONE = LEDColors.BLUE;
    // public static final double IN_GREEN_ZONE = LEDColors.GREEN;
    public static final double DISABLED = LEDColors.END_BLEND_C1_C2;
    public static final double ENABLED = LEDColors.OCEAN_COLOR_WAVES;

    // Intake/Coral Colors 
    public static final double IN_SCORER = LEDColors.FOREST_BPM;
    public static final double IN_GAP = LEDColors.LAVA_BPM;
    public static final double IN_GAP_SCORER = LEDColors.RED_LARSON_SCANNER;
    public static final double INTAKING = LEDColors.BLUE_HEARTBEAT;

    // Elevator Heights
    public static final double L1 = LEDColors.WHITE_HEARTBEAT;
    public static final double L2 = LEDColors.RED_HEARTBEAT;
    public static final double L3 = LEDColors.MEDIUM_HEARTBEAT;
    public static final double L4 = LEDColors.MEDIUM_HEARTBEAT_2;

    public static final double ALIGNED = LEDColors.RAINBOW_PARTY;

    private static int topCurrentPriority = 0;
    private static double[] patternArray = new double[4];

    public enum SubsystemPriority {
        
        ELEVATOR(3),
        CORAL(2),
        ALGAE(1),
        DEFAULT(0);

        private int priority;

        private SubsystemPriority(int priority){
            this.priority = priority;
        }

        public int get(){
            return priority;
        }
    }
    public static void setPattern(double ledPattern) {
        LED.set(ledPattern);
    }

    public static void disable() {
        LED.stopMotor();
    }

    public static void request(SubsystemPriority priority, double lightColor) {
        // update the top priority
        if (priority.get() > topCurrentPriority) {
            topCurrentPriority = priority.get();
        }
        // recording the request in the array
        patternArray[priority.get()] = lightColor;
    }

    public static void setStatus() {
        // turn light on for the top priority reqeust
        if (topCurrentPriority < patternArray.length) {
            setPattern(patternArray[topCurrentPriority]);
        }

        topCurrentPriority = SubsystemPriority.DEFAULT.get();

        SmartDashboard.putNumber("Top Priority", topCurrentPriority);
        SmartDashboard.putNumber("LED Value", patternArray[topCurrentPriority]);
        }

        @Override
        public void periodic() {
            // This method will be called once per scheduler run
            request(SubsystemPriority.DEFAULT, ENABLED);
            setStatus();
        }

}
