package frc.robot.subsystems;


import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LEDColors;
import frc.robot.utils.Ports;


public class LEDStrip extends SubsystemBase {

    private static LEDStrip instance;
    private static Spark LED = new Spark(Ports.BLINKIN_PORT);

    // Enable/Disable Colors
    public static final double DISABLED = LEDColors.END_BLEND_C1_C2;
    public static final double ENABLED = LEDColors.OCEAN_COLOR_WAVES;

    // Intake/Coral Colors 
    public static final double IN_SCORER = LEDColors.BLUE_STROBE;
    public static final double IN_GAP = LEDColors.RED_LARSON_SCANNER;
    public static final double IN_GAP_SCORER = LEDColors.RED_STROBE;
    public static final double INTAKING = LEDColors.WHITE_HEARTBEAT;
    public static final double HAVE_ALGAE = LEDColors.YELLOW; //random colors

    // Elevator Heights
    public static final double L1 = LEDColors.WHITE_HEARTBEAT;
    public static final double L2 = LEDColors.RED_HEARTBEAT;
    public static final double L3 = LEDColors.MEDIUM_HEARTBEAT;
    public static final double L4 = LEDColors.MEDIUM_HEARTBEAT_2;
    public static final double INTAKE_HEIGHT = LEDColors.WHITE_STROBE;
    public static final double MIN_HEIGHT = LEDColors.RED;
    public static final double MAX_HEIGHT = LEDColors.RED;

    // Drive Colors
    public static final double ALIGNED = LEDColors.RAINBOW_PARTY;

    private static int topCurrentPriority = 0;
    private static double[] patternArray = new double[4];

    
    // LEDStrip Singleton -- ensures only 1 LEDStrip is constructed
    public static LEDStrip getInstance() {
        if (instance == null) {
            instance = new LEDStrip();
        }
        return instance;
      }

    // Enum to determine which subsystem the light pattern is for
    public enum SubsystemPriority {
        
        ELEVATOR(1),
        CORAL(2),
        ALGAE(3),
        DEFAULT(0);

        private int priority;

        private SubsystemPriority(int priority){
            this.priority = priority;
        }

        public int get(){
            return priority;
        }
    }

    // primary method to set the lights
    public static void request(SubsystemPriority priority, double lightColor) {
        // update the top priority
        if (priority.get() > topCurrentPriority) {
            topCurrentPriority = priority.get();
        }
        // recording the request in the array
        patternArray[priority.get()] = lightColor;
    }

    public static void disable() {
        LED.stopMotor();
    }


    //---------- PRIVATE METHODS ---------//
    private static void setPattern(double ledPattern) {
        LED.set(ledPattern);
    }

    private static void setStatus() {
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
