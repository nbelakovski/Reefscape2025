package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.utils.Ports;

public class Sensors {
    private static Sensors instance;

    private AnalogInput score;

    private Sensors() {
        score = new AnalogInput(Ports.ANALOG_CORAL_SENSOR_PORT);
    }

    public static Sensors get() {
        if (instance == null) {
            instance = new Sensors();
        }
        return instance;
    }

    public static int scorer() {
        return instance.score.getValue();
    }
}
