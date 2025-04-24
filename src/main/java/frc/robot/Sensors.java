package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.utils.Ports;

public class Sensors {
    private static Sensors instance;

    private AnalogInput scorer;
    private AnalogInput gap;

    private Sensors() {
        scorer = new AnalogInput(Ports.ANALOG_CORAL_SENSOR_PORT);
        gap = new AnalogInput(Ports.ANALOG_GAP_SENSOR_PORT);
    }

    public static Sensors get() {
        if (instance == null) {
            instance = new Sensors();
        }
        return instance;
    }

    public static int scorer() {
        return instance.scorer.getValue();
    }

    public static int gap() {
        return instance.gap.getValue();
    }
}
