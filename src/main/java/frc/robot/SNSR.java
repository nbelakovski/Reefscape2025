package frc.robot;


import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.utils.Ports;

public class SNSR {
    public static RelativeEncoder jawEncoder = MTR.jawMotor.getEncoder();
    public static AnalogInput scorer = new AnalogInput(Ports.ANALOG_CORAL_SENSOR_PORT);
    public static AnalogInput gap = new AnalogInput(Ports.ANALOG_GAP_SENSOR_PORT);
    
    public static double jawAngle() {
        return jawEncoder.getPosition();
    }
}
