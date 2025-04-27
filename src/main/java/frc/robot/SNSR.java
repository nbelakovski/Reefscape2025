package frc.robot;

import com.revrobotics.RelativeEncoder;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.utils.Ports;

public class SNSR {
    public static AnalogInput     scorer     = new AnalogInput(Ports.ANALOG_CORAL_SENSOR_PORT);
    public static AnalogInput     gap        = new AnalogInput(Ports.ANALOG_GAP_SENSOR_PORT);
    public static RelativeEncoder jawEncoder = MTR.jawMotor.getEncoder();
    public static AHRS            navX       = new AHRS(NavXComType.kMXP_SPI);

    public static void configureSensors() {
        jawEncoder.setPosition(0);
    }
}
