package frc.robot.utils;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class TriggerButton extends Trigger{

    private XboxController controller;
    private int axis;
    public static final String Left = "LEFT";
    public static final String Right = "RIGHT";


    public TriggerButton(XboxController controller, int axis){
        super(()-> controller.getRawAxis(axis) >0.7 ); // Number can change
        this.controller = controller;
        this.axis = axis;
    }
}
