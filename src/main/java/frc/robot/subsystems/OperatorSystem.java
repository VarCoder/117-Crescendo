package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OperatorSystem extends SubsystemBase{
    /*TODO: Integrate Buttons and Operator Board within this file */
    public PS4Controller driverController = new PS4Controller(JoystickConstants.driverControllerPort);
    public Joystick operatorController = new Joystick(JoystickConstants.buttonBoardPort);
    
    public static final class JoystickConstants{
        public static final int driverControllerPort = 0;
        public static final int buttonBoardPort = 2;
        
        public static final int translationAxis = 1;
        public static final int strafeAxis= 0;
        public static final int rotAxis = 2;
    }
}
