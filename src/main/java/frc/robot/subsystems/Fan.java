package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem to control fan(s) for use in opening the Trap hatch at the end of Teleop - never fully implemented or used in competition
 */
public class Fan extends SubsystemBase {
    private final PWM fan1 = new PWM(0);
    private final PWM fan2 = new PWM(1); 
    private final PWM fan3 = new PWM(2); 

    public void fanOn(){
        fan1.setSpeed(1);
        fan2.setSpeed(1);
        fan3.setSpeed(1);
    }
    public void fanStop(){
        fan1.setSpeed(0);
        fan2.setSpeed(0);
        fan3.setSpeed(0);
    }
}