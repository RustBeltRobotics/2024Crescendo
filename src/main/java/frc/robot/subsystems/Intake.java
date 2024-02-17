package frc.robot.subsystems;

import static frc.robot.Constants.NEO_SECONDARY_CURRENT_LIMIT;
import static frc.robot.Constants.NEO_SMART_CURRENT_LIMIT;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private final CANSparkMax floorMotor;
    private static CANSparkMax intakeMotor;

    public Intake() {
        floorMotor = new CANSparkMax(99, MotorType.kBrushless);
        floorMotor.restoreFactoryDefaults();
        floorMotor.setIdleMode(IdleMode.kBrake);
        floorMotor.setInverted(false);
        floorMotor.setSmartCurrentLimit(NEO_SMART_CURRENT_LIMIT);
        floorMotor.setSecondaryCurrentLimit(NEO_SECONDARY_CURRENT_LIMIT);

        intakeMotor = new CANSparkMax(98, MotorType.kBrushless);
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.setInverted(false);
        intakeMotor.setSmartCurrentLimit(NEO_SMART_CURRENT_LIMIT);
        intakeMotor.setSecondaryCurrentLimit(NEO_SECONDARY_CURRENT_LIMIT);
    }
    public void runBothIntakes(double speed){
        floorMotor.set(speed);
        intakeMotor.set(speed);
    }
    public void stopBothIntakes(){
        floorMotor.set(0);
        intakeMotor.set(0);
    }
    public static void runArmIntake(double speed){
        intakeMotor.set(speed);
    }
    public static void stopArmIntake(){
        intakeMotor.set(0);
    }
    public void runFloorIntakes(double speed){
        floorMotor.set(speed);
    }
    //runs the intake for 2 seconds in order to feed note into the shooter
    public static void feedShooter(){
        runArmIntake(1);
        double startTime = System.currentTimeMillis();
        if (System.currentTimeMillis() - startTime < 2.0) { stopArmIntake(); }
    }
}
