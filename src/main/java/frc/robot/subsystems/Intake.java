package frc.robot.subsystems;

import static frc.robot.Constants.ARM_INTAKE;
import static frc.robot.Constants.GROUND_INTAKE;
import static frc.robot.Constants.NEO_SECONDARY_CURRENT_LIMIT;
import static frc.robot.Constants.NEO_SMART_CURRENT_LIMIT;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private static final CANSparkMax floorMotor = new CANSparkMax(GROUND_INTAKE, MotorType.kBrushless);;
    private static final CANSparkMax intakeMotor = new CANSparkMax(ARM_INTAKE, MotorType.kBrushless);;
    private static DigitalInput noteSwitch = new DigitalInput(1);
    static GenericEntry limitSwitch;


    public Intake() {
        floorMotor.restoreFactoryDefaults();
        floorMotor.setIdleMode(IdleMode.kBrake);
        floorMotor.setInverted(true);
        floorMotor.setSmartCurrentLimit(NEO_SMART_CURRENT_LIMIT);
        floorMotor.setSecondaryCurrentLimit(NEO_SECONDARY_CURRENT_LIMIT);

        
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.setInverted(false);
        intakeMotor.setSmartCurrentLimit(NEO_SMART_CURRENT_LIMIT);
        intakeMotor.setSecondaryCurrentLimit(NEO_SECONDARY_CURRENT_LIMIT);
    }
    @Override
    public void periodic(){
        getSwitch();
    }
    
    public void runBothIntakes(double speed){
        floorMotor.set(speed);
        intakeMotor.set(speed);
    }
    public void stopBothIntakes(){
        floorMotor.set(0);
        intakeMotor.set(0);
    }
    public static void runArmIntake(double speed) {
        intakeMotor.set(speed);
    }
    public static void runSmartArmIntake(double speed) {
        intakeMotor.set(speed);

    }
    public static void stopArmIntake() {
        intakeMotor.set(0);
    }
    public void runFloorIntakes(double speed) {

        floorMotor.set(speed);
    }
    //runs the intake for 2 seconds in order to feed note into the shooter
    public static void feedShooter() {
        runArmIntake(1);
        double startTime = System.currentTimeMillis();
        if (System.currentTimeMillis() - startTime < 2.0) { stopArmIntake(); }
    }
    public static boolean getSwitch() {
        limitSwitch.setBoolean(!noteSwitch.get());
        return !noteSwitch.get();
    }
    public static void makeShuffleboard(){
        limitSwitch = Shuffleboard.getTab("Competition")
                .add("Limit Switch", false)
                .withWidget("Boolean Box")
                .withPosition(9, 0)
                .withProperties(Map.of("colorWhenTrue", "lime", "colorWhenFalse", "gray"))
                .getEntry();
    }
}
