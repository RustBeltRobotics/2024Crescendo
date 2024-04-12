package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static frc.robot.Constants.COMPETITION_TAB;
import static frc.robot.Constants.LEFT_ROTATE;
import static frc.robot.Constants.NEO_SECONDARY_CURRENT_LIMIT;
import static frc.robot.Constants.NEO_SMART_CURRENT_LIMIT;
import static frc.robot.Constants.RIGHT_ROTATE;
import static frc.robot.Constants.THE_PDH;

import java.util.Map;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.SpeakerAimCommand;

public class Arm extends SubsystemBase {
    private PIDController anglePIDup;
    private PIDController anglePIDdown;
    private static final CANSparkMax armMotor1 = new CANSparkMax(LEFT_ROTATE, MotorType.kBrushless);
    private static final CANSparkMax armMotor2 = new CANSparkMax(RIGHT_ROTATE, MotorType.kBrushless);
    private static RelativeEncoder throughBoreRelative = armMotor2.getAlternateEncoder(8192);
    private static AbsoluteEncoder throughBoreAbsolute = armMotor1.getAbsoluteEncoder();

    private static GenericEntry bigEncoderEntry = COMPETITION_TAB.add("abs arm encoder", 0.0)
    .withPosition(8, 1)
    .withSize(2, 1)
    .getEntry();
    private static GenericEntry medEncoderEntry = COMPETITION_TAB.add("rel arm encoder", 0.0)
    .withPosition(8, 2)
    .withSize(2,1)
    .getEntry();
    private static GenericEntry encoderWarningEntry = COMPETITION_TAB.add("!ARM ENCODER!", false)
    .withWidget("Boolean Box")
    .withPosition(8, 0)
    .withProperties(Map.of("colorWhenTrue", "red", "colorWhenFalse", "gray"))
    .getEntry();
    
    public Arm() {
        anglePIDup = new PIDController(100, 0, 0);
        anglePIDdown = new PIDController(15, 20, 0);
        
        // set motor things
        armMotor1.setIdleMode(IdleMode.kBrake);
        armMotor1.setInverted(true);
        armMotor1.setSmartCurrentLimit(NEO_SMART_CURRENT_LIMIT);
        armMotor1.setSecondaryCurrentLimit(NEO_SECONDARY_CURRENT_LIMIT);

        armMotor2.setIdleMode(IdleMode.kBrake);
        armMotor2.setInverted(false);
        armMotor2.setSmartCurrentLimit(NEO_SMART_CURRENT_LIMIT);
        armMotor2.setSecondaryCurrentLimit(NEO_SECONDARY_CURRENT_LIMIT);
        throughBoreRelative.setPositionConversionFactor(1/8192);

        armMotor2.follow(armMotor1, true);
    }
    @Override
    public void periodic() {
        updateshuffle();
    }
    public void setAngle(double angle) { 
            armMotor1.setVoltage(anglePIDup.calculate(throughBoreRelative.getPosition(), angle)); 
    }
    public void setAngleDown(double angle) { 
            armMotor1.setVoltage(anglePIDdown.calculate(throughBoreRelative.getPosition(), angle)); 
    }
    public double getAngle() {
        return throughBoreRelative.getPosition();
    }

    public void rotate(double speed) {
        armMotor1.setVoltage(speed*THE_PDH.getVoltage());
    }

    public void ampPose() {
        setAngle(Constants.AMP_POSITION);
    }

    public void groundPose() {
        setAngleDown(Constants.GROUND_POSITION);
    }
    public void stop() {
        armMotor1.setVoltage(0);
    }
    public static void zeroThroughBoreRelative() {
        throughBoreRelative.setPosition(throughBoreAbsolute.getPosition());
    }
    public void updateshuffle(){
        bigEncoderEntry.setDouble(throughBoreAbsolute.getPosition());
        medEncoderEntry.setDouble(throughBoreRelative.getPosition());
        if (throughBoreAbsolute.getPosition() == 0.0 || throughBoreRelative.getPosition() == 0.0) {
            encoderWarningEntry.setBoolean(true);
        } else {
            encoderWarningEntry.setBoolean(false);
        }
    }
    public void autoAim(){
            setAngle(SpeakerAimCommand.armAngleCalculate());
    }
}