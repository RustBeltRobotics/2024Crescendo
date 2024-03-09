package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static frc.robot.Constants.LEFT_ROTATE;
import static frc.robot.Constants.NEO_SECONDARY_CURRENT_LIMIT;
import static frc.robot.Constants.NEO_SMART_CURRENT_LIMIT;
import static frc.robot.Constants.RIGHT_ROTATE;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.AprilTagAimCommand;

public class Arm extends SubsystemBase {
    private PIDController anglePIDup;
    private PIDController anglePIDdown;
    private static final CANSparkMax armMotor1 = new CANSparkMax(LEFT_ROTATE, MotorType.kBrushless);;
    private static final CANSparkMax armMotor2 = new CANSparkMax(RIGHT_ROTATE, MotorType.kBrushless);;
    private static final RelativeEncoder throughBoreRelative = armMotor2.getAlternateEncoder(8192);

    
    private static final DutyCycleEncoder bigEncoder = new DutyCycleEncoder(2);
    //private static final Encoder medEncoder = new Encoder(2, 3);


    private static ShuffleboardTab diag = Shuffleboard.getTab("Diag");
    private static GenericEntry bigEncoderEntry = diag.add("abs arm encoder", 0.0).getEntry();
    private static GenericEntry medEncoderEntry = diag.add("rel arm encoder", 0.0).getEntry();
    private static GenericEntry anglesetpoint = diag.add("arm angle control", 0.5).getEntry();
    private static ShuffleboardLayout pidvals = Shuffleboard.getTab("Diag")
            .getLayout("Arm PID", BuiltInLayouts.kList)
            .withSize(2, 3)
            .withPosition(3, 2);
    private static GenericEntry akP = pidvals.add("P", 10)
            .getEntry();
    private static GenericEntry akI = pidvals.add("I", 0.0)
            .getEntry();
    private static GenericEntry akD = pidvals.add("D", 0.0)
            .getEntry();
    private static GenericEntry affkA = pidvals.add("FF_kA", 0)
            .getEntry();
    
    public Arm() {
        anglePIDup = new PIDController(60, 10, 2);
        anglePIDdown = new PIDController(30, 10, 2);
        
        // set motor things
        armMotor1.setIdleMode(IdleMode.kBrake);
        armMotor1.setInverted(true);
        armMotor1.setSmartCurrentLimit(NEO_SMART_CURRENT_LIMIT);
        armMotor1.setSecondaryCurrentLimit(NEO_SECONDARY_CURRENT_LIMIT);

        throughBoreRelative.setPositionConversionFactor(1/8192);

        armMotor2.setIdleMode(IdleMode.kBrake);
        armMotor2.setInverted(false);
        armMotor2.setSmartCurrentLimit(NEO_SMART_CURRENT_LIMIT);
        armMotor2.setSecondaryCurrentLimit(NEO_SECONDARY_CURRENT_LIMIT);

        armMotor2.follow(armMotor1, true);
    }
    @Override
    public void periodic() {
        //anglePID.setPID(akP.getDouble(0), akI.getDouble(0), akD.getDouble(0));
        updateshuffle();
        if (DriverStation.isTestEnabled()) {
            setAngle(anglesetpoint.getDouble(0.5));
        }
    }
    public void setAngle(double angle) { 
        if (angle > throughBoreRelative.getPosition()) {
            armMotor1.setVoltage(anglePIDup.calculate(throughBoreRelative.getPosition(), angle)); 
        } else {
            armMotor1.setVoltage(anglePIDdown.calculate(throughBoreRelative.getPosition(), angle)); 
        }
        SmartDashboard.putNumber("setAngle", angle);
    }

    public double getAngle() {
        return throughBoreRelative.getPosition();
    }

    public void rotate(double speed) {
        armMotor1.setVoltage(speed*6);
    }

    public void ampPose() {
        setAngle(Constants.AMP_POSITION);
    }

    public void groundPose() {
        setAngle(Constants.GROUND_POSITION);
    }

    public void stop() {
        armMotor1.setVoltage(0);
    }
    public static void zeroThroughBoreRelative() {
        System.out.println("bigenc: " + bigEncoder.getAbsolutePosition());
    }
    public void updateshuffle(){
        bigEncoderEntry.setDouble(bigEncoder.getAbsolutePosition());
        medEncoderEntry.setDouble(throughBoreRelative.getPosition());
    }
    public void autoAim(){
        if (AprilTagAimCommand.getTargetGood()) {
            setAngle(AprilTagAimCommand.armAngleCalculate());
        }
    }
}