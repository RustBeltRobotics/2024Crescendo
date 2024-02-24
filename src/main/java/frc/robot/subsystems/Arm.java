package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static frc.robot.Constants.LEFT_ROTATE;
import static frc.robot.Constants.NEO_SECONDARY_CURRENT_LIMIT;
import static frc.robot.Constants.NEO_SMART_CURRENT_LIMIT;
import static frc.robot.Constants.RIGHT_ROTATE;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private static final CANSparkMax armMotor1 = new CANSparkMax(LEFT_ROTATE, MotorType.kBrushless);;
    private static final CANSparkMax armMotor2 = new CANSparkMax(RIGHT_ROTATE, MotorType.kBrushless);;
    
    private static final DutyCycleEncoder bigEncoder = new DutyCycleEncoder(0);
    private static final Encoder medEncoder = new Encoder(2, 3);

    private final SparkPIDController arm1PidController;

    private static ShuffleboardTab diag = Shuffleboard.getTab("Diag");
    private static GenericEntry encoderEntry = diag.add("arm encoder", 0.0).getEntry();
    private double medOffset;

    private static ShuffleboardLayout pidvals = Shuffleboard.getTab("Diag")
            .getLayout("Arm PID", BuiltInLayouts.kList)
            .withSize(2, 2);
    private static GenericEntry kP = pidvals.add("akP", 7e-5)
            .getEntry();
    private static GenericEntry kI = pidvals.add("akI", 0.0)
            .getEntry();
    private static GenericEntry kD = pidvals.add("akD", 0.0)
            .getEntry();
    private static GenericEntry kIz = pidvals.add("akIz", 0.0)
            .getEntry();
    private static GenericEntry kFF = pidvals.add("akFF", 0.0)
            .getEntry();
    private static GenericEntry kMaxOutput = pidvals.add("akMaxOutput", 1)
            .getEntry();
    private static GenericEntry kMinOutput = pidvals.add("akMinOutput", -1)
            .getEntry();
    private static GenericEntry akP = pidvals.add("a_akP", 7e-5)
            .getEntry();
    private static GenericEntry akI = pidvals.add("a_akI", 0)
            .getEntry();
    private static GenericEntry akD = pidvals.add("a_akD", 0)
            .getEntry();

    PIDController anglePID = new PIDController(akP.getDouble(0), akI.getDouble(0), akD.getDouble(0));
    
    public  Arm() {
        // set motor things
        armMotor1.restoreFactoryDefaults();
        armMotor1.setIdleMode(IdleMode.kBrake);
        armMotor1.setInverted(true);
        armMotor1.setSmartCurrentLimit(NEO_SMART_CURRENT_LIMIT);
        armMotor1.setSecondaryCurrentLimit(NEO_SECONDARY_CURRENT_LIMIT);

        armMotor2.restoreFactoryDefaults();
        armMotor2.setIdleMode(IdleMode.kBrake);
        armMotor2.setInverted(false);
        armMotor2.setSmartCurrentLimit(NEO_SMART_CURRENT_LIMIT);
        armMotor2.setSecondaryCurrentLimit(NEO_SECONDARY_CURRENT_LIMIT);

        armMotor2.follow(armMotor1, true);
        medEncoder.reset();
        medOffset = bigEncoder.getAbsolutePosition();

        // set pid things
        arm1PidController = armMotor1.getPIDController();
        arm1PidController.setP(kP.getDouble(7e-5));
        arm1PidController.setI(kI.getDouble(0));
        arm1PidController.setD(kD.getDouble(0));
        arm1PidController.setIZone(kIz.getDouble(0));
        arm1PidController.setFF(kFF.getDouble(0));
        arm1PidController.setOutputRange(kMinOutput.getDouble(0), kMaxOutput.getDouble(0));
        arm1PidController.setPositionPIDWrappingEnabled(true);
    }
    public void setAngle(double angle) {
        rotate(anglePID.calculate(medEncoder.getDistance()+medOffset, angle));
    }
    public double getAngle() {
        return medEncoder.getDistance()+medOffset;
    }

    public void rotate(double speed) {
        arm1PidController.setReference(speed*5000, ControlType.kVelocity);
    }

    public void ampPose() {
        arm1PidController.setReference(Constants.AMP_POSITION, ControlType.kPosition);
    }

    public void sourcePose() {
        arm1PidController.setReference(Constants.SOURCE_POSITION, ControlType.kPosition);
    }

    public void groundPose() {
        arm1PidController.setReference(Constants.GROUND_POSITION, ControlType.kPosition);
    }

    public void stop() {
        arm1PidController.setReference(0, ControlType.kVelocity);
    }
    public void zeroBigEncoder() {
        bigEncoder.reset();
    }
    public void updateshuffle(){
        encoderEntry.setDouble(bigEncoder.getAbsolutePosition());
    }
}