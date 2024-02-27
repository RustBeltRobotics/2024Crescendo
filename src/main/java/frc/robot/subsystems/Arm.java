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
    private PIDController anglePID;
    private PIDController gravAnglePID;
    private static final CANSparkMax armMotor1 = new CANSparkMax(LEFT_ROTATE, MotorType.kBrushless);;
    private static final CANSparkMax armMotor2 = new CANSparkMax(RIGHT_ROTATE, MotorType.kBrushless);;
    
    private static final DutyCycleEncoder bigEncoder = new DutyCycleEncoder(0);
    private static final Encoder medEncoder = new Encoder(2, 3);

    private final SparkPIDController arm1PidController;
    private double medOffset;

    private static ShuffleboardTab diag = Shuffleboard.getTab("Diag");
    private static GenericEntry bigEncoderEntry = diag.add("abs arm encoder", 0.0).getEntry();
    private static GenericEntry medEncoderEntry = diag.add("rel arm encoder", 0.0).getEntry();
    private static ShuffleboardLayout pidvals = Shuffleboard.getTab("Diag")
            .getLayout("Arm PID", BuiltInLayouts.kList)
            .withSize(2, 5);
    private static GenericEntry kP = pidvals.add("spark_P", 7e-5)
            .getEntry();
    private static GenericEntry kI = pidvals.add("spark_I", 0.0)
            .getEntry();
    private static GenericEntry kD = pidvals.add("spark_D", 0.0)
            .getEntry();
    private static GenericEntry kIz = pidvals.add("spark_Iz", 0.0)
            .getEntry();
    private static GenericEntry kFF = pidvals.add("spark_FF", 0.0)
            .getEntry();
    private static GenericEntry kMaxOutput = pidvals.add("spark_MaxOutput", 1)
            .getEntry();
    private static GenericEntry kMinOutput = pidvals.add("spark_MinOutput", -1)
            .getEntry();
    private static GenericEntry akP = pidvals.add("angle_P", 20)
            .getEntry();
    private static GenericEntry akI = pidvals.add("angle_I", 0)
            .getEntry();
    private static GenericEntry akD = pidvals.add("angle_D", 10)
            .getEntry();
    private static GenericEntry agkP = pidvals.add("grav_angle_P", 5)
            .getEntry();
    private static GenericEntry agkI = pidvals.add("grav_angle_I", 0)
            .getEntry();
    private static GenericEntry agkD = pidvals.add("grav_angle_D", 10)
            .getEntry();
    
    public Arm() {
        anglePID = new PIDController(0, 0, 0);
        gravAnglePID = new PIDController(0, 0, 0);
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
        medEncoder.setDistancePerPulse(90);//experimental

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
    @Override
    public void periodic() {
        anglePID.setPID(akP.getDouble(0), akI.getDouble(0), akD.getDouble(0));
        gravAnglePID.setPID(agkP.getDouble(5), agkI.getDouble(0), agkD.getDouble(0));
    }
    public void setAngle(double angle) {
        if (angle > bigEncoder.getAbsolutePosition()){
            rotate(anglePID.calculate(bigEncoder.get(), angle));
        } else {
            rotate(gravAnglePID.calculate(bigEncoder.get(), angle));
        }
    }
    public double getAngle() {
        //return medEncoder.getDistance()+medOffset;
        return bigEncoder.getAbsolutePosition();
    }

    public void rotate(double speed) {
        arm1PidController.setReference(speed*5000, ControlType.kVelocity);
        System.out.println(speed);
    }

    public void ampPose() {
        setAngle(Constants.AMP_POSITION);
    }

    public void groundPose() {
        setAngle(Constants.GROUND_POSITION);
    }

    public void stop() {
        arm1PidController.setReference(0, ControlType.kVelocity);
    }
    public void zeroBigEncoder() {
        bigEncoder.reset();
    }
    public void updateshuffle(){
        bigEncoderEntry.setDouble(bigEncoder.getAbsolutePosition());
        medEncoderEntry.setDouble(medEncoder.getDistance()+medOffset);
    }
}