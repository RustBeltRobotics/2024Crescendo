package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static frc.robot.Constants.ARM_FF_kG;
import static frc.robot.Constants.ARM_FF_kS;
import static frc.robot.Constants.ARM_FF_kA;
import static frc.robot.Constants.ARM_FF_kV;
import static frc.robot.Constants.LEFT_ROTATE;
import static frc.robot.Constants.NEO_SECONDARY_CURRENT_LIMIT;
import static frc.robot.Constants.NEO_SMART_CURRENT_LIMIT;
import static frc.robot.Constants.RIGHT_ROTATE;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.AprilTagAimRotateCommand;

public class Arm extends SubsystemBase {
    private PIDController anglePID;
    private ArmFeedforward angleFF;
    private static final CANSparkMax armMotor1 = new CANSparkMax(LEFT_ROTATE, MotorType.kBrushless);;
    private static final CANSparkMax armMotor2 = new CANSparkMax(RIGHT_ROTATE, MotorType.kBrushless);;
    
    private static final DutyCycleEncoder bigEncoder = new DutyCycleEncoder(2);
    //private static final Encoder medEncoder = new Encoder(2, 3);

    private double medOffset;

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
        anglePID = new PIDController(60, 10, 2);
        angleFF = new ArmFeedforward(ARM_FF_kS, ARM_FF_kG, ARM_FF_kV, ARM_FF_kA);
        
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
    }
    @Override
    public void periodic() {
        //anglePID.setPID(akP.getDouble(0), akI.getDouble(0), akD.getDouble(0));
        updateshuffle();
        //setAngle(anglesetpoint.getDouble(0.5));
    }
    public void setAngle(double angle) {
        System.out.println("setangle, " + angle);
        armMotor1.setVoltage(
            anglePID.calculate(bigEncoder.get(), angle)
        );
    }
    public double getAngle() {
        //return medEncoder.getDistance()+medOffset;
        return bigEncoder.getAbsolutePosition();
    }

    public void rotate(double speed) {
        armMotor1.setVoltage(speed*6);
        System.out.println("rotate, " + speed);
    }

    public void ampPose() {
        setAngle(Constants.AMP_POSITION);
    }

    public void groundPose() {
        setAngle(Constants.GROUND_POSITION);
    }

    public void stop() {
        armMotor1.setVoltage(0);
        System.out.println("stop");
    }
    public void zeroBigEncoder() {
        bigEncoder.reset();
    }
    public void updateshuffle(){
        bigEncoderEntry.setDouble(bigEncoder.getAbsolutePosition());
        //medEncoderEntry.setDouble(medEncoder.getDistance()+medOffset);
    }
    public void autoAim(){
        if (AprilTagAimRotateCommand.targetGood = true){
            setAngle(AprilTagAimRotateCommand.armTarget);
        }
    }
}