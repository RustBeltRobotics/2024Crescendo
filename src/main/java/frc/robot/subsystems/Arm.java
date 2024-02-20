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

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private final CANSparkMax armMotor1;
    private final CANSparkMax armMotor2;
    private final Encoder arm1AbsEncoder;
    private final SparkPIDController arm1PidController;
    // private final SparkPIDController arm2PidController;

    private static ShuffleboardLayout pidvals = Shuffleboard.getTab("Diag")
        .getLayout("Arm PID", BuiltInLayouts.kList)
        .withSize(2, 2);
    private static GenericEntry kP =
        pidvals.add("skP", 7e-5)
        .getEntry();
    private static GenericEntry kI =
        pidvals.add("skI", 0.0)
        .getEntry();
    private static GenericEntry kD =
        pidvals.add("skD", 0.0)
        .getEntry();
    private static GenericEntry kIz =
        pidvals.add("skIz", 0.0)
        .getEntry();
    private static GenericEntry kFF =
        pidvals.add("sdrive_kFF", 0.0)
        .getEntry();
    private static GenericEntry kMaxOutput =
        pidvals.add("skMaxOutput", 1)
        .getEntry();
    private static GenericEntry kMinOutput =
        pidvals.add("skMinOutput", -1)
        .getEntry();

    public Arm(){
        //set motor things
        armMotor1 = new CANSparkMax(LEFT_ROTATE, MotorType.kBrushless);
        armMotor1.restoreFactoryDefaults();
        armMotor1.setIdleMode(IdleMode.kBrake);
        armMotor1.setInverted(true);
        armMotor1.setSmartCurrentLimit(NEO_SMART_CURRENT_LIMIT);
        armMotor1.setSecondaryCurrentLimit(NEO_SECONDARY_CURRENT_LIMIT);

        armMotor2 = new CANSparkMax(RIGHT_ROTATE, MotorType.kBrushless);
        armMotor2.restoreFactoryDefaults();
        armMotor2.setIdleMode(IdleMode.kBrake);
        armMotor2.setInverted(false);
        armMotor2.setSmartCurrentLimit(NEO_SMART_CURRENT_LIMIT);
        armMotor2.setSecondaryCurrentLimit(NEO_SECONDARY_CURRENT_LIMIT);

        armMotor2.follow(armMotor1, true); //TODO: check

        arm1AbsEncoder = new Encoder(0,1);

        //set pid things
        arm1PidController = armMotor1.getPIDController();
        arm1PidController.setP(kP.getDouble(7e-5));
        arm1PidController.setI(kI.getDouble(0));
        arm1PidController.setD(kD.getDouble(0));
        arm1PidController.setIZone(kIz.getDouble(0));
        arm1PidController.setFF(kFF.getDouble(0));
        arm1PidController.setOutputRange(kMinOutput.getDouble(0), kMaxOutput.getDouble(0));
        arm1PidController.setPositionPIDWrappingEnabled(true);

        // arm2PidController = armMotor2.getPIDController();
        // arm2PidController.setP(kP);
        // arm2PidController.setI(kI);
        // arm2PidController.setD(kD);
        // arm2PidController.setIZone(kIz);
        // arm2PidController.setFF(kFF);
        // arm2PidController.setOutputRange(kMinOutput, kMaxOutput);
        // arm2PidController.setPositionPIDWrappingEnabled(true);
    }
    public void spinShooter(double velocity){
        arm1PidController.setReference(velocity, ControlType.kVelocity);
        // arm2PidController.setReference(velocity, ControlType.kVelocity);
    }
    public void stopShooter(){
        arm1PidController.setReference(0, ControlType.kVelocity);
        // arm2PidController.setReference(0, ControlType.kVelocity);
    }
}
