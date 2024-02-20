package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static frc.robot.Constants.LEFT_CLIMB;
import static frc.robot.Constants.NEO_SECONDARY_CURRENT_LIMIT;
import static frc.robot.Constants.NEO_SMART_CURRENT_LIMIT;
import static frc.robot.Constants.RIGT_CLIMB;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private final CANSparkMax climberMotor1;
    private final CANSparkMax climberMotor2;
    private final SparkPIDController climber1PidController;
    // private final SparkPIDController shooter2PidController;

    private static ShuffleboardLayout pidvals = Shuffleboard.getTab("Diag")
        .getLayout("Climber PID", BuiltInLayouts.kList)
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

    public Climber(){
        //set motor things
        climberMotor1 = new CANSparkMax(LEFT_CLIMB, MotorType.kBrushless);
        climberMotor1.restoreFactoryDefaults();
        climberMotor1.setIdleMode(IdleMode.kBrake);
        climberMotor1.setInverted(false);
        climberMotor1.setSmartCurrentLimit(NEO_SMART_CURRENT_LIMIT);
        climberMotor1.setSecondaryCurrentLimit(NEO_SECONDARY_CURRENT_LIMIT);

        climberMotor2 = new CANSparkMax(RIGT_CLIMB, MotorType.kBrushless);
        climberMotor2.restoreFactoryDefaults();
        climberMotor2.setIdleMode(IdleMode.kBrake);
        climberMotor2.setInverted(false);
        climberMotor2.setSmartCurrentLimit(NEO_SMART_CURRENT_LIMIT);
        climberMotor2.setSecondaryCurrentLimit(NEO_SECONDARY_CURRENT_LIMIT);
        climberMotor2.follow(climberMotor1, false); //TODO: check

        //set pid things
        climber1PidController = climberMotor1.getPIDController();
        climber1PidController.setP(kP.getDouble(7e-5));
        climber1PidController.setI(kI.getDouble(0));
        climber1PidController.setD(kD.getDouble(0));
        climber1PidController.setIZone(kIz.getDouble(0));
        climber1PidController.setFF(kFF.getDouble(0));
        climber1PidController.setOutputRange(kMinOutput.getDouble(-1), kMaxOutput.getDouble(1));
        climber1PidController.setPositionPIDWrappingEnabled(true);

        // shooter2PidController = climberMotor2.getPIDController();
        // shooter2PidController.setP(kP);
        // shooter2PidController.setI(kI);
        // shooter2PidController.setD(kD);
        // shooter2PidController.setIZone(kIz);
        // shooter2PidController.setFF(kFF);
        // shooter2PidController.setOutputRange(kMinOutput, kMaxOutput);
        // shooter2PidController.setPositionPIDWrappingEnabled(true);
    }
    public void spinShooter(double velocity){
        climber1PidController.setReference(velocity, ControlType.kVelocity);
        // shooter2PidController.setReference(velocity, ControlType.kVelocity);
    }
    public void stopShooter(){
        climber1PidController.setReference(0, ControlType.kVelocity);
        // shooter2PidController.setReference(0, ControlType.kVelocity);
    }
}
