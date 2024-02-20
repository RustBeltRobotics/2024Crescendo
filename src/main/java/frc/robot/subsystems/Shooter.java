package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static frc.robot.Constants.LEFT_SHOOTER;
import static frc.robot.Constants.NEO_SECONDARY_CURRENT_LIMIT;
import static frc.robot.Constants.NEO_SMART_CURRENT_LIMIT;
import static frc.robot.Constants.RIGHT_SHOOTER;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private final CANSparkMax shooterMotor1;
    private final CANSparkMax shooterMotor2;
    private static SparkPIDController shooter1PidController;
    // private final SparkPIDController shooter2PidController;    
    private static ShuffleboardLayout pidvals = Shuffleboard.getTab("Diag")
        .getLayout("Shooter PID", BuiltInLayouts.kList)
        .withSize(2, 2);
    private static GenericEntry kP =
        pidvals.add("shkP", 7e-5)
        .getEntry();
    private static GenericEntry kI =
        pidvals.add("shkI", 0.0)
        .getEntry();
    private static GenericEntry kD =
        pidvals.add("shkD", 0.0)
        .getEntry();
    private static GenericEntry kIz =
        pidvals.add("shkIz", 0.0)
        .getEntry();
    private static GenericEntry kFF =
        pidvals.add("shkFF", 0.0)
        .getEntry();
    private static GenericEntry kMaxOutput =
        pidvals.add("shkMaxOutput", 1)
        .getEntry();
    private static GenericEntry kMinOutput =
        pidvals.add("shkMinOutput", -1)
        .getEntry();

    public Shooter(){
        //set motor things
        shooterMotor1 = new CANSparkMax(LEFT_SHOOTER, MotorType.kBrushless);
        shooterMotor1.restoreFactoryDefaults();
        shooterMotor1.setIdleMode(IdleMode.kCoast);
        shooterMotor1.setInverted(false);
        shooterMotor1.setSmartCurrentLimit(NEO_SMART_CURRENT_LIMIT);
        shooterMotor1.setSecondaryCurrentLimit(NEO_SECONDARY_CURRENT_LIMIT);

        shooterMotor2 = new CANSparkMax(RIGHT_SHOOTER, MotorType.kBrushless);
        shooterMotor2.restoreFactoryDefaults();
        shooterMotor2.setIdleMode(IdleMode.kCoast);
        shooterMotor2.setInverted(false);
        shooterMotor2.setSmartCurrentLimit(NEO_SMART_CURRENT_LIMIT);
        shooterMotor2.setSecondaryCurrentLimit(NEO_SECONDARY_CURRENT_LIMIT);
        shooterMotor2.follow(shooterMotor1, false); //TODO: check

        //set pid things
        shooter1PidController = shooterMotor1.getPIDController();
        shooter1PidController.setP(kP.getDouble(7e-5));
        shooter1PidController.setI(kI.getDouble(0));
        shooter1PidController.setD(kD.getDouble(0));
        shooter1PidController.setIZone(kIz.getDouble(0));
        shooter1PidController.setFF(kFF.getDouble(0));
        shooter1PidController.setOutputRange(kMinOutput.getDouble(-1), kMaxOutput.getDouble(1));
        shooter1PidController.setPositionPIDWrappingEnabled(true);

        // shooter2PidController = shooterMotor2.getPIDController();
        // shooter2PidController.setP(kP);
        // shooter2PidController.setI(kI);
        // shooter2PidController.setD(kD);
        // shooter2PidController.setIZone(kIz);
        // shooter2PidController.setFF(kFF);
        // shooter2PidController.setOutputRange(kMinOutput, kMaxOutput);
        // shooter2PidController.setPositionPIDWrappingEnabled(true);
    }
    public static void spool(double velocity){
        shooter1PidController.setReference(velocity, ControlType.kVelocity);
        // shooter2PidController.setReference(velocity, ControlType.kVelocity);
    }
    public static void stop(){
        shooter1PidController.setReference(0, ControlType.kVelocity);
        // shooter2PidController.setReference(0, ControlType.kVelocity);
    }
}
