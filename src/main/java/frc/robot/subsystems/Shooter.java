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

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private static CANSparkMax shooterMotor1;
    private static CANSparkMax shooterMotor2;
    private static SparkPIDController shooter1PidController;

    public Shooter(){
        shooterMotor1 = new CANSparkMax(LEFT_SHOOTER, MotorType.kBrushless);
        shooterMotor1.restoreFactoryDefaults();
        shooterMotor1.setIdleMode(IdleMode.kBrake);
        shooterMotor1.setInverted(false);
        shooterMotor1.setSmartCurrentLimit(NEO_SMART_CURRENT_LIMIT);
        shooterMotor1.setSecondaryCurrentLimit(NEO_SECONDARY_CURRENT_LIMIT);

        shooterMotor2 = new CANSparkMax(RIGHT_SHOOTER, MotorType.kBrushless);
        shooterMotor2.restoreFactoryDefaults();
        shooterMotor2.setIdleMode(IdleMode.kBrake);
        shooterMotor2.setInverted(false);
        shooterMotor2.setSmartCurrentLimit(NEO_SMART_CURRENT_LIMIT);
        shooterMotor2.setSecondaryCurrentLimit(NEO_SECONDARY_CURRENT_LIMIT);
        shooterMotor2.follow(shooterMotor1, false);

        shooter1PidController = shooterMotor1.getPIDController();
        shooter1PidController.setP(1.0);
        shooter1PidController.setI(0.0);
        shooter1PidController.setD(0.0);
        shooter1PidController.setIZone(0.0);
        shooter1PidController.setFF(0.0);
        shooter1PidController.setOutputRange(-1.0, 1.0); 
    }
    public static double getShooterVelocity() {
        return shooterMotor1.getEncoder().getVelocity();
    }
    

    public static void spool(double velocity){
        // FIXME: We should figure out why we need a value of 4 here.
        // I have a feeling the issue is not something unique to the shooter, so if we
        // don't understand it, it could be an issue elsewhere as well and we're
        // currently unaware.
        // REV hardware client might be able to shed some insight
        shooter1PidController.setReference(velocity*4, ControlType.kVelocity);
    }
    public static void stop(){
        shooter1PidController.setReference(0., ControlType.kVoltage);
    }
    public static boolean stopped() {
        if (getShooterVelocity() == 0.0) {
            return true;
        } else {
            return false;
        }
    }
}
