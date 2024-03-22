package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import static frc.robot.Constants.LEFT_CLIMB;
import static frc.robot.Constants.NEO_SECONDARY_CURRENT_LIMIT;
import static frc.robot.Constants.NEO_SMART_CURRENT_LIMIT;
import static frc.robot.Constants.RIGHT_CLIMB;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private final CANSparkMax climberMotor1;
    private final CANSparkMax climberMotor2;

    public Climber(){
        //set motor things
        climberMotor1 = new CANSparkMax(LEFT_CLIMB, MotorType.kBrushed);
        climberMotor1.restoreFactoryDefaults();
        climberMotor1.setIdleMode(IdleMode.kBrake);
        climberMotor1.setInverted(false);
        climberMotor1.setSmartCurrentLimit(NEO_SMART_CURRENT_LIMIT);
        climberMotor1.setSecondaryCurrentLimit(NEO_SECONDARY_CURRENT_LIMIT);

        climberMotor2 = new CANSparkMax(RIGHT_CLIMB, MotorType.kBrushed);
        climberMotor2.restoreFactoryDefaults();
        climberMotor2.setIdleMode(IdleMode.kBrake);
        climberMotor2.setInverted(false);
        climberMotor2.setSmartCurrentLimit(NEO_SMART_CURRENT_LIMIT);
        climberMotor2.setSecondaryCurrentLimit(NEO_SECONDARY_CURRENT_LIMIT);

        climberMotor2.follow(climberMotor1, false);
    }
    public void climb(double speed){
        climberMotor1.set(speed);
    }
    public void stop(){
        climberMotor1.set(0);
    }
}