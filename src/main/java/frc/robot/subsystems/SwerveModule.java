package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

import static frc.robot.Constants.*;

public class SwerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax steerMotor;

    private SparkPIDController drivePidController;
    private SparkPIDController steerPidController;
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder steerEncoder;

    private final CANcoder absoluteSteerEncoder;

    //Shuffling
        private static ShuffleboardLayout pidvals = Shuffleboard.getTab("Diag")
             .getLayout("Swerve PID", BuiltInLayouts.kList)
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
        private static GenericEntry drive_kFF =
            pidvals.add("sdrive_kFF", 0.000015)
            .getEntry();
        private static GenericEntry kMaxOutput =
            pidvals.add("skMaxOutput", 1)
            .getEntry();
        private static GenericEntry kMinOutput =
            pidvals.add("skMinOutput", -1)
            .getEntry();
        private static GenericEntry steer_kFF =
            pidvals.add("ssteer_kFF", 0.0)
            .getEntry();

    public SwerveModule(int driveID, int steerID, int encoderID) {
        // Setup drive motor SparkMax
        driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
        driveMotor.restoreFactoryDefaults();
        driveMotor.setIdleMode(IdleMode.kBrake);
        driveMotor.setInverted(true);
        driveMotor.setSmartCurrentLimit(DRIVE_SMART_CURRENT_LIMIT);
        driveMotor.setSecondaryCurrentLimit(DRIVE_SECONDARY_CURRENT_LIMIT);

        // Setup PID functionality for drive motors
        drivePidController = driveMotor.getPIDController();

        // Setup drive motor relative encoder
        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(DRIVE_POSITION_CONVERSION);
        driveEncoder.setVelocityConversionFactor(DRIVE_VELOCITY_CONVERSION);

        // Setup steer motor SparkMax
        steerMotor = new CANSparkMax(steerID, MotorType.kBrushless);
        steerMotor.restoreFactoryDefaults();
        steerMotor.setIdleMode(IdleMode.kBrake);
        steerMotor.setInverted(true);
        steerMotor.setSmartCurrentLimit(STEER_SMART_CURRENT_LIMIT);
        steerMotor.setSecondaryCurrentLimit(STEER_SECONDARY_CURRENT_LIMIT);

        // Setup PID functionality for steer motors
        steerPidController = steerMotor.getPIDController();

        // Setup steer motor relative encoder
        steerEncoder = steerMotor.getEncoder();
        steerEncoder.setPositionConversionFactor(STEER_POSITION_CONVERSION);
        steerEncoder.setVelocityConversionFactor(STEER_VELOCITY_CONVERSION);

        // Setup steer motor absolute encoder
        absoluteSteerEncoder = new CANcoder(encoderID);

        // Zero encoders to ensure steer relative matches absolute
        resetEncoders();

        //set the things
        updateSwervePid();
    }

    /** @return Drive position, meters, -inf to +inf */
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    /** @return Steer position, degrees, -inf to +inf */
    public double getSteerPosition() {
        return steerEncoder.getPosition();
    }

    /** @return Drive position, meters/second */
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    /** @return Steer position, degrees/second */
    public double getSteerVelocity() {
        return steerEncoder.getVelocity();
    }

    /** @return Absolute steer position, degrees, -inf to +inf */
    public double getAbsolutePosition() {
        return absoluteSteerEncoder.getPosition().getValueAsDouble();
    }

    /** @return Drive encoder (meters) and steer encoder (Rotation2d) positions */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(Math.toRadians(getSteerPosition())));
    }

    /**
     * Resets the drive relative encoder to 0 and steer relative encoder to match
     * absolute encoder
     */
    public void resetEncoders() {
        driveEncoder.setPosition(0.0);
        steerEncoder.setPosition(getAbsolutePosition());
    }

    /**
     * @return The module state (velocity, m/s, and steer angle, Rotation2d)
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(Math.toRadians(getSteerPosition())));
    }

    /**
     * Set's the speed and angle of an idividual module.
     * 
     * @param state the desired state (velocity, m/s, and steer angle, Rotation2d)
     */
    public void setState(SwerveModuleState state) {
        // If input is minimal, ignore input to avoid reseting steer angle to 0 degrees
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stopModule();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        drivePidController.setReference(state.speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND, CANSparkMax.ControlType.kDutyCycle);
        setSparkAngle(state.angle.getDegrees());
    }

    /**
     * Locks the wheel at the provided angle
     * 
     * @param angle degrees
     */
    public void lockModule(int angle) {
        steerPidController.setReference(angle, CANSparkMax.ControlType.kPosition);
    }

    /** Set's the voltage to both motors to 0 */
    public void stopModule() {
        driveMotor.set(0.);
        steerMotor.set(0.);
    }
    public void setSparkAngle(double targetAngleInDegrees){
        double currentSparkAngle = steerMotor.getEncoder().getPosition();
        double sparkRelativeTargetAngle = reboundValue(targetAngleInDegrees, currentSparkAngle);
        steerPidController.setReference(sparkRelativeTargetAngle, ControlType.kPosition);
    }
    public double reboundValue(double value, double anchor){
        double lowerBound = anchor - 180;
        double upperBound = anchor + 180;

        if (value < lowerBound){
            value = lowerBound + ((value-lowerBound)%(upperBound - lowerBound));
        } else if (value > upperBound){
            value = lowerBound + ((value - upperBound)%(upperBound - lowerBound));
        }
        return value;
    }
    public void updateSwervePid(){
        // set PID coefficients (drive)
        drivePidController.setP(kP.getDouble(0));
        drivePidController.setI(kI.getDouble(0));
        drivePidController.setD(kD.getDouble(0));
        drivePidController.setIZone(kIz.getDouble(0));
        drivePidController.setFF(drive_kFF.getDouble(0));
        drivePidController.setOutputRange(kMinOutput.getDouble(0), kMaxOutput.getDouble(0));
        drivePidController.setPositionPIDWrappingEnabled(true);

        // set PID coefficients (steer)
        steerPidController.setP(STEER_P);
        steerPidController.setI(STEER_I);
        steerPidController.setD(STEER_D);
        steerPidController.setIZone(kIz.getDouble(0));
        steerPidController.setFF(steer_kFF.getDouble(0));
        steerPidController.setOutputRange(kMinOutput.getDouble(0), kMaxOutput.getDouble(1));
        steerPidController.setPositionPIDWrappingEnabled(true);
        steerPidController.setPositionPIDWrappingMaxInput(360);
        steerPidController.setPositionPIDWrappingMinInput(0);
    }
}