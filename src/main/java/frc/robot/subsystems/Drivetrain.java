package frc.robot.subsystems;

import static frc.robot.Constants.BACK_LEFT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.BACK_LEFT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.BACK_LEFT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.BACK_RIGHT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.BACK_RIGHT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.FRONT_LEFT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.FRONT_LEFT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.FRONT_RIGHT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.FRONT_RIGHT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.KINEMATICS;
import static frc.robot.Constants.MAX_VELOCITY_METERS_PER_SECOND;
import static frc.robot.Constants.STEER_D;
import static frc.robot.Constants.STEER_I;
import static frc.robot.Constants.STEER_P;
import static frc.robot.Constants.LL_NAME;
import static frc.robot.Constants.rotation_D;
import static frc.robot.Constants.rotation_I;
import static frc.robot.Constants.rotation_P;
import static frc.robot.Constants.translation_D;
import static frc.robot.Constants.translation_I;
import static frc.robot.Constants.translation_P;

import java.util.Map;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.AprilTagAimCommand;
import frc.robot.util.LimelightHelpers;

public class Drivetrain extends SubsystemBase {
    private String theMove;
    // NavX connected over MXP
    public final AHRS navx;

    /**
     * For user to reset zero for "forward" on the robot while maintaining absolute
     * field zero for odometry
     */
    private double gyroOffset;

    // These are our modules. We initialize them in the constructor.
    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    // The speed of the robot in x and y translational velocities and rotational velocity
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    // Boolean statement to control locking the wheels in an X-position
    private boolean wheelsLocked = false;

    private final SwerveDrivePoseEstimator poseEstimator;

    private SwerveModuleState[] states;

    // The Shuffle
    ShuffleboardTab comp = Shuffleboard.getTab("Competition");
    ShuffleboardLayout drivetrainLayout = Shuffleboard.getTab("Competition")
            .getLayout("Drivetrain", BuiltInLayouts.kList)
            .withSize(1, 2)
            .withPosition(0, 2);
    private final GenericEntry FRA = drivetrainLayout.add("Front Right Absolute", 0)
            .getEntry();
    private final GenericEntry FLA = drivetrainLayout.add(
            "Front Left Absolute", 0)
            .getEntry();
    private final GenericEntry BRA = drivetrainLayout.add("Back Right Absolute", 0)
            .getEntry();
    private final GenericEntry BLA = drivetrainLayout.add("Back Left Absolute", 0)
            .getEntry();
    private final GenericEntry BLV = drivetrainLayout.add("Back Left Velocity", 0)
            .getEntry();
    private final GenericEntry BRV = drivetrainLayout.add("Back Right Velocity", 0)
            .getEntry();
    private final GenericEntry FLV = drivetrainLayout.add("Front Left Velocity", 0)
            .getEntry();
    private final GenericEntry FRV = drivetrainLayout.add("Front Right Velocity", 0)
            .getEntry();
    private final GenericEntry Gyro = comp.add("Gryoscope Angle", 0)
            .withWidget(BuiltInWidgets.kGyro)
            .withPosition(1, 3)
            .getEntry();
    private final GenericEntry defaultEntry = Shuffleboard.getTab("Competition")
            .add("Center", false)
            .withWidget("Boolean Box")
            .withPosition(2, 0)
            .withProperties(Map.of("colorWhenTrue", "orange", "colorWhenFalse", "grey"))
            .getEntry();
    private final GenericEntry FLEntry = Shuffleboard.getTab("Competition")
            .add("FL", false)
            .withWidget("Boolean Box")
            .withPosition(1, 0)
            .withProperties(Map.of("colorWhenTrue", "orange", "colorWhenFalse", "grey"))
            .getEntry();
    private final GenericEntry FREntry = Shuffleboard.getTab("Competition")
            .add("FR", false)
            .withWidget("Boolean Box")
            .withPosition(3, 0)
            .withProperties(Map.of("colorWhenTrue", "orange", "colorWhenFalse", "grey"))
            .getEntry();

    //networktables publisher for advantagescope swerve visualization
    private final StructArrayPublisher<SwerveModuleState> statePublisher;
    //networktables publisher for advantagescope 2d pose visualization
    StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("MyPose", Pose2d.struct).publish();

    public Drivetrain() {
        // Start publishing an array of module states with the "/SwerveStates" key
        statePublisher = NetworkTableInstance.getDefault()
                .getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();

         // Configure AutoBuilder last
        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::autoDrive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(translation_P, translation_I, translation_D), // Translation PID constants
                        new PIDConstants(rotation_P, rotation_I, rotation_D), // Rotation PID constants
                        Constants.MAX_VELOCITY_METERS_PER_SECOND, // Max module speed, in m/s
                        Constants.DRIVETRAIN_BASE_RADIUS, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig()
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );

        // Initialize all modules
        backRightModule = new SwerveModule(
                BACK_RIGHT_MODULE_DRIVE_MOTOR,
                BACK_RIGHT_MODULE_STEER_MOTOR,
                BACK_RIGHT_MODULE_STEER_ENCODER);

        backLeftModule = new SwerveModule(
                BACK_LEFT_MODULE_DRIVE_MOTOR,
                BACK_LEFT_MODULE_STEER_MOTOR,
                BACK_LEFT_MODULE_STEER_ENCODER);

        frontRightModule = new SwerveModule(
                FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                FRONT_RIGHT_MODULE_STEER_MOTOR,
                FRONT_RIGHT_MODULE_STEER_ENCODER);

        frontLeftModule = new SwerveModule(
                FRONT_LEFT_MODULE_DRIVE_MOTOR,
                FRONT_LEFT_MODULE_STEER_MOTOR,
                FRONT_LEFT_MODULE_STEER_ENCODER);

        // Zero all relative encoders
        frontLeftModule.resetEncoders();
        frontRightModule.resetEncoders();
        backLeftModule.resetEncoders();
        backRightModule.resetEncoders();

        // Initialize and zero gyro
        navx = new AHRS(SPI.Port.kMXP);
        zeroGyroscope();

        poseEstimator = new SwerveDrivePoseEstimator(KINEMATICS, getGyroscopeRotation(), getSwerveModulePositions(),
                new Pose2d());

        theMove = "default";
        Shuffleboard.getTab("Diag").add(new InstantCommand(() -> updatePIDs()));
    }

    private void updatePIDs(){
        frontLeftModule.updatePIDs();
        frontRightModule.updatePIDs();
        backLeftModule.updatePIDs();
        backRightModule.updatePIDs();
    }

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the
     * robot is currently facing to the 'forwards' direction.
     */
    public void zeroGyroscope() {
        gyroOffset = -getGyroscopeAngle(); //we were calling the getgyrosocpe function shich was also negated
    }

    public double getGyroOffset() {
        return gyroOffset;
    }

    /**
     * Toggles whether or not the wheels are locked. If they are, the wheels are
     * crossed into an X pattern and any other drive input has no effect.
     */
    public void toggleWheelsLocked() {
        wheelsLocked = !wheelsLocked;
    }

    public double getGyroscopeAngle() {
        return -navx.getYaw(); // -180 to 180, 0 degres is forward, ccw is +
    }

    // Returns the measurment of the gyroscope yaw. Used for field-relative drive
    public Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(getGyroscopeAngle());
    }

    /** @return Pitch in degrees, -180 to 180 */
    public double getPitch() {
        return navx.getPitch();
    }

    /** @return Roll in degrees, -180 to 180 */
    public double getRoll() {
        return navx.getRoll();
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[] {
                frontLeftModule.getPosition(), frontRightModule.getPosition(), backLeftModule.getPosition(),
                backRightModule.getPosition()
        };
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(getGyroscopeRotation(), getSwerveModulePositions(), pose);
    }

    public void updateOdometry() {
        poseEstimator.update(getGyroscopeRotation(), getSwerveModulePositions());
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
        return chassisSpeeds;
    }

    public void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
        frontLeftModule.setState(states[0]);
        frontRightModule.setState(states[1]);
        backLeftModule.setState(states[2]);
        backRightModule.setState(states[3]);
    }

    /**
     * Decide where the center of rotation is going to be based on function call (we got the moves)
     **/
    public void setMoves(String theMove){
        this.theMove = theMove;
    }

    /**
     * Used to drive the robot with the provided ChassisSpeed object.
     * 
     * @param chassisSpeeds The translational and rotational velocities at which to
     *                      drive the robot.
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;
    }
    public void autoDrive(ChassisSpeeds chassisSpeeds) {
        if (AprilTagAimCommand.validTID){
            this.chassisSpeeds = new ChassisSpeeds(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, AprilTagAimCommand.AUTO_TX);
        } else { 
            this.chassisSpeeds = chassisSpeeds; 
        }
    }

    /**
     * This method is run every 20 ms.
     * <p>
     * This method is used to command the individual module states based off the
     * ChassisSpeeds object
     */
    @Override
    public void periodic() {
        // TODO: Consider moving the vision stuff to a Vision subsystem class, and passing the pose estimation info back to drivetrain
        var alliance = DriverStation.getAlliance();
        Pose2d visionPose2d;
        
        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Red) {
                visionPose2d = LimelightHelpers.getBotPose2d_wpiRed(LL_NAME);
            } else {
                visionPose2d = LimelightHelpers.getBotPose2d_wpiBlue(LL_NAME);
            }
        } else {
            visionPose2d = LimelightHelpers.getBotPose2d(LL_NAME);
        }

        double totalVisionLatencyMs = LimelightHelpers.getLatency_Capture(LL_NAME);
        totalVisionLatencyMs += LimelightHelpers.getLatency_Pipeline(LL_NAME);

        double poseReadingTimestamp = Timer.getFPGATimestamp() - (totalVisionLatencyMs / 1000.0);
        
		double poseDifference = poseEstimator.getEstimatedPosition().getTranslation()
        .getDistance(visionPose2d.getTranslation());

        if (visionPose2d.getX() != 0.0 && poseDifference < 0.5) {
            poseEstimator.addVisionMeasurement(visionPose2d, poseReadingTimestamp);
        }

        handleMoves();

        handleLocked();

        updateOdometry();

        //updateTelemetry();
    }

    private void handleMoves() {
        switch (theMove){
            case "FL":
                states = KINEMATICS.toSwerveModuleStates(chassisSpeeds, new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2., Constants.DRIVETRAIN_WHEELBASE_METERS / 2.));
                defaultEntry.setBoolean(false);
                FLEntry.setBoolean(true);
                FREntry.setBoolean(false); 
                break;
            case "FR":
                states = KINEMATICS.toSwerveModuleStates(chassisSpeeds, new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2., -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.));
                defaultEntry.setBoolean(false);
                FLEntry.setBoolean(false);
                FREntry.setBoolean(true); 
                break;
            case "default":
                states = KINEMATICS.toSwerveModuleStates(chassisSpeeds);
                defaultEntry.setBoolean(true);
                FLEntry.setBoolean(false);
                FREntry.setBoolean(false); 
                break;
        }
    }
    
    private void handleLocked(){
        if (!wheelsLocked) {
            // If we are not in wheel's locked mode, set the states normally
            frontLeftModule.setState(states[0]);
            frontRightModule.setState(states[1]);
            backLeftModule.setState(states[2]);
            backRightModule.setState(states[3]);
        } else {
            // If we are in wheel's locked mode, set the drive velocity to 0 so there is no
            // movment, and command the steer angle to either plus or minus 45 degrees to
            // form an X pattern.
            frontLeftModule.lockModule(45);
            frontRightModule.lockModule(-45);
            backLeftModule.lockModule(-45);
            backRightModule.lockModule(45);
        }
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
    }

    private void updateTelemetry() {
        FLA.setDouble(frontLeftModule.getAbsolutePosition());
        FRA.setDouble(frontRightModule.getAbsolutePosition());
        BLA.setDouble(backLeftModule.getAbsolutePosition());
        BRA.setDouble(backRightModule.getAbsolutePosition());
        FLV.setDouble(frontLeftModule.getDriveVelocity());
        FRV.setDouble(frontRightModule.getDriveVelocity());
        BLV.setDouble(backLeftModule.getDriveVelocity());
        BRV.setDouble(backRightModule.getDriveVelocity());
        //Gyro.setDouble(getGyroscopeAngle()+getGyroOffset());
        SmartDashboard.putNumber("navx angle", navx.getAngle());
        SmartDashboard.putNumber("navx yaw", navx.getYaw());
        SmartDashboard.putNumber("displacement X", navx.getDisplacementX());
        SmartDashboard.putNumber("displacement Y", navx.getDisplacementY());
        SmartDashboard.putNumber("getGyroscopeAngle()", getGyroscopeAngle());

        // Periodically send a set of module states (I hope) I love the confidince!
        statePublisher.set(new SwerveModuleState[] {
            states[0],
            states[1],
            states[2],
            states[3]
        });
        posePublisher.set(poseEstimator.getEstimatedPosition());
    }
}
