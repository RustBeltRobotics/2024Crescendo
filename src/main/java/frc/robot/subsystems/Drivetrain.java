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
import static frc.robot.Constants.PDH;
import static frc.robot.Constants.LL_NAME;
import static frc.robot.Constants.rotation_D;
import static frc.robot.Constants.rotation_I;
import static frc.robot.Constants.rotation_P;
import static frc.robot.Constants.translation_D;
import static frc.robot.Constants.translation_I;
import static frc.robot.Constants.translation_P;

import java.sql.Driver;
import java.util.Map;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.SpeakerAimCommand;
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

    // The speed of the robot in x and y translational velocities and rotational
    // velocity
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    // Boolean statement to control locking the wheels in an X-position
    private boolean wheelsLocked = false;

    private double tagDist;
    private double visionWeight;

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

    // networktables publisher for advantagescope swerve visualization
    private final StructArrayPublisher<SwerveModuleState> statePublisher;
    // networktables publisher for advantagescope 2d pose visualization
    StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("/MyPose", Pose2d.struct).publish();
           
    PowerDistribution thePDH = new PowerDistribution(PDH, ModuleType.kRev);
        public Drivetrain() {
        // Start publishing an array of module states with the "/SwerveStates" key
        statePublisher = NetworkTableInstance.getDefault()
                .getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();

        // Configure AutoBuilder last
        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                                                 // Constants class
                        new PIDConstants(translation_P, translation_I, translation_D), // Translation PID constants
                        new PIDConstants(rotation_P, rotation_I, rotation_D), // Rotation PID constants
                        Constants.MAX_VELOCITY_METERS_PER_SECOND, // Max module speed, in m/s
                        Constants.DRIVETRAIN_BASE_RADIUS, // Drive base radius in meters. Distance from robot center to
                                                          // furthest module.
                        new ReplanningConfig()),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
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
        
        // Create the poseEstimator with vectors to weight our vision measurements
        poseEstimator = new SwerveDrivePoseEstimator(
            KINEMATICS, 
            getGyroscopeRotation(), getSwerveModulePositions(), new Pose2d(), 
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))
        );
        
        theMove = "default";
    }

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the
     * robot is currently facing to the 'forwards' direction.
     */
    public void zeroGyroscope() {
        gyroOffset = -getGyroscopeAngle(); // we were calling the getgyrosocpe function shich was also negated
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

    public static double getTagDistance() {
        return (Constants.SPEAKER_HEIGHT - Constants.LL_HEIGHT) / Math.tan((Constants.LL_ANGLE + LimelightHelpers.getTY(LL_NAME)) * Math.PI / 180);
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
        // Update odometry from swerve states
        poseEstimator.update(
            getGyroscopeRotation(),
            getSwerveModulePositions()
        );

        // Update odometry from vision if we can see two or more apriltags
        // LimelightHelpers.SetRobotOrientation(LL_NAME, poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(LL_NAME);
            tagDist = getTagDistance();
            System.out.println(tagDist);
            if (limelightMeasurement.tagCount >= 1) {
                if (tagDist < 450) {
                    thePDH.setSwitchableChannel(true);
                    System.out.println("true");
                    // Desmos format equation for comp tuning: \frac{1}{3.6}\left(x-600\right)^{\frac{1}{5}}+1
                    /** How to craft the java function:
                     *  change the 600 to the distance the LL pose starts jumping
                     *  adjust the denominator of the amplitude so that the line just about hits zero making sure it doesnt go below that
                     *  replace the 1 on the end with the value of the numerator from the amplitude
                     */
                    visionWeight = (1/3.6) * (Math.pow(Math.abs(tagDist-600), 1.0 / 5.0) * -1 + 3.6);
                    // From LL example:  poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
                    // (higher number -> trust vision less)
                    // 9999999 is our gyro weight because we trust that a whole bunch
                    poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(visionWeight, visionWeight, 9999999));
                    poseEstimator.addVisionMeasurement(
                        limelightMeasurement.pose,
                        limelightMeasurement.timestampSeconds
                    );
                }
        } else {
            System.out.println("false");
            thePDH.setSwitchableChannel(false);
        }
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
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
     * Decide where the center of rotation is going to be based on function call (we
     * got the moves)
     **/
    public void setMoves(String theMove) {
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

    // drives the robot, using limelight aim data if applicable
    public void autoDrive(ChassisSpeeds chassisSpeeds) {
        if (SpeakerAimCommand.isRunning()) {
            this.chassisSpeeds = new ChassisSpeeds(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond,
                    SpeakerAimCommand.rotationCalculate());
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
        handleMoves();

        handleLocked();

        updateOdometry();

        updateTelemetry();
    }

    private void handleMoves() {
        switch (theMove) {
            case "FL":
                states = KINEMATICS.toSwerveModuleStates(chassisSpeeds, new Translation2d(
                        Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2., Constants.DRIVETRAIN_WHEELBASE_METERS / 2.));
                defaultEntry.setBoolean(false);
                FLEntry.setBoolean(true);
                FREntry.setBoolean(false);
                break;
            case "FR":
                states = KINEMATICS.toSwerveModuleStates(chassisSpeeds, new Translation2d(
                        Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2., -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.));
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

    public void forceVisionPose() {
        Pose2d visionPose2d;
        visionPose2d = LimelightHelpers.getBotPose2d_wpiBlue(LL_NAME);
        if (LimelightHelpers.getBotPoseEstimate_wpiBlue(LL_NAME).tagCount >= 1) {
            poseEstimator.resetPosition(getGyroscopeRotation(), getSwerveModulePositions(), visionPose2d);
            System.out.println("force vision heading successful");
        } else if (DriverStation.getAlliance().get() == Alliance.Red) { // Take a guess as to where we are based on alliance and DS position, hope vision kicks in
            System.err.println("NO TAG FOUND, DEFUALTING TO SUBWOOFER POSITIONS");
            switch (DriverStation.getLocation().getAsInt()) {
                case 1:
                    poseEstimator.resetPosition(getGyroscopeRotation(), getSwerveModulePositions(), new Pose2d(0.78, 6.63, new Rotation2d(Math.toRadians(60))));
                    break;
                case 2:
                    poseEstimator.resetPosition(getGyroscopeRotation(), getSwerveModulePositions(), new Pose2d(1.34, 5.55, new Rotation2d(Math.toRadians(0))));
                    break;
                case 3:
                    poseEstimator.resetPosition(getGyroscopeRotation(), getSwerveModulePositions(), new Pose2d(0.82, 4.51, new Rotation2d(Math.toRadians(-60))));
                    break;
            }
        } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
            System.err.println("NO TAG FOUND, DEFUALTING TO SUBWOOFER POSITIONS");
            switch (DriverStation.getLocation().getAsInt()) {
                case 1:
                    poseEstimator.resetPosition(getGyroscopeRotation(), getSwerveModulePositions(), new Pose2d(15.74, 6.59, new Rotation2d(Math.toRadians(120))));
                    break;
                case 2:
                    poseEstimator.resetPosition(getGyroscopeRotation(), getSwerveModulePositions(), new Pose2d(15.24, 5.55, new Rotation2d(Math.toRadians(180))));
                    break;
                case 3:
                    poseEstimator.resetPosition(getGyroscopeRotation(), getSwerveModulePositions(), new Pose2d(15.74, 4.52, new Rotation2d(Math.toRadians(-120))));
                    break;
            }
        } else {
            System.err.println("NO TAGS OR DS POSITION FOUND, POSE NOT SET");
        }
    }

    private void handleLocked() {
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

        posePublisher.set(poseEstimator.getEstimatedPosition());


        // FLA.setDouble(frontLeftModule.getAbsolutePosition());
        // FRA.setDouble(frontRightModule.getAbsolutePosition());
        // BLA.setDouble(backLeftModule.getAbsolutePosition());
        // BRA.setDouble(backRightModule.getAbsolutePosition());
        // FLA.setDouble(states[0].angle.getDegrees());
        // FRA.setDouble(states[1].angle.getDegrees());
        // BLA.setDouble(states[2].angle.getDegrees());
        // BRA.setDouble(states[3].angle.getDegrees());
        // FLV.setDouble(frontLeftModule.getSteerPosition());
        // FRV.setDouble(frontRightModule.getSteerPosition());
        // BLV.setDouble(backLeftModule.getSteerPosition());
        // BRV.setDouble(backRightModule.getSteerPosition());

        FLA.setDouble(states[0].speedMetersPerSecond);
        FRA.setDouble(states[1].speedMetersPerSecond);
        BLA.setDouble(states[2].speedMetersPerSecond);
        BRA.setDouble(states[3].speedMetersPerSecond);
        FLV.setDouble(frontLeftModule.getDriveVelocity());
        FRV.setDouble(frontRightModule.getDriveVelocity());
        BLV.setDouble(backLeftModule.getDriveVelocity());
        BRV.setDouble(backRightModule.getDriveVelocity());
        Gyro.setDouble(getGyroscopeAngle() + getGyroOffset());

        // SmartDashboard.putNumber("navx angle", navx.getAngle());
        // SmartDashboard.putNumber("navx yaw", navx.getYaw());
        // SmartDashboard.putNumber("displacement X", navx.getDisplacementX());
        // SmartDashboard.putNumber("displacement Y", navx.getDisplacementY());
        // SmartDashboard.putNumber("getGyroscopeAngle()", getGyroscopeAngle());

        // Periodically send a set of module states (I hope) I love the confidince!
        statePublisher.set(new SwerveModuleState[] {
        states[0],
        states[1],
        states[2],
        states[3]
        });
    }
}
