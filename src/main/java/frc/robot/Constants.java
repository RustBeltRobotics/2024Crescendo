package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    /** Maximum battery voltage */
    public static final double MAX_VOLTAGE = 12.;

    // Spark Max current limits
    /** Smart drive motor current */
    public static final int DRIVE_SMART_CURRENT_LIMIT = 30;
    /** Smart steer motor current */
    public static final int STEER_SMART_CURRENT_LIMIT = 40;
    /** Secondary drive motor current */
    public static final int DRIVE_SECONDARY_CURRENT_LIMIT = 80;
    /** Secondary steer motor current */
    public static final int STEER_SECONDARY_CURRENT_LIMIT = 80;
    /** Smart current limit applied to NEOs */
    public static final int NEO_SMART_CURRENT_LIMIT = 60;
    /** Secondary current limit applied to NEOs */
    public static final int NEO_SECONDARY_CURRENT_LIMIT = 80;

    // Drivetrain Constants
    /**
     * The left-to-right distance between the drivetrain wheels
     * <p>
     * Should be measured from center to center
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.4445;

    /**
     * The front-to-back distance between the drivetrain wheels
     * <p>
     * Should be measured from center to center
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.4445;

    public static final double DRIVETRAIN_BASE_RADIUS = Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS, DRIVETRAIN_WHEELBASE_METERS) / 2.;

    /**
     * Creates a swerve kinematics object, to convert desired chassis velocity into
     * individual module states
     */
    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2., DRIVETRAIN_WHEELBASE_METERS / 2.),
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2., -DRIVETRAIN_WHEELBASE_METERS / 2.),
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2., DRIVETRAIN_WHEELBASE_METERS / 2.),
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2., -DRIVETRAIN_WHEELBASE_METERS / 2.));

    /** Conversion between motor rotations and drive meters */
    public static final double DRIVE_POSITION_CONVERSION = Math.PI
            * 0.1016 // 4" wheel diameter
            * (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0);
            
    /** Conversion between motor rotations per minute and drive meters per seconds */
    public static final double DRIVE_VELOCITY_CONVERSION = DRIVE_POSITION_CONVERSION / 60.;

    /**
     * The maximum linear velocity of the robot in meters per second. This is a
     * measure of how fast the robot can move linearly.
     */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 5676. * DRIVE_VELOCITY_CONVERSION;

    /**
     * The maximum angular velocity of the robot in radians per second. This is a
     * measure of how fast the robot can rotate in place.
     */
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND
            / Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2., DRIVETRAIN_WHEELBASE_METERS / 2.);

    /** Conversion between motor rotations and steer degrees */
    public static final double STEER_POSITION_CONVERSION = 360. * (1. / 12.8); // 12.8:1 gear ratio
    /** Conversion between motor rotations per minute and steer degrees per seconds */
    public static final double STEER_VELOCITY_CONVERSION = STEER_POSITION_CONVERSION / 60.;

    // Steer PID Constants
    public static final double STEER_P = 0.01;
    public static final double STEER_I = 0.0;
    public static final double STEER_D = 0.0;

    /** Max velocity while following a trajectory. Meters per second */
    public static final double MAX_TRAJECTORY_VELOCITY = 3.;

    /**
     * Max acceleration while following a trajectory. Meters per second per second
     */
    public static final double MAX_TRAJECTORY_ACCELERATION = 2.;

    // PID Constants for translation and rotation moves
    public static final double translation_P = 10.0;
    public static final double translation_I = 0.0;
    public static final double translation_D = 0.0;

    public static final double rotation_P = 5.0;
    public static final double rotation_I = 0.0;
    public static final double rotation_D = 0.0;

    // CAN IDs
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 21;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 20;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 1;

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 11;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 10;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 2;

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 16;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 17;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 4;

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 14;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 15;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 3;

    public static final int RIGHT_CLIMB = 12;
    public static final int ARM_INTAKE = 13;
    public static final int GROUND_INTAKE = 18;
    public static final int LEFT_CLIMB = 19;
    public static final int RIGHT_ROTATE = 22;
    public static final int LEFT_ROTATE = 23;
    public static final int RIGHT_SHOOTER = 24;
    public static final int LEFT_SHOOTER = 25;

    //Field
    public static final double SPEAKER_HEIGHT = 204.0; //cm

    //Limelight/vision
    public static final String LL_NAME = "limelight";
    public static final double LL_SPEED_LIMIT = 1.0;
    public static final double LL_HEIGHT = 10; //TODO: get the real number
    public static final double LL_ANGLE = 20; //TODO: get the real number

    //Shooter
    public static final double SPOOL_VELOCITY = 4257; //75% of max velocity

    //Arm
    public static final double AMP_POSITION = 0.75; //TODO: get this encoder reading
    public static final double SOURCE_POSITION = 0.75; //TODO: get this encoder reading
    public static final double GROUND_POSITION = 0.5; //TODO: get this encoder reading
    public static final double ARM_FF_kS = 0;
    public static final double ARM_FF_kG = 0;
    public static final double ARM_FF_kV = 0;
    public static final double MAX_ARM_VELOCITY = 2*Math.PI;
}
