package frc.robot.commands;

import static frc.robot.Constants.LL_NAME;
import static frc.robot.Constants.LL_SPEED_LIMIT;

import java.util.Map;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.util.LimelightHelpers;

public class AprilTagAimRotateCommand extends Command {
    private static double tx;
    private static double ty;
    private static double sightedTID;
    private static double targetTID;
    private static double targetTID2 = -1;
    private static double steeringAdjust;
    private static double armTarget;
    private static DoubleSupplier stickX;
    private static DoubleSupplier stickY;
    private static boolean finished;
    private static boolean validTID;

    private static GenericEntry kP;
    private static GenericEntry kI;
    private static GenericEntry kD;
    private static GenericEntry shotTheashold;
    private static GenericEntry autoAimWorking;
    private static GenericEntry LLdistance;

    private final Drivetrain drivetrain;

    private final Interpolator<Double> doubleInterpolator = Interpolator.forDouble();

    private static Arm arm = new Arm();
    private static PowerDistribution thePDH;
    private static PIDController steerPID = new PIDController(kP.getDouble(0.13), kI.getDouble(0.0), kD.getDouble(0.01));

    // constructor for teleop
    public AprilTagAimRotateCommand(Drivetrain drivetrain, DoubleSupplier stickX, DoubleSupplier stickY, PowerDistribution thePDH) {
        this.drivetrain = drivetrain;
        this.stickX = stickX;
        this.stickY = stickY;
        this.thePDH = thePDH;

        thePDH.setSwitchableChannel(false);
        addRequirements(drivetrain);
        steerPID.enableContinuousInput(0.0, 360.0);
    }

    // constructor for autonomous
    public AprilTagAimRotateCommand(Drivetrain drivetrain, String target, PowerDistribution thePDH) {
        this.drivetrain = drivetrain;
        stickX = () -> 0;
        stickY = () -> 0;
        this.thePDH = thePDH;
        thePDH.setSwitchableChannel(false);
    }

    @Override
    public void initialize() {
        finished = false;
    }

    @Override
    public void execute() {
        // Get PID gains from shuffleboard and apply them.
        steerPID.setPID(kP.getDouble(0.13), kI.getDouble(0.0), kD.getDouble(0.01));
        // Grab the ID of the primary tag from LimeLight.
        sightedTID = LimelightHelpers.getFiducialID(LL_NAME);
        
        // The alliance needs to exist for the code to work.
        if (DriverStation.getAlliance().isPresent()) {
            // Logic to determine the tag ID's we want
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                targetTID = 3.0;
                targetTID2 = 4.0;
            } else { // It's Blue
                targetTID = 7.0;
                targetTID2 = 8.0;
            }
        }
        // Determine if the primary tag id matches our target tag ID.
        if (sightedTID == targetTID || sightedTID == targetTID2) {
            // used to let outside functions and operator know we got the tags
            validTID = true;
            autoAimWorking.setBoolean(true);

            //PID and interpolation calculations that are complicated
            rotationCalculate();
            armAngleCalculate();
            
            // Drive using stick input for teleop
            ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    stickX.getAsDouble() * LL_SPEED_LIMIT,
                    stickY.getAsDouble() * LL_SPEED_LIMIT,
                    steeringAdjust,
                    Rotation2d.fromDegrees(-(drivetrain.getGyroscopeAngle() + drivetrain.getGyroOffset())));
            drivetrain.drive(ChassisSpeeds.discretize(fieldRelativeSpeeds, 0.020));

            // Auto shoot
            if (DriverStation.isAutonomous() && rotationTargetMet() && armAngleTargetMet()) {
                System.out.println("aim successful");
                Intake.autoShoot();
                finished = true;
            }

        } else {
            // Let the gang know that the thing is blind
            autoAimWorking.setBoolean(false);
            validTID = false;

            // Just normal drive with no rotation
            drivetrain.drive(ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(
                    stickX.getAsDouble(),
                    stickY.getAsDouble(),
                    0,
                    Rotation2d.fromDegrees(drivetrain.getGyroscopeAngle() + drivetrain.getGyroOffset())),
                    0.020)
            );
        }
    }

    // Function to let the other subsystems know weather to use outputs or not.
    public final boolean getTargetGood() {
        return validTID;
    }

    public void rotationCalculate() {
        tx = LimelightHelpers.getTX(LL_NAME);
        ty = LimelightHelpers.getTY(LL_NAME);
        steeringAdjust = steerPID.calculate(tx);
    }

    public void armAngleCalculate() {
        armTarget = doubleInterpolator.interpolate(
                0.5,
                0.568783,
                MathUtil.inverseInterpolate(150, 516, getTagDistance()));
    }

    // Straight line distance between camera sensor and tag across the XY plane in centimeters
    public double getTagDistance() {
        return (Constants.SPEAKER_HEIGHT - Constants.LL_HEIGHT) / Math.tan((Constants.LL_ANGLE + ty) * Math.PI / 180);
    }
    // Are we there yet?
    public static boolean armAngleTargetMet() {
        return (arm.getAngle() < armTarget + 0.02 && arm.getAngle() > armTarget - 0.02);
    }
    // Determine if we are considered within margin for a note to go into the speaker
    public static boolean rotationTargetMet() {
            if ((tx < 3.0 && tx > -3.0)) { 
                shotTheashold.setBoolean(true);
                thePDH.setSwitchableChannel(true);
                return true;
            } else {
                shotTheashold.setBoolean(false);
                thePDH.setSwitchableChannel(false);
                return false;
            }
    }

    public final static void makeShuffleboard() {
        ShuffleboardLayout pidvals = Shuffleboard.getTab("Diag")
                .getLayout("LL Aim", BuiltInLayouts.kList)
                .withSize(2, 2);
        kP = pidvals.add("kP", 0.13)
                .getEntry();
        kI = pidvals.add("kI", 0.0)
                .getEntry();
        kD = pidvals.add("kD", 0.0)
                .getEntry();
        shotTheashold = Shuffleboard.getTab("Competition")
                .add("shot threashold", false)
                .withWidget("Boolean Box")
                .withPosition(10, 0)
                .withProperties(Map.of("colorWhenTrue", "lime", "colorWhenFalse", "gray"))
                .getEntry();
        autoAimWorking = Shuffleboard.getTab("Competition")
                .add("auto aiming", false)
                .withWidget("Boolean Box")
                .withPosition(10, 2)
                .withProperties(Map.of("colorWhenTrue", "lime", "colorWhenFalse", "gray"))
                .getEntry();
        LLdistance = Shuffleboard.getTab("Diag").add("distance", 0.0).getEntry();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(new ChassisSpeeds(stickX.getAsDouble(), stickY.getAsDouble(), 0));
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}