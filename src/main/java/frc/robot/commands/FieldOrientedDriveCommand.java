package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.Constants.LL_SPEED_LIMIT;
import static frc.robot.Constants.MAX_VELOCITY_METERS_PER_SECOND;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * This command is used to drive the robot with a coordinate system that is
 * relative to the field, not the robot
 */
public class FieldOrientedDriveCommand extends Command {
    private final Drivetrain drivetrain;

    // DoubleSupplier objects need to be used, not double
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;
    private final BooleanSupplier BbuttonSupplier;

    GenericEntry feildOrientEntry = Shuffleboard.getTab("Competition")
            .add("Field Oriented", false)
            .withWidget("Boolean Box")
            .withPosition(0, 0)
            .withProperties(Map.of("colorWhenTrue", "lime", "colorWhenFalse", "gray"))
            .getEntry();

    public FieldOrientedDriveCommand(Drivetrain drivetrain,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier, 
            BooleanSupplier BbuttonSupplier) {
        this.drivetrain = drivetrain;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;
        this.BbuttonSupplier = BbuttonSupplier;

        // Command requires the drivetrain subsystem
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        feildOrientEntry.setBoolean(true);
    }

    /**
     * This method is run every 20 ms.
     * <p>
     * Send the input x, y, and rotation velocities to the drivetrain's drive method
     * as a ChassisSpeed object.
     */
    @Override
    public void execute() {
        if (AprilTagAimCommand.getTargetGood() && BbuttonSupplier.getAsBoolean()) {
            drivetrain.drive(ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(
                    translationXSupplier.getAsDouble() * LL_SPEED_LIMIT,
                    translationYSupplier.getAsDouble() * LL_SPEED_LIMIT,
                    AprilTagAimCommand.rotationCalculate(),
                    Rotation2d.fromDegrees((drivetrain.getGyroscopeAngle() + drivetrain.getGyroOffset()))), 0.020));
        } else {
            drivetrain.drive(ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(
                    translationXSupplier.getAsDouble(),
                    translationYSupplier.getAsDouble(),
                    rotationSupplier.getAsDouble(),
                    Rotation2d.fromDegrees((drivetrain.getGyroscopeAngle() + drivetrain.getGyroOffset()))), 0.020));
        }
    }

    /** When the drive method is interupted, set all velocities to zero. */
    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(new ChassisSpeeds(0, 0, 0));
        feildOrientEntry.setBoolean(false);
    }
}
