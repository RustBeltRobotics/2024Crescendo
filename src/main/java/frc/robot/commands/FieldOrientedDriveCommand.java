package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

import java.util.Map;
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

    GenericEntry feildOrientEntry = Shuffleboard.getTab("Competition")
            .add("Field Oriented", false)
            .withWidget("Boolean Box")
            .withPosition(0, 0)
            .withProperties(Map.of("colorWhenTrue", "lime", "colorWhenFalse", "gray"))
            .getEntry();

    public FieldOrientedDriveCommand(Drivetrain drivetrain,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier) {
        this.drivetrain = drivetrain;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;

        // Command requires the drivetrain subsystem
        addRequirements(drivetrain);
    }

    /**
     * This method is run every 20 ms.
     * <p>
     * Send the input x, y, and rotation velocities to the drivetrain's drive method
     * as a ChassisSpeed object.
     */
    @Override
    public void execute() {
            feildOrientEntry.setBoolean(true); 
            drivetrain.drive(ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(
                    translationXSupplier.getAsDouble(),
                    translationYSupplier.getAsDouble(),
                    rotationSupplier.getAsDouble(),
                    // TODO: for the next line, I'm wondering if it makes more sense to handle the
                    // negative sign inside of the Drivetrain class, instead of here. Seems like it
                    // could be a pain to have to remember we need to throw a negative sign out
                    // front everytime we call getGyro
                    // RE: pretty sure that will return a different number because that negative is distributed 
                    Rotation2d.fromDegrees(-(drivetrain.getGyroscopeAngle() + drivetrain.getGyroOffset()))), 0.020));
    }

    /** When the drive method is interupted, set all velocities to zero. */
    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(new ChassisSpeeds(0, 0, 0));
        feildOrientEntry.setBoolean(false);
    }
}
