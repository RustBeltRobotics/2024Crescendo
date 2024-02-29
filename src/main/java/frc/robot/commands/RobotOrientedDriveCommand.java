package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

/**
 * This command is used to drive the robot with a coordinate system that is
 * relative to the robot, not the field.
 */
public class RobotOrientedDriveCommand extends Command {
    private final Drivetrain drivetrain;

    // DoubleSupplier objects need to be used, not double
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;
    private final IntSupplier povSupplier;

    GenericEntry robotOrientEntry = Shuffleboard.getTab("Competition")
            .add("Robot Oriented", false)
            .withWidget("Boolean Box")
            .withPosition(0, 1)
            .withProperties(Map.of("colorWhenTrue", "lime", "colorWhenFalse", "gray"))
            .getEntry();

    public RobotOrientedDriveCommand(Drivetrain drivetrain,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier,
            IntSupplier povSupplier) {
        this.drivetrain = drivetrain;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;
        this.povSupplier = povSupplier;

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
        robotOrientEntry.setBoolean(true);
        // FIXME: Unless you're intentionally leaving it in here for some reason, I
        // would recommend removing all of the POV stuff from this commmand. For
        // background, this was obselete code from Arno. Our first attempt at giving the
        // driver precision driving capability was to allow them to use the D-Pad to
        // drive at a heavily reduced max velocity. We ended up switching to the Bumper
        // button speed limiter controls instead, but because we never used robot
        // oriented drive, we forgot to go back and remove it from this command. Since
        // you're only ever calling this command with a -1 for the povSupplier input, I
        // don't think leaving it in should break anything, but removing it will reduce
        // confusion and be slightly safer. If someone were to ever link a call to this
        // command with the actual D-Pad input for some reason, it would cause some
        // wonky behavior, since you're already binding various D-Pad buttons to other
        // commands.
        int pov = povSupplier.getAsInt();
        if (pov == -1) {
            drivetrain.drive(new ChassisSpeeds(
                    translationXSupplier.getAsDouble(),
                    translationYSupplier.getAsDouble(),
                    rotationSupplier.getAsDouble()));
        } else {
            drivetrain.drive(new ChassisSpeeds(
                    Math.cos(Math.toRadians(pov)),
                    Math.sin(Math.toRadians(pov - 180)),
                    0.));
        }
    }

    /** When the drive method is interupted, set all velocities to zero. */
    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(new ChassisSpeeds(0, 0, 0));
        robotOrientEntry.setBoolean(false);
    }
}
