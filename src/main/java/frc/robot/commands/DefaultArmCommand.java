package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import java.util.function.DoubleSupplier;

/** This command is used to manually control the arm rotation and extension. */
public class DefaultArmCommand extends Command {
    private final Arm arm;

    // DoubleSupplier objects need to be used, not double
    private final DoubleSupplier rotationSupplier;

    public DefaultArmCommand(Arm arm, DoubleSupplier rotationSupplier) {
        this.arm = arm;
        this.rotationSupplier = rotationSupplier;

        // Command requires the arm subsystem
        addRequirements(arm);
    }

    /**
     * This method is run every 20 ms.
     * <p>
     * Send the input rotation and extension velocities to the arm's driveArm method.
     */
    @Override
    public void execute() {
        arm.rotate(rotationSupplier.getAsDouble());
    }

    /** When the drive method is interupted, set all velocities to zero. */
    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }
}
