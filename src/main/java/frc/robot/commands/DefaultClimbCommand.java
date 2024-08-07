package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import java.util.function.DoubleSupplier;

/**
 * This command is used to perform climbing with a joystick (speedSupplier is the joystick input value )
 */
public class DefaultClimbCommand extends Command {
    private final Climber climber;

    // DoubleSupplier objects need to be used, not double
    private final DoubleSupplier speedSupplier;

    public DefaultClimbCommand(Climber climber, DoubleSupplier speedSupplier) {
        this.climber = climber;
        this.speedSupplier = speedSupplier;

        // Command requires the intake subsystem
        addRequirements(climber);
    }

    /**
     * This method is run every 20 ms.
     * <p>
     * Send the input speeds to the appropriate intake/outtake method
     */
    @Override
    public void execute() {
        // dont always command zero so other commands can use the intake
        if (!DriverStation.isAutonomous()){
            climber.climb(speedSupplier.getAsDouble());
        }
    }

    /** When the intake method is interupted, set all velocities to zero. */
    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }
}
