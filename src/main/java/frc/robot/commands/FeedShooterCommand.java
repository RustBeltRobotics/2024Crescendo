package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * This command is used to feed the shooter with a note using the intake and beam breaker switch
 */
public class FeedShooterCommand extends Command {
    private boolean finished = false;
    private Intake intake;
    private Shooter shooter;

    public FeedShooterCommand(Intake intake, Shooter shooter) {
        this.intake = intake;
        this.shooter = shooter;
    }

    @Override
    public void execute() {
        intake.feedShooter();
        if (!intake.getSwitch()) {
            intake.stopBothIntakes();
            if(!DriverStation.isAutonomous()){
                shooter.stop();
            }
            finished = true;
        }
    }
    @Override
    public boolean isFinished() {
        return finished;
    }
}
