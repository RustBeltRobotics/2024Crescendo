package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class FeedShooterCommand extends Command {
    boolean finished = false;

    private Intake intake;
    public FeedShooterCommand(Intake intake) {
        this.intake = intake;
    }

    @Override
    public void execute() {
        intake.feedShooter();
        if (!intake.getSwitch()) {
            intake.stopBothIntakes();
            finished = true;
            System.out.println("finished");
        }
    }
    @Override
    public boolean isFinished() {
        return finished;
    }
}
