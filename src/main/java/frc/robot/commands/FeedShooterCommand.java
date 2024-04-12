package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class FeedShooterCommand extends Command {
    boolean finished = false;
    @Override
    public void execute() {
        Intake.feedShooter();
        if (!Intake.getSwitch()) {
            Intake.stopBothIntakes();
            finished = true;
            System.out.println("finished");
        }
    }
    @Override
    public boolean isFinished() {
        return finished;
    }
}
