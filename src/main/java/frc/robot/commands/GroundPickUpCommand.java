package frc.robot.commands;

import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class GroundPickUpCommand extends Command {
    private Arm arm = new Arm();
    private Intake intake = new Intake();
    private EventLoop switchEventLoop = new EventLoop();
    private BooleanEvent loaded = new BooleanEvent(switchEventLoop, Intake::getSwitch);
    private boolean finished;

    @Override
    public void initialize() {
        finished = false;
        addRequirements(intake, arm);
    }
    
    @Override
    public void execute() {
        arm.groundPose();
        intake.runBothIntakes(1);
        switchEventLoop.poll();
        loaded.ifHigh(() -> finished = true);
        loaded.ifHigh(() -> new InstantCommand(() -> Intake.stopBothIntakes()));
    }

    @Override
    public void end(boolean interrupted) {
        Intake.stopBothIntakes();
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
