package frc.robot.commands;

import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class GroundPickUpCommand extends Command {
    private Arm arm = new Arm();
    private Intake intake = new Intake();
    private EventLoop switchEventLoop = new EventLoop();
    BooleanEvent loaded = new BooleanEvent(switchEventLoop, Intake::getSwitch);
    boolean finished;

    @Override
    public void initialize() {
        finished = false;
    }
    
    @Override
    public void execute() {
        arm.groundPose();
        intake.runBothIntakes(1);
        switchEventLoop.poll();
        loaded.ifHigh(() -> Intake.stopArmIntake());
        loaded.ifHigh(() -> finished = true);
        System.out.println("groundpicking");
    }
    @Override
    public boolean isFinished() {
        return finished;
    }
}
