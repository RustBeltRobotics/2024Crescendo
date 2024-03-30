package frc.robot.commands;

import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class GroundPickUpCommand extends Command {
    private Arm arm = new Arm();
    private Intake intake = new Intake();
    private EventLoop switchEventLoop = new EventLoop();
    BooleanEvent loaded = new BooleanEvent(switchEventLoop, Intake::getSwitch);
    boolean finished;
    boolean lockIn;

    Command dillanCommand = new SequentialCommandGroup(
            new ParallelRaceGroup(new WaitCommand(.4), new RepeatCommand(new InstantCommand(() -> intake.runArmIntake(.35)))), 
            new ParallelRaceGroup(new RepeatCommand(new InstantCommand(() -> Intake.runArmIntake(-1))), new WaitCommand(0.25)), 
            new InstantCommand(() -> Intake.stopArmIntake()));

    public GroundPickUpCommand(boolean lockIn) {
        this.lockIn = lockIn;
    }

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
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted && lockIn) {
            dillanCommand.schedule();
        } else { 
            Intake.stopBothIntakes();
        }
    }
    @Override
    public boolean isFinished() {
        return finished;
    }
}
