package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;

public class LockNoteCommand extends Command {
    boolean finished = false;
    Intake intake;
    public LockNoteCommand(Intake intake){
        this.intake = intake;
    }
    @Override
    public void initialize() {
        addRequirements(intake);
        finished = false;
        if (Intake.getSwitch()) {
            new SequentialCommandGroup(
                new ParallelRaceGroup(new WaitCommand(.4), new RepeatCommand(new InstantCommand(() -> intake.runArmIntake(.35)))), 
                new ParallelRaceGroup(new RepeatCommand(new InstantCommand(() -> intake.runArmIntake(-1))), new WaitCommand(0.25)), 
                new InstantCommand(() -> intake.stopArmIntake())).finallyDo(() -> finished = true).schedule();
        }
    }
    @Override
    public boolean isFinished() {
        return finished;
    }
}
