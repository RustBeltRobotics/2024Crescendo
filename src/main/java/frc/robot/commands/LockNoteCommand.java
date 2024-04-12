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
    @Override
    public void initialize() {
        finished = false;
        if (Intake.getSwitch()) {
            new SequentialCommandGroup(
                new ParallelRaceGroup(new WaitCommand(.4), new RepeatCommand(new InstantCommand(() -> Intake.runArmIntake(.35)))), 
                new ParallelRaceGroup(new RepeatCommand(new InstantCommand(() -> Intake.runArmIntake(-1))), new WaitCommand(0.25)), 
                new InstantCommand(() -> Intake.stopArmIntake())).schedule();
        }
        finished = true;
    }
    @Override
    public boolean isFinished() {
        return finished;
    }
}
