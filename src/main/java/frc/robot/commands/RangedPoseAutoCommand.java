package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

/**
 * Command for use in Autonomous mode to aim the arm at the speaker for a ranged shot
 */
public class RangedPoseAutoCommand extends Command {
    boolean finished = false;
    Arm arm;

    public RangedPoseAutoCommand(Arm arm) {
        this.arm = arm;
    }

    @Override
    public void initialize() {
        finished = false;
        addRequirements(arm);
    }

    @Override
    public void execute(){
        arm.autoAim();
    }

    @Override
    public void end(boolean interrupted){
        finished = true;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
