package frc.robot.commands;

import static frc.robot.Constants.DISTANCE_MAP;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class SpeakerAimCommand extends Command {
    private static double tx;
    private static double speakerX;
    private static double speakerY;
    private static double heading;
    private static double armTarget;
    private static boolean finished;
    private static boolean running;
    private static boolean turnAround;

    private static Arm arm;
    private static PIDController steerPID;
    private static SimpleMotorFeedforward steerFF;
    private static Drivetrain drivetrain;

    public SpeakerAimCommand(Arm arm, Drivetrain drivetrain) {
        SpeakerAimCommand.arm = arm;
        //thePDH.setSwitchableChannel(false);
        SpeakerAimCommand.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        finished = false;
        steerPID = new PIDController(0.1, 0.0, 0.01);
        steerPID.enableContinuousInput(0.0, 360.0);
        steerFF = new SimpleMotorFeedforward(0.000001, 0.00001, 0.0000001);
        running = true;
    }

    @Override
    public void execute() {
        System.out.println(getTagDistance());
        // Get PID gains from shuffleboard and apply them.
        steerPID.setPID(0.1, 0.0, 0.01);

        // The alliance needs to exist for the code to work.
        if (DriverStation.getAlliance().isPresent()) {
            // Logic to determine the tag ID's we want
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                speakerX = 16.5608;
                speakerY = 5.5479;
                turnAround = false;
            } else { // It's Blue
                speakerX = -0.0381;
                speakerY = 5.5479;
                turnAround = true;
            }
    
            if (DriverStation.isAutonomous()) {
                // arm.autoAim();
                // Auto shoot
                if (rotationTargetMet() && armAngleTargetMet()) {
                    System.out.println("aim successful");
                    Intake.feedShooter();
                    finished = true;
                }
            }
        }
    }

    public static double rotationCalculate() {
        double robotX = drivetrain.getPose().getX();
        double robotY = drivetrain.getPose().getY();

        double deltaX = speakerX - robotX;
        double deltaY = speakerY - robotY;

        tx = Math.toDegrees(Math.atan(deltaY / deltaX));
        heading = drivetrain.getPose().getRotation().getDegrees();
        if (turnAround) { // Hi im blue
            return steerPID.calculate(heading - tx) - steerFF.calculate(heading - tx);
        } else {
            return steerPID.calculate(-(180-heading + tx)) - steerFF.calculate(-(180-heading + tx));
        }
    }

    public static double armAngleCalculate() { //basic calculation
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                speakerX = 16.5608;
                speakerY = 5.5479;
                turnAround = false;
            } else { // It's Blue
                speakerX = -0.0381;
                speakerY = 5.5479;
                turnAround = true;
            }
        if (getTagDistance() < 1.6) {
            System.out.println("layup mode");
            return 0.5;
        } else {
            armTarget = DISTANCE_MAP.get(getTagDistance());
            if (armTarget < 0.75 && armTarget > 0.5) {
                System.out.println(armTarget + ", " + getTagDistance());
                return armTarget;
            }
        }
        return(0.5);
    }

    // Straight line distance between camera sensor and tag across the XY plane in
    // centimeters
    public static double getTagDistance() {
        double robotX = drivetrain.getPose().getX();
        double robotY = drivetrain.getPose().getY();

        double deltaX = robotX - speakerX;
        double deltaY = robotY - speakerY ;
        
        return Math.hypot(Math.abs(deltaX), Math.abs(deltaY));
    }

    // Are we there yet?
    public static boolean armAngleTargetMet() {
        return (arm.getAngle() < armTarget + 0.02 && arm.getAngle() > armTarget - 0.02);
    }

    // Determine if we are considered within margin for a note to go into the
    // speaker
    public static boolean rotationTargetMet() {
        if ((tx < 3.0 && tx > -3.0)) {
            //thePDH.setSwitchableChannel(true);
            return true;
        } else {
            //thePDH.setSwitchableChannel(false);
            return false;
        }
    }
    
    public static boolean isRunning() {
        return running;
    }

    @Override
    public void end(boolean interrupted) {
        running = false;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}