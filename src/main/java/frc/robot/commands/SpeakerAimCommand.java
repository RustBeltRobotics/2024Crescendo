package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private static GenericEntry kP;
    private static GenericEntry kI;
    private static GenericEntry kD;
    private static GenericEntry shotTheashold;
    private static GenericEntry autoAimWorking;

    private static Arm arm;
    private static PowerDistribution thePDH;
    private static PIDController steerPID;
    private static SimpleMotorFeedforward steerFF;
    private static Drivetrain drivetrain;

    public SpeakerAimCommand(PowerDistribution thePDH, Arm arm, Drivetrain drivetrain) {
        SpeakerAimCommand.arm = arm;
        SpeakerAimCommand.thePDH = thePDH;
        thePDH.setSwitchableChannel(false);
        SpeakerAimCommand.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        finished = false;
        steerPID = new PIDController(0.7, 0.0, 0.01);
        steerPID.enableContinuousInput(0.0, 360.0);
        steerFF = new SimpleMotorFeedforward(0.000001, 0.00001, 0.0000001);
        running = true;
    }

    @Override
    public void execute() {
        // Get PID gains from shuffleboard and apply them.
        steerPID.setPID(kP.getDouble(0.7), kI.getDouble(0.0), kD.getDouble(0.01));

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
            autoAimWorking.setBoolean(true);
    
            if (DriverStation.isAutonomous()) {
                arm.autoAim();
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
        SmartDashboard.putNumber("calc: ", -heading + tx);
        if (turnAround) { // Hi im blue
            return steerPID.calculate(-(180-heading + tx));
        } else {
            return steerPID.calculate(-(180-heading + tx));
        }
        //- steerFF.calculate(heading + tx)  //TODO: REMEMBER TO ADD THIS BACK IT MAKES THE THING WORK GOOD
    }

    public static double armAngleCalculate() { //basic calculation
        if (getTagDistance() < 1.6) {
            return 0.5;
        } else {
            armTarget = (1.67*Math.pow(10, -3)*Math.pow(getTagDistance(), 2)) - (2.531*Math.pow(10, -2)*Math.pow(getTagDistance(), 2)) + (1.321*Math.pow(10,-1)*getTagDistance()) + 0.3467;
            if (armTarget < 0.75 && armTarget > 0.5) { 
                return armTarget; 
            }
        }
        return 0.5;
    }

    // Straight line distance between camera sensor and tag across the XY plane in
    // centimeters
    public static double getTagDistance() {
        double robotX = drivetrain.getPose().getX();
        double robotY = drivetrain.getPose().getY();

        double deltaX = robotX - speakerX;
        double deltaY = robotY - speakerY ;

        SmartDashboard.putNumber("deltaY", deltaY);
        SmartDashboard.putNumber("deltaX", deltaX);
        SmartDashboard.putNumber("robotX", robotX);
        SmartDashboard.putNumber("robotY", robotY);
        
        SmartDashboard.putNumber("dist, ", Math.hypot(deltaX, deltaY));
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
            //shotTheashold.setBoolean(true);
            thePDH.setSwitchableChannel(true);
            return true;
        } else {
            //shotTheashold.setBoolean(false);
            thePDH.setSwitchableChannel(false);
            return false;
        }
    }
    
    public static boolean isRunning() {
        return running;
    }

    @Override
    public void end(boolean interrupted) {
        running = true;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    public final static void makeShuffleboard() {
        ShuffleboardLayout pidvals = Shuffleboard.getTab("Diag")
                .getLayout("Pose Aim", BuiltInLayouts.kList)
                .withSize(2, 2);
        kP = pidvals.add("kP", 0.05)
                .getEntry();
        kI = pidvals.add("kI", 0.1)
                .getEntry();
        kD = pidvals.add("kD", 0.01)
                .getEntry();
        kP = pidvals.add("kV", 0.05)
                .getEntry();
        kI = pidvals.add("kS", 0.1)
                .getEntry();
        kD = pidvals.add("kA", 0.01)
                .getEntry();
        shotTheashold = Shuffleboard.getTab("Competition")
                .add("pose shot threashold", false)
                .withWidget("Boolean Box")
                .withPosition(10, 0)
                .withProperties(Map.of("colorWhenTrue", "lime", "colorWhenFalse", "gray"))
                .getEntry();
        autoAimWorking = Shuffleboard.getTab("Competition")
                .add("pose auto aiming", false)
                .withWidget("Boolean Box")
                .withPosition(10, 2)
                .withProperties(Map.of("colorWhenTrue", "lime", "colorWhenFalse", "gray"))
                .getEntry();
    }
}