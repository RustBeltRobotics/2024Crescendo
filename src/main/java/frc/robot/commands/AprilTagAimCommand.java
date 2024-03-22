package frc.robot.commands;

import static frc.robot.Constants.LL_NAME;

import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.util.LimelightHelpers;

public class AprilTagAimCommand extends Command {
    private static double tx;
    private static double ty;
    private static double sightedTID;
    private static double targetTID;
    private static double targetTID2;
    private static double armTarget;
    private static boolean finished;
    private static boolean validTID;

    private static GenericEntry kP;
    private static GenericEntry kI;
    private static GenericEntry kD;
    private static GenericEntry shotTheashold;
    private static GenericEntry autoAimWorking;
    private static GenericEntry LLdistance;

    private final static Interpolator<Double> doubleInterpolator = Interpolator.forDouble();

    private static Arm arm;
    private static PowerDistribution thePDH;
    private static PIDController steerPID;
    private static SimpleMotorFeedforward steerFF;

    public AprilTagAimCommand(PowerDistribution thePDH, Arm arm) {
        AprilTagAimCommand.arm = arm;
        AprilTagAimCommand.thePDH = thePDH;
        thePDH.setSwitchableChannel(false);
        //temp
    }

    @Override
    public void initialize() {
        finished = false;
        steerPID = new PIDController(0.7, 0.0, 0.01);
        steerPID.enableContinuousInput(0.0, 360.0);
        steerFF = new SimpleMotorFeedforward(0.000001, 0.00001, 0.0000001);
        LimelightHelpers.setPipelineIndex(LL_NAME, 0);
    }

    @Override
    public void execute() {
        // Get PID gains from shuffleboard and apply them.
        steerPID.setPID(kP.getDouble(0.7), kI.getDouble(0.0), kD.getDouble(0.01));
        // Grab the ID of the primary tag from LimeLight.
        sightedTID = LimelightHelpers.getFiducialID(LL_NAME);

        // The alliance needs to exist for the code to work.
        if (DriverStation.getAlliance().isPresent()) {
            // Logic to determine the tag ID's we want
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                targetTID = 3.0;
                targetTID2 = 4.0;
            } else { // It's Blue
                targetTID = 7.0;
                targetTID2 = 8.0;
            }
        }
        // Determine if the primary tag id matches our target tag ID.
        if (sightedTID == targetTID || sightedTID == targetTID2) {
            // used to let outside functions and operator know we got the tags
            validTID = true;
            autoAimWorking.setBoolean(true);
            LLdistance.setDouble(getTagDistance());

            if (DriverStation.isAutonomous()) {
                arm.autoAim();
                // Auto shoot
                if (rotationTargetMet() && armAngleTargetMet()) {
                    System.out.println("aim successful");
                    Intake.feedShooter();
                    finished = true;
                }
            }

        } else {
            // Let the gang know that the thing is blind
            autoAimWorking.setBoolean(false);
            validTID = false;
        }
    }

    // Function to let the other subsystems know weather to use outputs or not.
    public final static boolean getTargetGood() {
        if (LimelightHelpers.getFiducialID(LL_NAME) == targetTID || LimelightHelpers.getFiducialID(LL_NAME) == targetTID2) {
            return true;
        } else {
            return false;
        }
    }

    public static double rotationCalculate() {
        tx = LimelightHelpers.getTX(LL_NAME);
        ty = LimelightHelpers.getTY(LL_NAME);
        return steerPID.calculate(tx) - steerFF.calculate(tx);
    }

    public static double armAngleCalculate() {
        armTarget = doubleInterpolator.interpolate(
                0.5,
                0.568783,
                MathUtil.inverseInterpolate(150, 516, getTagDistance()));
        return armTarget;
    }

    // Straight line distance between camera sensor and tag across the XY plane in
    // centimeters
    public static double getTagDistance() {
        ty = LimelightHelpers.getTY(LL_NAME);
        return (Constants.SPEAKER_HEIGHT - Constants.LL_HEIGHT) / Math.tan((Constants.LL_ANGLE + ty) * Math.PI / 180);
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

    public final static void makeShuffleboard() {
        ShuffleboardLayout pidvals = Shuffleboard.getTab("Diag")
                .getLayout("LL Aim", BuiltInLayouts.kList)
                .withSize(2, 2);
        kP = pidvals.add("atkP", 0.05)
                .getEntry();
        kI = pidvals.add("atkI", 0.1)
                .getEntry();
        kD = pidvals.add("atkD", 0.01)
                .getEntry();
        kP = pidvals.add("atkV", 0.05)
                .getEntry();
        kI = pidvals.add("atkS", 0.1)
                .getEntry();
        kD = pidvals.add("atkA", 0.01)
                .getEntry();
        shotTheashold = Shuffleboard.getTab("Competition")
                .add("shot threashold", false)
                .withWidget("Boolean Box")
                .withPosition(10, 0)
                .withProperties(Map.of("colorWhenTrue", "lime", "colorWhenFalse", "gray"))
                .getEntry();
        autoAimWorking = Shuffleboard.getTab("Competition")
                .add("auto aiming", false)
                .withWidget("Boolean Box")
                .withPosition(10, 2)
                .withProperties(Map.of("colorWhenTrue", "lime", "colorWhenFalse", "gray"))
                .getEntry();
        LLdistance = Shuffleboard.getTab("Diag").add("distance", 0.0).getEntry();
    }
    @Override
    public void end(boolean interrupted) {
        validTID = false;
        LimelightHelpers.setPipelineIndex(LL_NAME, 0);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}