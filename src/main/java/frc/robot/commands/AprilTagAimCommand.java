package frc.robot.commands;

import static frc.robot.Constants.LL_NAME;
import static frc.robot.Constants.LL_SPEED_LIMIT;

import java.util.Map;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class AprilTagAimCommand extends Command {
    private double tx;
    private double ty;
    private String target;
    private double sightedTID;
    private double targetTID;
    private double targetTID2 = -1;
    private double steeringAdjust;
    private double armTarget;
    private DoubleSupplier stickX;
    private DoubleSupplier stickY;
    private boolean autonomous;
    private boolean finished;

    static GenericEntry kP;
    static GenericEntry kI;
    static GenericEntry kD;
    static GenericEntry aimCommand;
    static GenericEntry autoAimCommand;


    public static double AUTO_TX;
    public static boolean validTID = false;

    private final Drivetrain drivetrain;

    private final Interpolator<Double> doubleInterpolator = Interpolator.forDouble();

    Arm arm = new Arm();

    // constructor for teleop
    public AprilTagAimCommand(Drivetrain drivetrain, String target, DoubleSupplier stickX, DoubleSupplier stickY) {
        this.drivetrain = drivetrain;
        this.target = target;
        this.stickX = stickX;
        this.stickY = stickY;
        addRequirements(drivetrain);
    }

    // constructor for autonomous
    public AprilTagAimCommand(Drivetrain drivetrain, String target) {
        this.drivetrain = drivetrain;
        this.target = target;
        stickX = () -> 0;
        stickY = () -> 0;
        autonomous = true;
    }

    public final static void makeShuffleboard() {
        ShuffleboardLayout pidvals = Shuffleboard.getTab("Diag")
                .getLayout("LL Aim", BuiltInLayouts.kList)
                .withSize(2, 2);
        kP = pidvals.add("kP", 0.13)
                .getEntry();
        kI = pidvals.add("kI", 0.13)
                .getEntry();
        kD = pidvals.add("kD", 0.00)
                .getEntry();
        aimCommand = Shuffleboard.getTab("Competition")
                .add("shot threashold", false)
                .withWidget("Boolean Box")
                .withPosition(10, 0)
                .withProperties(Map.of("colorWhenTrue", "lime", "colorWhenFalse", "gray"))
                .getEntry();
        autoAimCommand = Shuffleboard.getTab("Competition")
                .add("auto aiming", false)
                .withWidget("Boolean Box")
                .withPosition(10, 2)
                .withProperties(Map.of("colorWhenTrue", "lime", "colorWhenFalse", "gray"))
                .getEntry();
    }

    @Override
    public void initialize() {
        finished = false;
    }

    @Override
    public void execute() {
        final PIDController steerPID = new PIDController(kP.getDouble(0.13), kI.getDouble(0.0), kD.getDouble(0.01));
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                switch (target) {
                    case "speaker":
                        targetTID = 3.0;
                        targetTID2 = 4.0;
                        break;
                    case "amp":
                        targetTID = 5.0;
                        break;
                    case "source":
                        targetTID = 9.0;
                        targetTID2 = 10.0;
                        break;
                }
            } else {// its blue
            switch (target) {
                case "speaker":
                    targetTID = 7.0;
                    targetTID2 = 8.0;
                    break;
                case "amp":
                    targetTID = 6.0;
                    break;
                case "source":
                    targetTID = 1.0;
                    targetTID2 = 2.0;
                    break;
            }
        }
        // determine if the primary tag id matches our target tag id
        sightedTID = LimelightHelpers.getFiducialID(LL_NAME);
        if (sightedTID == targetTID || sightedTID == targetTID2) {
            steerPID.enableContinuousInput(0.0, 360.0);
            tx = LimelightHelpers.getTX(LL_NAME);
            ty = LimelightHelpers.getTY(LL_NAME);
            steeringAdjust = steerPID.calculate(tx);
            if (autonomous) {
                AUTO_TX = steeringAdjust;
                validTID = true;
                autoAimCommand.setBoolean(true);
            } else {
                armTarget = doubleInterpolator.interpolate(0.5,0.75, MathUtil.inverseInterpolate(0,1,(Constants.SPEAKER_HEIGHT-Constants.LL_HEIGHT) / Math.tan(Constants.LL_ANGLE+ty))); //TODO: change this start end vals
                //arm.setAngle(armTarget);
                ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                        stickX.getAsDouble() * LL_SPEED_LIMIT,
                        stickY.getAsDouble() * LL_SPEED_LIMIT,
                        steeringAdjust,
                        Rotation2d.fromDegrees(drivetrain.getGyroscopeAngle() + drivetrain.getGyroOffset()));

                drivetrain.drive(ChassisSpeeds.discretize(fieldRelativeSpeeds, 0.020));
            }
            if ((tx < 3.0 || tx > -3.0)&& (arm.getAngle() < armTarget+1 || arm.getAngle() > armTarget-1)) {
                aimCommand.setBoolean(true);
            } else {
                aimCommand.setBoolean(false);
            }

            // auto shoot
            if (autonomous && aimCommand.getBoolean(false) == true) {
                Intake.feedShooter();
                finished = true;
            }

        } else { // just normal drive with no rotation
            if (autonomous) {
                validTID = false;
                autoAimCommand.setBoolean(false);
            } else {
            drivetrain.drive(ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(
                    stickX.getAsDouble(),
                    stickY.getAsDouble(),
                    0,
                    Rotation2d.fromDegrees(drivetrain.getGyroscopeAngle() + drivetrain.getGyroOffset())), 0.020));
            aimCommand.setBoolean(false);
            }
        }
    }
    }
    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(new ChassisSpeeds(stickX.getAsDouble(), stickY.getAsDouble(), 0));
    }
    @Override
    public boolean isFinished() {
        return finished;
    }
}