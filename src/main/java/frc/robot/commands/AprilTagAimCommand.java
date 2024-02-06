package frc.robot.commands;

import static frc.robot.Constants.limelightName;
import static frc.robot.Constants.speedLimit;

import java.util.Map;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Drivetrain;

public class AprilTagAimCommand extends Command {
    private double tx;
    private String target;
    private double sightedTID;
    private double targetTID;
    private double targetTID2 = -1;
    private double steeringAdjust; 
    private DoubleSupplier stickX;
    private DoubleSupplier stickY;
    ;
    private Optional<Alliance> alliance = DriverStation.getAlliance();

    GenericEntry aimCommand = Shuffleboard.getTab("Competition")
   .add("shot threashold", false)
   .withWidget("Boolean Box")
   .withProperties(Map.of("colorWhenTrue", "green", "colorWhenFalse", "maroon"))
   .getEntry();
    
    private final Drivetrain drivetrain;
    public AprilTagAimCommand(Drivetrain drivetrain, String target, DoubleSupplier stickX, DoubleSupplier stickY) {
        this.drivetrain = drivetrain;
        this.target = target;
        this.stickX = stickX;
        this.stickY = stickY;
        addRequirements(drivetrain);
    }
    private final PIDController steerPID = new PIDController(0.13, 0.13, 0.01);

    @Override
    public void execute() {
        System.out.println("execute start, alliance="+alliance.get());
        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Red) {
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
            }
        } else {//its blue
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
        //determine if the primary tag id matches our target tag id
        sightedTID = LimelightHelpers.getFiducialID(limelightName);
        if (sightedTID == targetTID || sightedTID == targetTID2) {
            steerPID.enableContinuousInput(0.0, 360.0);
            tx = LimelightHelpers.getTX(limelightName);
            steeringAdjust = steerPID.calculate(tx);
            ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                stickX.getAsDouble()*speedLimit,
                stickY.getAsDouble()*speedLimit,
                steeringAdjust,
                Rotation2d.fromDegrees(drivetrain.getGyroscopeAngle() + drivetrain.getGyroOffset()));
            
            drivetrain.drive(ChassisSpeeds.discretize(fieldRelativeSpeeds, 0.020));
            SmartDashboard.putNumber("steringadjust",steeringAdjust);
            SmartDashboard.putNumber("tx",tx);
            if (tx < 3.0){ aimCommand.setBoolean(true); } else { aimCommand.setBoolean(false); }
        } else { // just normal drive with no rotation
            SmartDashboard.putString("steeringadjust", "no valid TID " + sightedTID + ", " + targetTID);
            drivetrain.drive(ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(
                stickX.getAsDouble(),
                stickY.getAsDouble(),
                0,
                Rotation2d.fromDegrees(drivetrain.getGyroscopeAngle() + drivetrain.getGyroOffset())), 0.020));
                aimCommand.setBoolean(false);
        }
    }
    @Override
    public void end(boolean interrupted) {
        System.out.println("interupted");
        drivetrain.drive(new ChassisSpeeds(stickX.getAsDouble(), stickY.getAsDouble(), 0));
    }
}