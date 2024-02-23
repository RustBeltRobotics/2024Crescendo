package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.AprilTagAimCommand;
import frc.robot.commands.DefaultArmCommand;
import frc.robot.commands.DefaultIntakeCommand;
import frc.robot.commands.FieldOrientedDriveCommand;
import frc.robot.commands.GroundPickUpCommand;
import frc.robot.commands.RobotOrientedDriveCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import static frc.robot.Constants.*;
import static frc.robot.Utilities.*;

import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    POVButton d_dpadUpButton = new POVButton(driverController, 0);
    POVButton d_dpadLeftButton = new POVButton(driverController, 270);
    POVButton d_dpadRightButton = new POVButton(driverController, 90);
    POVButton o_dpadUpButton = new POVButton(driverController, 0);
    POVButton o_dpadDownButton = new POVButton(driverController, 180);
    double time;

    public static final Intake intake = new Intake();
    public static final Arm arm = new Arm();
    public static final Climber climber = new Climber();
    public static final Shooter shooter = new Shooter();

    //this has to be the shittiest code in the whole project
    private static EventLoop triggerEventLoop = new EventLoop();

    // The robot's subsystems are defined here
    public static final Drivetrain drivetrain = new Drivetrain();

    // The drive team controllers are defined here
    public static final XboxController driverController = new XboxController(0);
    public static final XboxController operatorController = new XboxController(1);

    // Limits maximum speed
    private double maxSpeedFactor = .2;

    private ShuffleboardTab comp = Shuffleboard.getTab("Competition");
    private GenericEntry speedometer = comp.add("Speed Limit", 0.0)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("min", 0, "max", 1))
            .withPosition(1, 1)
            .withSize(2, 2)
            .getEntry();
    private ComplexWidget webcamWidget = comp.addCamera("Webcam", "webcam0", "10.4.24.2")
            .withPosition(3, 1)
            .withSize(4, 4);
    private ComplexWidget limeLightWidget = comp.addCamera("LimeLight", "limelight", "10.4.24.2")
            .withPosition(7, 1)
            .withSize(3, 4);

    public static SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Set up the default command for the drivetrain
        // The controls are for field-oriented driving
        // Left stick Y axis -> forward and backwards movement
        // Left stick X axis -> left and right movement
        // Right stick X axis -> rotation

        drivetrain.setDefaultCommand(new FieldOrientedDriveCommand(drivetrain,
                () -> -modifyAxis(driverController.getLeftY()) * MAX_VELOCITY_METERS_PER_SECOND * maxSpeedFactor,
                () -> -modifyAxis(driverController.getLeftX()) * MAX_VELOCITY_METERS_PER_SECOND * maxSpeedFactor,
                () -> -modifyAxis(driverController.getRightX()) * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
                        * maxSpeedFactor));

        intake.setDefaultCommand(new DefaultIntakeCommand(intake,
                () -> modifyAxis(operatorController.getRightTriggerAxis()),
                () -> modifyAxis(operatorController.getLeftTriggerAxis())));

        arm.setDefaultCommand(new DefaultArmCommand(arm,
                () -> modifyAxis(operatorController.getLeftY())));

        AprilTagAimCommand.makeShuffleboard();

        // register april aim with pathplanner, passing 0,0 as stick suppliers and
        // targeting speaker
        NamedCommands.registerCommand("AprilTagAim", new AprilTagAimCommand(drivetrain, "speaker"));
        NamedCommands.registerCommand("SpoolShooter", new InstantCommand(() -> Shooter.spool(Constants.SPOOL_VELOCITY)));
        NamedCommands.registerCommand("GroundPickUp", new GroundPickUpCommand());

        // Configure the button bindings
        configureButtonBindings();
        configureAutos();
    }

    /** Button -> command mappings are defined here. */
    private void configureButtonBindings() {
        // Driver Controller Bindings
        // Pressing A button zeros the gyroscope
        new Trigger(driverController::getAButton).onTrue(new InstantCommand(() -> drivetrain.zeroGyroscope()));
        // Pressing Y button locks the wheels in an X pattern
        new Trigger(driverController::getYButton).onTrue(new InstantCommand(() -> drivetrain.toggleWheelsLocked()));

        new Trigger(driverController::getRightBumper).whileTrue(new RunCommand(() -> speedThrottle()));
        // Pressing the Left Bumper shifts to low speed
        new Trigger(driverController::getLeftBumper).onTrue(new InstantCommand(() -> speedDown()));
        new Trigger(driverController::getXButton).toggleOnTrue(new RobotOrientedDriveCommand(drivetrain,
                () -> -modifyAxis(driverController.getLeftY()) * MAX_VELOCITY_METERS_PER_SECOND * maxSpeedFactor,
                () -> -modifyAxis(driverController.getLeftX()) * MAX_VELOCITY_METERS_PER_SECOND * maxSpeedFactor,
                () -> -modifyAxis(driverController.getRightX()) * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
                        * maxSpeedFactor,
                () -> -1));
        new Trigger(d_dpadUpButton::getAsBoolean).onTrue(new InstantCommand(() -> drivetrain.setMoves("default")));
        new Trigger(d_dpadLeftButton::getAsBoolean).onTrue(new InstantCommand(() -> drivetrain.setMoves("FL")));
        new Trigger(d_dpadRightButton::getAsBoolean).onTrue(new InstantCommand(() -> drivetrain.setMoves("FR")));

        driverController.leftTrigger(triggerEventLoop).ifHigh(() -> speedUp());

        new Trigger(o_dpadUpButton::getAsBoolean).onTrue(new InstantCommand(() -> climber.climb(0.3)));
        new Trigger(o_dpadDownButton::getAsBoolean).onTrue(new InstantCommand(() -> climber.climb(-0.3)));

        new Trigger(operatorController::getStartButtonPressed).onTrue(new InstantCommand(() -> Shooter.spool(Constants.SPOOL_VELOCITY)));
        new Trigger(operatorController::getBackButtonPressed).onTrue(new InstantCommand(() -> Shooter.stop()));
        //shooter pose
        new Trigger(operatorController::getBButton).whileTrue(new AprilTagAimCommand(drivetrain,
                "speaker",
                () -> -modifyAxis(driverController.getLeftY()) * MAX_VELOCITY_METERS_PER_SECOND * maxSpeedFactor,
                () -> -modifyAxis(driverController.getLeftX()) * MAX_VELOCITY_METERS_PER_SECOND * maxSpeedFactor));
        //amp pose
        new Trigger(operatorController::getYButton).onTrue(new InstantCommand(() -> arm.ampPose()));
        //source pose
        new Trigger(operatorController::getXButton).onTrue(new InstantCommand(() -> arm.sourcePose()));
        //ground pose
        new Trigger(operatorController::getAButton).onTrue(new InstantCommand(() -> arm.groundPose()));

        //TODO: remove this
        new Trigger(operatorController::getLeftBumper).onTrue(new InstantCommand(() -> arm.zeroBigEncoder()));
    }

    public void configureAutos() {
        autoChooser = AutoBuilder.buildAutoChooser();
        comp.add("Auto Chooser", autoChooser).withPosition(0, 4);
    }

    /**
     * This method returns the autonomous routine that will be run at the start of
     * the match.
     * <p>
     * For now, we only have one routine, so it just returns that one.
     * Once we have more than one routine, we will want to implement a chooser
     * dropdown on the dashboard.
     * 
     * @return The autonomous routine that will be run
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void speedThrottle() {
        if (driverController.getRightTriggerAxis() != 0) {
            maxSpeedFactor = driverController.getRightTriggerAxis();
            maxSpeedFactor = MathUtil.clamp(maxSpeedFactor, 0.1, 1.0);
            speedometer.setValue(maxSpeedFactor);
        }
    }

    public void speedMax() {
        maxSpeedFactor = 1;
        speedometer.setValue(maxSpeedFactor);
    }

    public void speedUp() {
        if (time - Timer.getFPGATimestamp()+.2 < 0){
            maxSpeedFactor += .1;
            maxSpeedFactor = MathUtil.clamp(maxSpeedFactor, .1, 1.);
            speedometer.setValue(maxSpeedFactor);
            time = Timer.getFPGATimestamp();
        }
    }

    public void speedDown() {
        maxSpeedFactor -= .1;
        maxSpeedFactor = MathUtil.clamp(maxSpeedFactor, .1, 1.);
        speedometer.setValue(maxSpeedFactor);
    }
    public static void pollEventLoop() { triggerEventLoop.poll(); }

    public void rumble() {
        if(Intake.getSwitch()) { 
            driverController.setRumble(RumbleType.kLeftRumble, 1.0); 
            driverController.setRumble(RumbleType.kRightRumble, 1.0);
        } else { 
            driverController.setRumble(RumbleType.kLeftRumble, 0.0); 
            driverController.setRumble(RumbleType.kRightRumble, 0.0);
        }
    }
}
