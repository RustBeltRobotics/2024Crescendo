package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
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
import edu.wpi.first.wpilibj2.command.RepeatCommand;
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
import static frc.robot.util.Utilities.*;

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
    private static EventLoop triggerEventLoop = new EventLoop();

    // The robot's subsystems are defined here
    public static final Drivetrain drivetrain = new Drivetrain();
    public static final Intake intake = new Intake();
    public static final Arm arm = new Arm();
    public static final Climber climber = new Climber();
    public static final Shooter shooter = new Shooter();
    //TODO: get can id of the thing
    PowerDistribution thePDH = new PowerDistribution(5, ModuleType.kRev);

    // The drive team controllers are defined here
    public static final XboxController driverController = new XboxController(0);
    public static final XboxController operatorController = new XboxController(1);
    POVButton d_dpadUpButton = new POVButton(driverController, 0);
    POVButton d_dpadLeftButton = new POVButton(driverController, 270);
    POVButton d_dpadRightButton = new POVButton(driverController, 90);
    POVButton o_dpadUpButton = new POVButton(operatorController, 0);
    POVButton o_dpadDownButton = new POVButton(operatorController, 180);

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
    //private ComplexWidget limeLightWidget = comp.addCamera("LimeLight", "limelight", "10.4.24.2")
    //        .withPosition(7, 1)
    //        .withSize(3, 4);

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
        // TODO: depending on driver feedback, we may want to consider different
        // maxSpeedFactors for translation vs rotation
        // TODO: depending on driver feedback, we may want to consider different methods
        // of modifiy axis. Steeper ramp, shallower ramp, larger or smaller deadband,
        // etc.
        drivetrain.setDefaultCommand(new FieldOrientedDriveCommand(drivetrain,
                () -> -modifyAxis(driverController.getLeftY()) * MAX_VELOCITY_METERS_PER_SECOND * maxSpeedFactor,
                () -> -modifyAxis(driverController.getLeftX()) * MAX_VELOCITY_METERS_PER_SECOND * maxSpeedFactor,
                () -> -modifyAxis(driverController.getRightX()) * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
                        * maxSpeedFactor));
        // TODO: Do we want to run modifyAxis on intake commands? Do we want it to be
        // the same as what we do for driving input? If not, maybe we want a more
        // generic version of modifyAxis, I've taken a stab at this in Utilities, let me
        // know what you think
        intake.setDefaultCommand(new DefaultIntakeCommand(intake,
                () -> modifyAxis(operatorController.getRightTriggerAxis()),
                () -> modifyAxis(operatorController.getLeftTriggerAxis())));

        arm.setDefaultCommand(new DefaultArmCommand(arm,
                () -> operatorController.getLeftY()));

        AprilTagAimCommand.makeShuffleboard();
        Intake.makeShuffleboard();

        // register april aim with pathplanner, passing 0,0 as stick suppliers and
        // targeting speaker
        NamedCommands.registerCommand("AprilTagAim", new AprilTagAimCommand(drivetrain, "speaker"));
        NamedCommands.registerCommand("SpoolShooter", new InstantCommand(() -> Shooter.spool(Constants.SPOOL_VELOCITY)));
        NamedCommands.registerCommand("GroundPickUp", new GroundPickUpCommand());
        NamedCommands.registerCommand("FeedShooter", new InstantCommand(() -> Intake.feedShooter()));

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
        // Pressing Right Bumper sets the speed limit to the value on the Right Trigger
        new Trigger(driverController::getRightBumper).whileTrue(new RunCommand(() -> speedThrottle()));
        // Pressing the Left Bumper shifts to low speed
        new Trigger(driverController::getLeftBumper).onTrue(new InstantCommand(() -> speedDown()));
        // Toggles between robot oriented and field oriented
        new Trigger(driverController::getXButton).toggleOnTrue(new RobotOrientedDriveCommand(drivetrain,
                () -> -modifyAxis(driverController.getLeftY()) * MAX_VELOCITY_METERS_PER_SECOND * maxSpeedFactor,
                () -> -modifyAxis(driverController.getLeftX()) * MAX_VELOCITY_METERS_PER_SECOND * maxSpeedFactor,
                () -> -modifyAxis(driverController.getRightX()) * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
                        * maxSpeedFactor));
        // Automatically aims at speaker while the b button is held
        new Trigger(driverController::getBButton).whileTrue(new AprilTagAimCommand(drivetrain,
                "speaker",
                () -> -modifyAxis(driverController.getLeftY()) * MAX_VELOCITY_METERS_PER_SECOND * maxSpeedFactor,
                () -> -modifyAxis(driverController.getLeftX()) * MAX_VELOCITY_METERS_PER_SECOND * maxSpeedFactor, thePDH));

        // Bindings to change center of rotation
        new Trigger(d_dpadUpButton::getAsBoolean).onTrue(new InstantCommand(() -> drivetrain.setMoves("default")));
        new Trigger(d_dpadLeftButton::getAsBoolean).onTrue(new InstantCommand(() -> drivetrain.setMoves("FL")));
        new Trigger(d_dpadRightButton::getAsBoolean).onTrue(new InstantCommand(() -> drivetrain.setMoves("FR")));

        // If left trigger is pressed for 0.2 seconds, increase the speed limit
        driverController.leftTrigger(triggerEventLoop).debounce(0.2).ifHigh(() -> speedUp());

        // Activate climb motors
        new Trigger(o_dpadUpButton::getAsBoolean).whileTrue(new InstantCommand(() -> climber.climb(0.3)));
        new Trigger(o_dpadDownButton::getAsBoolean).whileTrue(new InstantCommand(() -> climber.climb(-0.3)));
        // Disable the climb motors when the buttons arent pressed
        new Trigger(o_dpadDownButton::getAsBoolean).whileFalse(new InstantCommand(() -> climber.stop())).and(() -> !o_dpadDownButton.getAsBoolean());

        // Pressing the Start Button spools up the shooter.
        new Trigger(operatorController::getStartButtonPressed).onTrue(new InstantCommand(() -> Shooter.spool(Constants.SPOOL_VELOCITY)));
        // Pressing the Back Button spools down the shooter.
        new Trigger(operatorController::getBackButtonPressed).onTrue(new InstantCommand(() -> Shooter.stop()));
        // Pressing the Y Button rotates the arm to the amp pose
        new Trigger(operatorController::getYButton).whileTrue(new RepeatCommand(new InstantCommand(() -> arm.ampPose())));        
        // Ground pose
        new Trigger(operatorController::getXButton).onTrue(new GroundPickUpCommand());
        // Pressing the A Button rotates the arm to the ground pose
        new Trigger(operatorController::getAButton).whileTrue(new RepeatCommand(new InstantCommand(() -> arm.groundPose())));
    }

    public void configureAutos() {
        autoChooser = AutoBuilder.buildAutoChooser();
        comp.add("Auto Chooser", autoChooser).withPosition(0, 4);
    }

    /**
     * This method returns the autonomous routine that will be run at the start of
     * the match.
     * 
     * @return The autonomous routine that will be run
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    // TODO: lets work with you and Andrzej on Thursday or Saturday to test out a
    // couple different throttle methods, and pick which one you guys like best,
    // that way we can clean these next four methods up a bit
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
        speedometer.setValue(maxSpeedFactor);
    }

    public void speedDown() {
        maxSpeedFactor -= .1;
        maxSpeedFactor = MathUtil.clamp(maxSpeedFactor, .1, 1.);
        speedometer.setValue(maxSpeedFactor);
    }
    public static void pollEventLoop() { triggerEventLoop.poll(); }

    public void rumble() {
        if(Intake.getSwitch()) {
            operatorController.setRumble(RumbleType.kLeftRumble, .5); 
            operatorController.setRumble(RumbleType.kRightRumble, .5);
        } else { 
            operatorController.setRumble(RumbleType.kLeftRumble, 0.0); 
            operatorController.setRumble(RumbleType.kRightRumble, 0.0);
        }
    }
}
