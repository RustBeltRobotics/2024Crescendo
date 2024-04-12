package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.DefaultArmCommand;
import frc.robot.commands.DefaultClimbCommand;
import frc.robot.commands.DefaultIntakeCommand;
import frc.robot.commands.FieldOrientedDriveCommand;
import frc.robot.commands.GroundPickUpCommand;
import frc.robot.commands.LockNoteCommand;
import frc.robot.commands.RobotOrientedDriveCommand;
import frc.robot.commands.SpeakerAimCommand;
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
    PowerDistribution thePDH = new PowerDistribution(PDH, ModuleType.kRev);

    // The drive team controllers are defined here
    public static final XboxController driverController = new XboxController(0);
    public static final XboxController operatorController = new XboxController(1);
    POVButton d_dpadUpButton = new POVButton(driverController, 0);
    POVButton d_dpadLeftButton = new POVButton(driverController, 270);
    POVButton d_dpadRightButton = new POVButton(driverController, 90);
    POVButton o_dpadUpButton = new POVButton(operatorController, 0);
    POVButton o_dpadDownButton = new POVButton(operatorController, 180);
    POVButton o_dpadLeftButton = new POVButton(operatorController, 270);
    POVButton o_dpadRightButton = new POVButton(operatorController, 90);
    // Limits maximum speed
    private double maxSpeedFactor = 1;

    private ShuffleboardTab comp = Shuffleboard.getTab("Competition");

    public static SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    SlewRateLimiter driveSlewRateLimiter = new SlewRateLimiter(0.5);
    //SlewRateLimiter rotationSlewRateLimiter = new SlewRateLimiter(0.5);
    private GroundPickUpCommand gpk = new GroundPickUpCommand();
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        drivetrain.setDefaultCommand(new FieldOrientedDriveCommand(drivetrain,
                () -> 
                    -modifyAxisGeneric(driverController.getLeftY(), 1.0, 0.05) * MAX_VELOCITY_METERS_PER_SECOND * Robot.driveEntry.getDouble(0),
                () ->
                    -modifyAxisGeneric(driverController.getLeftX(), 1.0, 0.05)* MAX_VELOCITY_METERS_PER_SECOND * Robot.driveEntry.getDouble(0),
                () -> -modifyAxisGeneric(driverController.getRightX(), 1.0, 0.05) * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * Robot.driveEntry.getDouble(0), 
                () -> driverController.getBButton()));
        
        intake.setDefaultCommand(new DefaultIntakeCommand(intake,
                () -> operatorController.getRightTriggerAxis() * Robot.intakeSpeedEntry.getDouble(0),
                () -> operatorController.getLeftTriggerAxis() * Robot.intakeSpeedEntry.getDouble(0)
        ));

        arm.setDefaultCommand(new DefaultArmCommand(arm, () -> -operatorController.getLeftY() * Robot.armSpeedEntry.getDouble(0)));

        climber.setDefaultCommand(new DefaultClimbCommand(climber, () -> operatorController.getRightY()));

        SpeakerAimCommand.makeShuffleboard();
        Intake.makeShuffleboard();

        // register commands with pathplanner
        NamedCommands.registerCommand("AprilTagAim", new SpeakerAimCommand(thePDH, arm, drivetrain));
        NamedCommands.registerCommand("SpoolShooter", new InstantCommand(() -> Shooter.spool(Constants.SPOOL_VELOCITY)));
        NamedCommands.registerCommand("StopShooter", new InstantCommand(() -> Shooter.stop()));
        NamedCommands.registerCommand("GroundPickUp", new GroundPickUpCommand());
        NamedCommands.registerCommand("FeedShooter", new InstantCommand(() -> Intake.feedShooter()));
        NamedCommands.registerCommand("RangedPose", new RepeatCommand(new InstantCommand(() -> arm.autoAim())));
        NamedCommands.registerCommand("LockNote", new LockNoteCommand());

        configureButtonBindings();
        configureAutos();
    }

    /** Button -> command mappings are defined here. */
    private void configureButtonBindings() {
        // Driver Controller Bindings ---

        // Pressing A button zeros the gyroscope
        new Trigger(driverController::getAButton).onTrue(new InstantCommand(() -> drivetrain.zeroGyroscope()).onlyIf(driverController::getBButton));
        // Pressing Y button locks the wheels in an X pattern
        // new Trigger(driverController::getYButton).onTrue(new InstantCommand(() -> drivetrain.toggleWheelsLocked()));

        // Pressing the X Button toggles between robot oriented and field oriented
        // new Trigger(driverController::getXButton).toggleOnTrue(new RobotOrientedDriveCommand(drivetrain,
        //        () -> -modifyAxis(driverController.getLeftY()) * MAX_VELOCITY_METERS_PER_SECOND * Robot.driveEntry.getDouble(0),
        //        () -> -modifyAxis(driverController.getLeftX()) * MAX_VELOCITY_METERS_PER_SECOND * Robot.driveEntry.getDouble(0),
        //        () -> -modifyAxis(driverController.getRightX()) * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * Robot.driveEntry.getDouble(0)));


        // Operator Controller Bindings ---

        // Pressing the Y Button rotates the arm to the amp pose
        
        new Trigger(operatorController::getYButton).whileTrue(new RepeatCommand(new InstantCommand(() -> arm.ampPose())));
        // Pressing the X Button initiates ground intake of a game piece
        new Trigger(operatorController::getXButton).onTrue(gpk);
        new Trigger(operatorController::getLeftBumper).onTrue(new InstantCommand(() -> gpk.cancel()));
        new Trigger(operatorController::getStartButton).onTrue(new LockNoteCommand());
        // Up D-pad is stop shooter
        new Trigger(o_dpadUpButton::getAsBoolean).onTrue(new InstantCommand(() -> Shooter.stop()));
        // Left D-pad is amp spool
        new Trigger(o_dpadLeftButton::getAsBoolean).onTrue(new InstantCommand(() -> Shooter.spool(Robot.shooterVelEntry.getDouble(0))));
        new Trigger(this::survivalMode).onTrue(new InstantCommand(() -> Robot.driveEntry.setDouble(.2)));
        new Trigger(this::survivalMode).onTrue(new InstantCommand(() -> Robot.intakeSpeedEntry.setDouble(.6)));
        new Trigger(this::survivalMode).onTrue(new InstantCommand(() -> Robot.armSpeedEntry.setDouble(.5)));
    }

    public void configureAutos() {
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("auto machine", autoChooser);
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

    public void speedThrottle() {
        if (driverController.getRightTriggerAxis() != 0) {
            maxSpeedFactor = driverController.getRightTriggerAxis();
            maxSpeedFactor = MathUtil.clamp(maxSpeedFactor, 0.1, 1.0);
        }
    }

    public void speedMax() {
        maxSpeedFactor = 1;
    }

    public void speedUp() {
    }

    public void speedDown() {
        maxSpeedFactor -= .1;
        maxSpeedFactor = MathUtil.clamp(maxSpeedFactor, .1, 1.);
    }
    public static void pollEventLoop() { 
        triggerEventLoop.poll(); 
    }

    public void rumble() {
        if(Intake.getSwitch()) {
            rumble(true);
        } else { 
            rumble(false);
        }
    }

    public void rumble(boolean set){
        if (set){
            operatorController.setRumble(RumbleType.kLeftRumble, .5); 
            operatorController.setRumble(RumbleType.kRightRumble, .5);
            driverController.setRumble(RumbleType.kLeftRumble, .5); 
            driverController.setRumble(RumbleType.kRightRumble, .5);
        } else {
            operatorController.setRumble(RumbleType.kLeftRumble, 0.0); 
            operatorController.setRumble(RumbleType.kRightRumble, 0.0);
            driverController.setRumble(RumbleType.kLeftRumble, 0.0); 
            driverController.setRumble(RumbleType.kRightRumble, 0.0);
        }
    }
    // This is an elegant solution (it took me 2 seconds and it works)
    public void forceVisionPose() {
        drivetrain.forceVisionPose();
    }
    public boolean survivalMode() {
        return !Robot.godModeEntry.getBoolean(false);
    }
}