package frc.robot;

import static frc.robot.Constants.LL_NAME;

import java.util.Map;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.SpeakerAimCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;
import frc.robot.util.LimelightHelpers;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private RobotContainer robotContainer;
    private Command autonomousCommand;

    static ShuffleboardTab demo = Shuffleboard.getTab("demo");
    public static GenericEntry intakeEntry = demo.add("Intake", false).withWidget(BuiltInWidgets.kToggleSwitch).withPosition(3, 2).getEntry();
    public static GenericEntry shooterEntry = demo.add("Shooter", false).withWidget(BuiltInWidgets.kToggleSwitch).withPosition(3, 0).getEntry();
    public static GenericEntry armEntry = demo.add("arm", false).withWidget(BuiltInWidgets.kToggleSwitch).withPosition(3, 1).getEntry();
    public static GenericEntry armSetpointsEntry = demo.add("Arm Setpoints", false).withWidget(BuiltInWidgets.kToggleSwitch).withPosition(2, 1).getEntry();
    public static GenericEntry intakeLockEntry = demo.add("Intake lock button", false).withWidget(BuiltInWidgets.kToggleSwitch).withPosition(2, 0).getEntry();
    public static GenericEntry climberEntry = demo.add("climber", false).withWidget(BuiltInWidgets.kToggleSwitch).withPosition(3, 3).getEntry();
    public static GenericEntry shooterVelEntry = demo.add("Shooter vel", 2000).withWidget(BuiltInWidgets.kNumberSlider).withPosition(0, 2).withProperties(Map.of("min", 0, "max", 5000)).getEntry();
    public static GenericEntry driveEntry = demo.add("Drive Speed Control", 0.2).withWidget(BuiltInWidgets.kNumberSlider).withPosition(0, 3).withProperties(Map.of("min", 0, "max", 1)).getEntry();
    public static GenericEntry intakeSpeedEntry = demo.add("Intake Speed Control", 0.5).withWidget(BuiltInWidgets.kNumberSlider).withPosition(0, 0).withProperties(Map.of("min", 0, "max", 1)).getEntry();
    public static GenericEntry armSpeedEntry = demo.add("Arm Speed Control", 0.5).withWidget(BuiltInWidgets.kNumberSlider).withPosition(0, 1).withProperties(Map.of("min", 0, "max", 1)).getEntry();
    public static GenericEntry godModeEntry = demo.add("Creative Mode", false).withWidget(BuiltInWidgets.kToggleSwitch).withPosition(0, 4).getEntry();


    /**
     * This function is run once when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();

        // Port forwarding for LimeLight
        for (int port = 5800; port <= 5807; port++) {
            PortForwarder.add(port, "limelight.local", port);
        }

        // Set our relative encoder offset based on the position of the absolute encoder
        Arm.zeroThroughBoreRelative();

        // Start datalogging of all networkTables entrys onto the RIO
        DataLogManager.start();

        // Start datalogging of all DS data and joystick data onto the RIO
        DriverStation.startDataLog(DataLogManager.getLog());
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics that you want ran during disabled, autonomous, teleoperated
     * and test.
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        SmartDashboard.putNumber("dist: ", SpeakerAimCommand.getTagDistance());

        if (godModeEntry.getBoolean(false)) {
            driveEntry.setDouble(1);
            intakeSpeedEntry.setDouble(1);
            armSpeedEntry.setDouble(1);
        }
    }

    /**
     * This function is called once at the start of autonomous. It should be used to
     * send the correct autonomous routine to the command scheduler.
     */
    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }

        // Stop the stupid controllers from vibrating all the time
        robotContainer.rumble(false);

        // Force our LL to pipeline zero because we have paranoia
        LimelightHelpers.setPipelineIndex(LL_NAME, 0);

        // Full reset our pose using LL on init, done to avoid pose spiraling
        robotContainer.forceVisionPose();
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        RobotContainer.pollEventLoop();
        robotContainer.rumble();

        
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
        robotContainer.rumble(false);
        Shooter.stop();
    }

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {
    }

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
    }
}