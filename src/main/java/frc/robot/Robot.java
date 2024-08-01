package frc.robot;

import static frc.robot.Constants.COMPETITION_TAB;
import static frc.robot.Constants.LL_NAME;

import java.util.Map;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.util.LimelightHelpers;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 * 
 * Framework lifecycle hooks to initialize the robot and run the command scheduler.
 */
public class Robot extends TimedRobot {
    private RobotContainer robotContainer;
    private Command autonomousCommand;

    private GenericEntry timeEntry = COMPETITION_TAB.add("Time Left", 0.0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withPosition(3, 0)
            .withSize(4, 1)
            .withProperties(Map.of("min", 0, "max", 165))
            .getEntry();
    
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

        //Shuffleboard things belong here
        Intake.makeShuffleboard();

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
        timeEntry.setDouble(DriverStation.getMatchTime());
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
        if (DriverStation.getAlliance().get() == Alliance.Blue){
            LimelightHelpers.setPipelineIndex(LL_NAME, 0);
        } else if (DriverStation.getAlliance().get() == Alliance.Red) {
            LimelightHelpers.setPipelineIndex(LL_NAME, 0);
        } else {
            System.err.println("NO ALLIANCE, LL PIPELINE NOT SET");
        }
        // Stop the stupid controllers from vibrating all the time
        robotContainer.rumble(false);

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
