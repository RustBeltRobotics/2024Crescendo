package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Do NOT add any static variables to this class, or any initialization at all.
 * Unless you know what you are doing, do not modify this file except to change
 * he parameter class to the startRobot call.
 */
public final class Main {
    private Main() {
    }

    // Main initialization function. Do not perform any initialization here.
    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}
