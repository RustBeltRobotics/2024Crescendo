package frc.robot.util;

import edu.wpi.first.math.MathUtil;

public class Utilities {
    /**
     * This method is used to filter driver input.
     * <p>
     * First it applies a deadband to the axis value. Then, it squares the value, keeping the same sign as the original value.
     * @param value The value you want to modify
     * @return The filtered value
     */
    public static double modifyAxis(double value) {
        // Deadband
        value = MathUtil.applyDeadband(value, 0.05);
        // Square the axis
        value = Math.copySign(value * value, value);
        return value;
    }
}
