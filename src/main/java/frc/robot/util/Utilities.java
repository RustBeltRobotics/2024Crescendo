package frc.robot.util;

import edu.wpi.first.math.MathUtil;

public class Utilities {
    /**
     * This method is used to filter driver input.
     * <p>
     * First it applies a deadband to the axis value. Then, it squares the value,
     * keeping the same sign as the original value.
     * 
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

    /**
     * This method is used to filter driver input.
     * <p>
     * First it applies a deadband to the axis value. Then, it raises the value to a
     * given power, keeping the same sign as the original value.
     * 
     * @param value    The value you want to modify
     * @param power    The expmonent you want to raise the value to the power of.
     *                 Can be any positive double, non-integers work too, for
     *                 example 0.5 to take the square root.
     * @param deadband The width of the deadband to apply
     * @return The filtered value
     */
    public static double modifyAxisGeneric(double value, double power, double deadband) {
        // Deadband
        value = MathUtil.applyDeadband(value, deadband);
        // Exponent the input
        value = Math.copySign(Math.pow(Math.abs(value), power), value);
        return value;
    }
}
