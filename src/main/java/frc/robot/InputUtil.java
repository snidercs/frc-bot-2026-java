// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * Input shaping utilities for joystick axis values.
 *
 * Ported from inpututil.hpp in the C++ project.
 */
public final class InputUtil {
    private InputUtil() {}

    /**
     * Applies a normalized exponential curve to a raw joystick axis value.
     *
     * <p>The input is first checked against the deadband. If within the deadband,
     * returns 0. Otherwise the value is normalized so that the deadband edge maps
     * to 0 and full deflection maps to 1, the curve is applied, and the result is
     * scaled back with the original sign.
     *
     * <p>Desmos: <a href="https://www.desmos.com/calculator/btx9xycoaz">visualisation</a>
     *
     * <pre>
     *     normalized = (|raw| - deadband) / (1 - deadband)
     *     output     = sign(raw) * normalized^exponent
     * </pre>
     *
     * @param raw      Raw axis value in the range [-1, 1]
     * @param deadband Deadband threshold in the range [0, 1)
     * @param exponent Curve exponent (1.0 = linear, 2.0 = quadratic, etc.)
     * @return Shaped output value in the range [-1, 1]
     */
    public static double applyCurve(double raw, double deadband, double exponent) {
        if (Math.abs(raw) < deadband) {
            return 0.0;
        }

        double normalized = (Math.abs(raw) - deadband) / (1.0 - deadband);
        return Math.copySign(Math.pow(normalized, exponent), raw);
    }
}
