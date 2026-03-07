// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

/**
 * Tests for InputUtil — ported from inpututiltest.cpp in the C++ project.
 */
class InputUtilTest {

    private static final double EPSILON = 1e-9;

    @Test
    void applyCurveReturnsZeroInsideDeadband() {
        assertEquals(0.0, InputUtil.applyCurve(0.0, 0.05, 2.0), EPSILON);
        assertEquals(0.0, InputUtil.applyCurve(0.04, 0.05, 2.0), EPSILON);
        assertEquals(0.0, InputUtil.applyCurve(-0.04, 0.05, 2.0), EPSILON);
    }

    @Test
    void applyCurveReturnsFullAtExtremes() {
        assertEquals(1.0, InputUtil.applyCurve(1.0, 0.05, 2.0), EPSILON);
        assertEquals(-1.0, InputUtil.applyCurve(-1.0, 0.05, 2.0), EPSILON);
    }

    @Test
    void applyCurveLinearExponent() {
        // With exponent=1 the response should be linear after deadband removal
        double deadband = 0.1;
        double raw = 0.55;
        double expected = (0.55 - 0.1) / (1.0 - 0.1); // 0.5
        assertEquals(expected, InputUtil.applyCurve(raw, deadband, 1.0), EPSILON);
    }

    @Test
    void applyCurveWithZeroDeadbandLinear() {
        // No deadband, linear exponent → identity
        assertEquals(0.5, InputUtil.applyCurve(0.5, 0.0, 1.0), EPSILON);
        assertEquals(-0.5, InputUtil.applyCurve(-0.5, 0.0, 1.0), EPSILON);
    }

    @Test
    void applyCurvePreservesSign() {
        double pos = InputUtil.applyCurve(0.8, 0.1, 2.5);
        double neg = InputUtil.applyCurve(-0.8, 0.1, 2.5);
        assertTrue(pos > 0);
        assertTrue(neg < 0);
        assertEquals(pos, -neg, EPSILON);
    }
}
