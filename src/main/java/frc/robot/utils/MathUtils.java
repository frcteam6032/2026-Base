package frc.robot.utils;

import java.util.stream.DoubleStream;

import edu.wpi.first.math.filter.SlewRateLimiter;

public class MathUtils {
    private static final double BASE_SPEED = 0.4;

    public static double scaleDriverController(double controllerInput, SlewRateLimiter limiter, double trigger) {
        return limiter.calculate(
                controllerInput * (BASE_SPEED
                        + trigger * (1 - BASE_SPEED)));
    }

    public static double normalizeVector(double vector, double magnitude) {
        return vector / magnitude;
    }
    
    public static double getMagnitudeVector(double x, double y) {
        return Math.sqrt(x * x + y * y);
    }
}