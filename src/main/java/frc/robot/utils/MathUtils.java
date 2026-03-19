package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class MathUtils {
    public static double BASE_SPEED = 0.4;

    private static final double kTwistLookAheadBaseSeconds = 0.1;
    private static final double kTwistLookAheadMaxSeconds = 0.35;
    private static final double kTwistLookAheadSpeedScale = 0.02;

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

    public static double getTwistLookAheadSeconds(ChassisSpeeds speeds) {
        // Linearize speed for to get the lookahead amount
        double linearSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        double lookAhead = kTwistLookAheadBaseSeconds + (linearSpeed * kTwistLookAheadSpeedScale);
        return MathUtil.clamp(lookAhead, kTwistLookAheadBaseSeconds, kTwistLookAheadMaxSeconds);
    }

    public static Pose2d predictPose(Pose2d currentPose, ChassisSpeeds speeds, double lookAheadSeconds) {
        double cos = currentPose.getRotation().getCos();
        double sin = currentPose.getRotation().getSin();

        // Get the direcion vector of the bot
        double fieldVx = speeds.vxMetersPerSecond * cos - speeds.vyMetersPerSecond * sin;
        double fieldVy = speeds.vxMetersPerSecond * sin + speeds.vyMetersPerSecond * cos;

        double newX = currentPose.getX() + fieldVx * lookAheadSeconds;
        double newY = currentPose.getY() + fieldVy * lookAheadSeconds;
        Rotation2d newRotation = currentPose.getRotation()
                .plus(Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * lookAheadSeconds));

        return new Pose2d(newX, newY, newRotation);
    }

}