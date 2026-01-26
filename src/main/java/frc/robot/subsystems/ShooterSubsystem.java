package frc.robot.subsystems;

import java.util.Optional;
import java.util.OptionalDouble;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.subsystems.motors.ShooterMotor;
import frc.robot.subsystems.motors.ShooterSparkMAX;
import frc.robot.vision.Limelight;
import frc.robot.utils.ShooterTable;
import frc.robot.utils.ShooterTable.ShooterTableEntry;

public class ShooterSubsystem extends SubsystemBase {
    private final ShooterMotor m_shooter;
    private final Limelight m_limelight;

    private double m_vx = 0.0;
    private double m_vy = 0.0;

    private Pose2d m_lastPose = null;
    private double m_lastTime = -1.0;

    private static final double kVelSmoothingAlpha = 0.2;

    private double m_lookaheadSeconds = Constants.ShooterConstants.SHOOTER_LOOKAHEAD_SECONDS;

    public ShooterSubsystem() {
        m_shooter = new ShooterSparkMAX(Constants.ShooterConstants.SHOOTER_CAN_ID);
        m_limelight = new Limelight();
    }

    @Override
    public void periodic() {
        if (!m_limelight.targetValid()) {
            // If no valid target we still may be able to read botpose; but skip if not
            return;
        }

        Pose2d currentPose = m_limelight.getBotPose();
        double now = Timer.getFPGATimestamp();

        if (m_lastPose != null && (now - m_lastTime) > 1e-6) {
            double dt = now - m_lastTime;
            double dx = currentPose.getX() - m_lastPose.getX();
            double dy = currentPose.getY() - m_lastPose.getY();
            double rawVx = dx / dt;
            double rawVy = dy / dt;

            // Low-pass smooth the velocity
            m_vx = kVelSmoothingAlpha * rawVx + (1.0 - kVelSmoothingAlpha) * m_vx;
            m_vy = kVelSmoothingAlpha * rawVy + (1.0 - kVelSmoothingAlpha) * m_vy;
        }

        m_lastPose = currentPose;
        m_lastTime = now;
    }

    public void setLookaheadSeconds(double seconds) {
        m_lookaheadSeconds = Math.max(0.0, seconds);
    }

    public double getPredictedDistanceMeters() {
        if (!m_limelight.targetValid()) {
            return Double.NaN;
        }

        Optional<Pose3d> tagOpt = m_limelight.getFiducialPose3d();
        if (tagOpt.isEmpty()) {
            return Double.NaN;
        }

        Pose2d botPose = m_limelight.getBotPose();

        double predictedX = botPose.getX() + m_vx * m_lookaheadSeconds;
        double predictedY = botPose.getY() + m_vy * m_lookaheadSeconds;

        Pose3d tagPose = tagOpt.get();
        double dx = tagPose.getX() - predictedX;
        double dy = tagPose.getY() - predictedY;

        return Math.hypot(dx, dy);
    }

    public double calculateWheelRPMForDistance(double distanceMeters) {
        return Constants.ShooterConstants.SHOOTER_BASE_RPM
                + Constants.ShooterConstants.SHOOTER_RPM_PER_METER * distanceMeters;
    }

    public ShooterTableEntry predictedTableEntry() {
        if (!m_limelight.targetValid()) {
            return null;
        }

        double distance_prediction = getPredictedDistanceMeters();
        if (Double.isNaN(distance_prediction)) {
            return null;
        }

        return ShooterTable.calcShooterTableEntry(distance_prediction);
    }

    public boolean applySpeed(boolean useVelocityMapping) {
        ShooterTableEntry entry = predictedTableEntry();
        if (entry == null)
            return false;

        if (useVelocityMapping) {
            double rpm = calculateWheelRPMForDistance(getPredictedDistanceMeters());
            m_shooter.setVelocityRPM(rpm);
        } else {
            m_shooter.setPercent(entry.beans);
        }

        return true;
    }

    public void runPercent(double percent) {
        m_shooter.setPercent(percent);
    }

    public void stop() {
        m_shooter.stop();
    }

    public double getShooterVelocityRPM() {
        return m_shooter.getVelocityRPM();
    }
}
