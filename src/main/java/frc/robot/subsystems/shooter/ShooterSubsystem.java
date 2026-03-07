package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.vision.Limelight;
import frc.robot.utils.DashboardStore;
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

    public static final double SHOOTER_LOOKAHEAD_SECONDS = 0.1;

    private double m_target = 0.0;

    public ShooterSubsystem(Limelight limelight) {
        m_shooter = new ShooterSparkMAX();
        m_limelight = limelight;
        // m_shooter = new ShooterTalonFX();

        setupDashboard();
    }

    private void setupDashboard() {
        DashboardStore.add("Shooter/RPM", this::getVelocityRPM);
        SmartDashboard.putNumber("Shooter/Target", 0.0);
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

        m_target = SmartDashboard.getNumber("Shooter/Target", 0.0);
    }

    private boolean isShotPlausible(double distanceMeters) {
        double offsetX = m_limelight.getXOffset();
        // If the x offset is large, we are prob looking at the edge or the vec proj is
        // not a chord
        if (Math.abs(offsetX) > 25.0) {
            return false;
        }
        return true;
    }

    private double getDynamicLookAhead() {
        // Mag of vel vecs
        double speed = Math.hypot(m_vx, m_vy);
        double baseLookAhead = 0.1;
        double maxLookAhead = 0.3;
        double lookAhead = baseLookAhead + speed * 0.01;
        return Math.min(lookAhead, maxLookAhead);
    }

    public double getPredictedDistanceMeters() {
        if (!m_limelight.targetValid()) {
            return Double.NaN;
        }

        Optional<Pose3d> tagOpt = m_limelight.getFiducialPose3d();

        if (tagOpt.isEmpty()) {
            return Double.NaN;
        }

        // Prevent small errs
        if (m_vx == 0.0 && m_vy == 0.0) {
            // If we have no velocity, just return the current distance
            Pose3d tagPose = tagOpt.get();
            return Math.hypot(tagPose.getX(), tagPose.getY());
        }

        Pose2d botPose = m_limelight.getBotPose();

        double lookAhead = getDynamicLookAhead();

        double predictedX = botPose.getX() + m_vx * lookAhead;
        double predictedY = botPose.getY() + m_vy * lookAhead;

        Pose3d tagPose = tagOpt.get();
        double dx = tagPose.getX() - predictedX;
        double dy = tagPose.getY() - predictedY;

        return Math.hypot(dx, dy);
    }

    public ShooterTableEntry predictedTableEntry() {
        if (!m_limelight.targetValid()) {
            return null;
        }

        double distance_prediction = getPredictedDistanceMeters();
        if (Double.isNaN(distance_prediction)) {
            return null;
        }

        return ShooterTable.calcShooterTableEntry(Meters.of(distance_prediction));
    }

    public Command runShooterTableCommand(ShooterTableEntry entry) {
        return run(() -> m_shooter.setVelocityRPM(entry.wheelSpeed.in(RPM)));
    }

    public Command runPercentCommand(DoubleSupplier percent) {
        return run(() -> m_shooter.setPercent(percent.getAsDouble()));
    }

    public Command runTargetCommand() {
        return run(() -> m_shooter.setPercent(m_target));
    }

    public Command stopCommand() {
        return runOnce(() -> m_shooter.stop());
    }

    // Dynamic shooter
    public Command automaticShooter() {
        ShooterTableEntry entry = predictedTableEntry();
        return run(() -> m_shooter.setVelocityRPM(entry.wheelSpeed.in(RPM)));
    }

    public double getVelocityRPM() {
        return m_shooter.getVelocityRPM();
    }
}