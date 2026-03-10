package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import java.lang.constant.DirectMethodHandleDesc;
import java.time.OffsetDateTime;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.vision.Limelight;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.DashboardStore;
import frc.robot.utils.ShooterTable;
import frc.robot.utils.ShooterTable.ShooterTableEntry;

public class ShooterSubsystem extends SubsystemBase {
    private final ShooterMotor m_shooter;
    private final Limelight m_limelight;
    private final DriveSubsystem m_DriveSubsystem;

    private static final double POINT_A_X = 8.0;
    private static final double POINT_A_Y = 4.0;
    private static final double POINT_B_X = 8.0;
    private static final double POINT_B_Y = -4.0;

    private double m_vx = 0.0;
    private double m_vy = 0.0;

    private Pose2d m_lastPose = null;
    private double m_lastTime = -1.0;

    private static final double kVelSmoothingAlpha = 0.2;

    public static final double SHOOTER_LOOKAHEAD_SECONDS = 0.1;

    private double m_target = 0.0;

    public ShooterSubsystem(Limelight limelight, DriveSubsystem drivetrain) {
        // m_shooter = new ShooterSparkMAX();
        m_limelight = limelight;
        m_DriveSubsystem = drivetrain;
        m_shooter = new ShooterTalonFX();

        setupDashboard();
    }

    private void setupDashboard() {
        DashboardStore.add("Shooter/RPM", this::getVelocityRPM);
        SmartDashboard.putNumber("Shooter/Target", 0.0);
    }

    @Override
    public void periodic() {
        Pose2d currentPose = m_DriveSubsystem.getRobotPoseEstimate();
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

        final boolean shooterReady = m_limelight.targetValid();
        double distance = m_limelight.getDistance();
        double xOffset = m_limelight.getXOffset();

        DashboardStore.add("shooter/Shooter Ready", () -> shooterReady);
        DashboardStore.add("shooter/Distance", () -> distance);
        DashboardStore.add("shooter/Offset", () -> xOffset);

    }

    private boolean isShotPlausible() {
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

        Optional<Pose3d> tagOpt = m_limelight.getFiducialPose3d();
        if (tagOpt.isEmpty()) return -1;

        // Prevent small errs
        if (m_vx == 0.0 && m_vy == 0.0) {
            // If we have no velocity, just return the current distance
            Pose3d tagPose = tagOpt.get();
            return Math.hypot(tagPose.getX(), tagPose.getY());
        }

        Pose2d botPose = m_DriveSubsystem.getRobotPoseEstimate();

        double lookAhead = getDynamicLookAhead();

        double predictedX = botPose.getX() + m_vx * lookAhead;
        double predictedY = botPose.getY() + m_vy * lookAhead;

        Pose3d tagPose = tagOpt.get();
        double dx = tagPose.getX() - predictedX;
        double dy = tagPose.getY() - predictedY;

        return Math.hypot(dx, dy);
    }

    public double predictedDistanceToPoint() {

        Pose2d botPose = m_DriveSubsystem.getRobotPoseEstimate();
        if (botPose == null) {
            return 0.0;
        }

        double lookAhead = getDynamicLookAhead();

        double rx = botPose.getX() + m_vx * lookAhead;
        double ry = botPose.getY() + m_vy * lookAhead;
        double heading = botPose.getRotation().getRadians();

        double fx = Math.cos(heading);
        double fy = Math.sin(heading);

        // Dist vecs
        double v1x = POINT_A_X - rx;
        double v1y = POINT_A_Y - ry;
        double v2x = POINT_B_X - rx;
        double v2y = POINT_B_Y - ry;

        // Use forward vec to compare which one is closer
        double dot1 = v1x * fx + v1y * fy;
        double dot2 = v2x * fx + v2y * fy;

        double dist1 = Math.hypot(v1x, v1y);
        double dist2 = Math.hypot(v2x, v2y);

        // Infer which one is closer
        if (dot1 >= 0.0 && dot2 < 0.0) {
            return dist1;
        } else if (dot2 >= 0.0 && dot1 < 0.0) {
            return dist2;
        } else {
            if (Double.compare(dot1, dot2) > 0) {
                return dist1;
            } else if (Double.compare(dot2, dot1) > 0) {
                return dist2;
            } else {
                return Math.min(dist1, dist2);
            }
        }

    }

    public ShooterTableEntry predictedTableEntryShooter() {

        double distance_prediction = getPredictedDistanceMeters();

        return ShooterTable.calcShooterTableEntryShooter(Meters.of(distance_prediction));
    }

    public ShooterTableEntry predictedTableEntryShuttle() {

        double distance_prediction = predictedDistanceToPoint();

        return ShooterTable.calcShooterTableEntryShuttle(Meters.of(distance_prediction));
    }

    public Command runShooterTableCommand(ShooterTableEntry entry) {
        return run(() -> m_shooter.setVelocityRPM(entry.wheelSpeed.in(RPM)));
    }

    public Command runRPMCommand(double rpm) {
        return run(() -> m_shooter.setVelocityRPM(rpm));
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
        ShooterTableEntry entry = predictedTableEntryShooter();
        if (isShotPlausible()) {
            return run(() -> m_shooter.setVelocityRPM(entry.wheelSpeed.in(RPM)));
        }
        return run(() -> m_shooter.setVelocityRPM(0));
    }

    public Command automaticShuttle() {
        ShooterTableEntry entry = predictedTableEntryShuttle();
        return run(() -> m_shooter.setVelocityRPM(entry.wheelSpeed.in(RPM)));
    }

    public double getVelocityRPM() {
        return m_shooter.getVelocityRPM();
    }
}