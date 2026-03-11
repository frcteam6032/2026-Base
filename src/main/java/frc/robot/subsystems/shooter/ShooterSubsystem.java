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

    public static final double SHOOTER_LOOKAHEAD_SECONDS = 0.1;

    private double m_target = 0.0;

    public ShooterSubsystem(Limelight limelight) {
        // m_shooter = new ShooterSparkMAX();
        m_limelight = limelight;
        m_shooter = new ShooterTalonFX();

        setupDashboard();
    }

    private void setupDashboard() {
        DashboardStore.add("Shooter/RPM", this::getVelocityRPM);
        SmartDashboard.putNumber("Shooter/Target", 0.0);
    }

    @Override
    public void periodic() {
        m_target = SmartDashboard.getNumber("Shooter/Target", 0.0);

        final boolean shooterReady = m_limelight.targetValid();
        double distance = m_limelight.getDistance();
        double xOffset = m_limelight.getXOffset();

        DashboardStore.add("shooter/Shooter Ready", () -> shooterReady);
        DashboardStore.add("shooter/Distance", () -> distance);
        DashboardStore.add("shooter/Offset", () -> xOffset);

    }

    public ShooterTableEntry predictedTableEntryShuttle(double dist) {
        return ShooterTable.calcShooterTableEntryShuttle(Meters.of(dist));
    }

    public ShooterTableEntry predictedTableEntryHub(double dist) {

        return ShooterTable.calcShooterTableEntryShooter(Meters.of(dist));
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

    public Command runTargetCommmandRPM() {
        return run(() -> m_shooter.setVelocityRPM(m_target));
    }

    public Command stopCommand() {
        return runOnce(() -> m_shooter.stop());
    }

    public Command automaticShuttle(double dist) {
        ShooterTableEntry entry = predictedTableEntryShuttle(dist);
        return run(() -> m_shooter.setVelocityRPM(entry.wheelSpeed.in(RPM)));
    }

    public Command automaticHubShooter(double dist) {
        ShooterTableEntry entry = predictedTableEntryHub(dist);
        return run(() -> m_shooter.setVelocityRPM(entry.wheelSpeed.in(RPM)));
    }

    public double getVelocityRPM() {
        return m_shooter.getVelocityRPM();
    }
}