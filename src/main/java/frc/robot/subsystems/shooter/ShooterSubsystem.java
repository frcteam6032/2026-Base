package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.vision.Limelight;
import frc.robot.RobotContainer.FireType;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.DashboardStore;
import frc.robot.utils.ShooterTable;
import frc.robot.utils.ShooterTable.ShooterTableEntry;

public class ShooterSubsystem extends SubsystemBase {
    private final ShooterMotor m_shooter;
    private final Limelight m_limelight;
    private final DriveSubsystem m_driveSubsystem;

    private double m_targetRPM = 0.0;

    public static final double SHOOTER_LOOKAHEAD_SECONDS = 0.1;

    private double m_target = 6300.0;
    private double m_distance = -1.0;
    private double m_xOffset = 0.0;

    private static final double SHOOTER_COAST_VELOCITY = 700;

    public ShooterSubsystem(Limelight limelight, DriveSubsystem driveSubsystem) {
        // m_shooter = new ShooterSparkMAX();
        m_limelight = limelight;
        m_driveSubsystem = driveSubsystem;
        m_shooter = new ShooterTalonFX();

        setupDashboard();
    }

    private boolean isReady() {
        if (getVelocityRPM() > 1000 && Math.abs(getVelocityRPM() - m_targetRPM) < 200) {
            return true;
        }
        return false;
    }

    private void setupDashboard() {
        DashboardStore.add("Shooter/RPM", this::getVelocityRPM);
        DashboardStore.add("shooter/Distance", () -> m_distance);
        DashboardStore.add("shooter/Offset", () -> m_xOffset);
        DashboardStore.add("Shooter/Running", this::isReady);
        SmartDashboard.putNumber("Shooter/Target", m_target);
    }

    @Override
    public void periodic() {
        m_target = SmartDashboard.getNumber("Shooter/Target", 0.0);

        m_distance = m_limelight.getDistance();
        m_xOffset = m_limelight.getXOffset();
    }

    private void setVelocityRPM(double rpm) {
        m_targetRPM = rpm;
        m_shooter.setVelocityRPM(rpm);
    }

    public ShooterTableEntry predictedTableEntryShuttle(double dist) {
        return ShooterTable.calcShooterTableEntryShuttle(Feet.of(dist));
    }

    public ShooterTableEntry predictedTableEntryHub(double dist) {
        return ShooterTable.calcShooterTableEntryShooter(Feet.of(dist));
    }

    public Command runShooterTableCommand(ShooterTableEntry entry) {
        return run(() -> setVelocityRPM(entry.wheelSpeed.in(RPM)));
    }

    public Command coastCommand() {
        // gradually slow down to ~ the static velocity
        return run(() -> {
            if (getVelocityRPM() <= SHOOTER_COAST_VELOCITY + 100)
                setVelocityRPM(SHOOTER_COAST_VELOCITY);
            else
                m_shooter.stop();
        });
    }

    public Command runRPMCommand(double rpm) {
        return run(() -> setVelocityRPM(rpm));
    }

    public Command runPercentCommand(DoubleSupplier percent) {
        return run(() -> m_shooter.setPercent(percent.getAsDouble()));
    }

    public Command runTargetCommand() {
        return run(() -> m_shooter.setPercent(m_target));
    }

    public Command runTargetCommmandRPM() {
        return run(() -> setVelocityRPM(m_target));
    }

    public Command stopCommand() {
        return runOnce(() -> m_shooter.stop());
    }

    public Command automaticShuttle(DoubleSupplier dist) {
        return run(() -> {
            ShooterTableEntry entry = predictedTableEntryShuttle(dist.getAsDouble());
            setVelocityRPM(entry.wheelSpeed.in(RPM));
        });
    }

    public Command automaticHubShooter(DoubleSupplier dist, Supplier<FireType> mode) {
        return run(() -> {
            if (mode.get() == FireType.Assisted || mode.get() == FireType.Manual) {
                setVelocityRPM(2800);

            } else if (mode.get() == FireType.Automatic) {
                ShooterTableEntry entry = predictedTableEntryHub(dist.getAsDouble());
                setVelocityRPM(entry.wheelSpeed.in(RPM));
                // SmartDashboard.putNumber("Trying RPM", entry.wheelSpeed.in(RPM));
            } else if (mode.get() == FireType.Testing) {
                setVelocityRPM(500);
            }
        });
    }

    public double getShotProbability() {
        double shotProbability = 0.0;
        // Determine if a 3000 RPM shot would make it at the current distance.
        // We use the shooter table to find what distance corresponds to 3000 RPM.
        double currentDistance = m_distance;
        if (Double.isFinite(currentDistance)) {
            ShooterTableEntry requiredEntry = predictedTableEntryHub(currentDistance);
            double requiredRPM = requiredEntry.wheelSpeed.in(RPM);

            double error = Math.abs(requiredRPM - 3000.0);
            shotProbability = MathUtil.clamp(1.0 - (error / 500.0), 0.0, 1.0);
        } else {
            shotProbability = 0.0;
        }

        return shotProbability;
    }

    public double getVelocityRPM() {
        return m_shooter.getVelocityRPM();
    }

}