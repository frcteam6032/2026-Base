package frc.robot.subsystems.spindexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.DashboardStore;

public class SpindexerSubsystem extends SubsystemBase {
    private final SpindexerMotor m_spindexer;

    private double m_timeSeconds = 0.0;
    private boolean m_agitateReverse = false;

    public SpindexerSubsystem() {
        m_spindexer = new SpindexerSparkMAX();

        DashboardStore.add("Spindexer/Velocity", m_spindexer::getVelocityRPM);
        DashboardStore.add("Spindexer/Current", m_spindexer::getSupplyCurrent);
    }

    public Command agitateSpindexer() {
        // Simple one-shot “bump” backwards then forwards.
        // Note: using `run` (not `runOnce`) so the motor output is continuously applied
        // for the entire timeout duration.
        return run(() -> m_spindexer.setPercent(-0.5)).withTimeout(0.25)
                .andThen(run(() -> m_spindexer.setPercent(0.5)).withTimeout(0.25))
                .finallyDo(this::stop);
    }

    private void runPercent(double percent) {
        m_spindexer.setPercent(percent);
    }

    private void stop() {
        m_spindexer.stop();
    }

    public double getVelocityRPM() {
        return m_spindexer.getVelocityRPM();
    }

    public void spinAgitated(double percent) {
      
        final double dtSeconds = 0.02; // called from command scheduler (~50Hz)
        final double reverseDurationSeconds = 0.15;
        final double forwardDurationSeconds = 0.35;
        final double reversePercent = -Math.abs(percent);
        final double forwardPercent = Math.abs(percent);

        m_timeSeconds += dtSeconds;

        if (m_agitateReverse) {
            m_spindexer.setPercent(reversePercent);
            if (m_timeSeconds >= reverseDurationSeconds) {
                m_timeSeconds = 0.0;
                m_agitateReverse = false;
            }
        } else {
            m_spindexer.setPercent(forwardPercent);
            if (m_timeSeconds >= forwardDurationSeconds) {
                m_timeSeconds = 0.0;
                m_agitateReverse = true;
            }
        }
    }

    public Command spinCommand(double percent) {
        // Ensure the motor is stopped when the command finishes (button
        // release/interrupted)
        return run(() -> spinAgitated(percent)).finallyDo(this::stop);
    }

    public Command stopCommand() {
        return runOnce(this::stop);
    }
}
