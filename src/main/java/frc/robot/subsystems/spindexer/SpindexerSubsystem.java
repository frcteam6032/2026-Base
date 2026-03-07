package frc.robot.subsystems.spindexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.DashboardStore;

public class SpindexerSubsystem extends SubsystemBase {
    private final SpindexerMotor m_spindexer;

    public SpindexerSubsystem() {
        m_spindexer = new SpindexerSparkMAX();

        DashboardStore.add("Spindexer/Velocity", m_spindexer::getVelocityRPM);
        DashboardStore.add("Spindexer/Current", m_spindexer::getSupplyCurrent);
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

    public Command spinCommand(double percent) {
        return run(() -> runPercent(percent));
    }

    public Command stopCommand() {
        return runOnce(this::stop);
    }
}
