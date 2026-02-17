package frc.robot.subsystems.infeed;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.DashboardStore;

public class InfeedSubsystem extends SubsystemBase {
    private final InfeedMotor m_infeed;

    public InfeedSubsystem() {
        // m_infeed = new InfeedSparkMAX();
        // m_infeed = new InfeedTalonFX();
        m_infeed = new InfeedSparkFlex();

        DashboardStore.add("Infeed/Velocity", m_infeed::getVelocityRPM);
        DashboardStore.add("Infeed/Current", m_infeed::getSupplyCurrent);
    }

    private void runPercent(double percent) {
        m_infeed.setPercent(percent);
    }

    private void stop() {
        m_infeed.stop();
    }

    public double getVelocityRPM() {
        return m_infeed.getVelocityRPM();
    }

    public Command intakeCommand(double percent) {
        return run(() -> runPercent(percent));
    }

    public Command stopCommand() {
        return runOnce(this::stop);
    }
}
