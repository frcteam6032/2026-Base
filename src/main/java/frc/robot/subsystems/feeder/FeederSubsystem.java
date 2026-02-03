package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase {
    private final FeederMotor m_feeder;

    public FeederSubsystem() {
        m_feeder = new FeederSparkMAX();
    }

    private void runPercent(double percent) {
        m_feeder.setPercent(percent);
    }

    private void stop() {
        m_feeder.stop();
    }

    public double getVelocityRPM() {
        return m_feeder.getVelocityRPM();
    }

    public Command intakeCommand(double percent) {
        return run(() -> runPercent(percent));
    }

    public Command stopCommand() {
        return runOnce(this::stop);
    }
}
