package frc.robot.subsystems.infeed;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class InfeedSubsystem {
    private final InfeedMotor m_infeed;

    public InfeedSubsystem() {
        m_infeed = new InfeedSparkMAX();
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
        return Commands.runOnce(() -> runPercent(percent));
    }

    public Command stopCommand() {
        return Commands.runOnce(this::stop);
    }
}
