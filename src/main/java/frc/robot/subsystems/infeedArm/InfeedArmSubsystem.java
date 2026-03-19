package frc.robot.subsystems.infeedArm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.DashboardStore;

public class InfeedArmSubsystem extends SubsystemBase {
    private static final double OSCILLATE_STEP_DEGREES = 3;
    private static final double OSCILLATE_HOLD_SECONDS = 0.5;

    public final InfeedArm m_infeed;

    enum Location {
        Stow(0),
        Deploy(30);

        public int Position;

        private Location(int pos) {
            Position = pos;
        }
    }

    private Location m_currentPosition = Location.Stow;
    private double m_target;

    // if this is true, we are using a custom position.
    private boolean m_override = false;

    public InfeedArmSubsystem() {
        m_infeed = new InfeedArmSparkMAX();
        // m_infeed = new InfeedTalonFX();
        DashboardStore.add("InfeedArm/Velocity", m_infeed::getVelocityRPM);
        DashboardStore.add("InfeedArm/Current", m_infeed::getSupplyCurrent);
        DashboardStore.add("InfeedArm/Position", m_infeed::getPosition);
        DashboardStore.add("InfeedArm/Target", () -> m_target);
    }

    public Command vbusCMD(double pct) {
        return run(() -> m_infeed.setPercent(pct));
    }

    private void stop() {
        m_infeed.stop();
    }

    public double getVelocityRPM() {
        return m_infeed.getVelocityRPM();
    }

    private void setCurrentPosition(Location loc) {
        m_override = false;
        m_currentPosition = loc;
        m_target = loc.Position;
    }

    public Command goToDeploy() {
        return runOnce(() -> setCurrentPosition(Location.Deploy));
    }

    public Command goToStow() {
        return runOnce(() -> setCurrentPosition(Location.Stow));
    }

    private void switchPosition() {
        switch (m_currentPosition) {
            case Deploy:
                m_currentPosition = Location.Stow;
                break;
            case Stow:
                m_currentPosition = Location.Deploy;
                break;
            default:
                m_currentPosition = Location.Stow;
                break;
        }

        m_target = m_currentPosition.Position;
    }

    public Command switchPositionCommand() {
        return runOnce(this::switchPosition);
    }

    private void bump(double bump) {
        if (!m_override) {
            m_override = true;
            m_target = m_currentPosition.Position;
        }

        m_target += bump;
    }

    private void bumpArmUp() {
        bump(OSCILLATE_STEP_DEGREES);
    }

    private void bumpArmDown() {
        bump(-OSCILLATE_STEP_DEGREES);
    }

    private void resetToPresetTarget() {
        m_override = false;
        m_target = m_currentPosition.Position;
    }

    public Command oscillateArmCommand() {
        Command oscillateCycle = Commands.repeatingSequence(
                runOnce(this::bumpArmUp),
                Commands.waitSeconds(OSCILLATE_HOLD_SECONDS),
                runOnce(this::bumpArmDown),
                Commands.waitSeconds(OSCILLATE_HOLD_SECONDS));

        return Commands.sequence(
                runOnce(() -> {
                    if (!m_override) {
                        m_override = true;
                        m_target = m_currentPosition.Position;
                    }
                }),
                oscillateCycle).finallyDo(interrupted -> resetToPresetTarget());
    }

    public Command bumpCommand(double bump) {
        return runOnce(() -> this.bump(bump));
    }

    public Command stopCommand() {
        return runOnce(this::stop);
    }

    @Override
    public void periodic() {
        m_infeed.runToPosition(m_target);
    }
}
