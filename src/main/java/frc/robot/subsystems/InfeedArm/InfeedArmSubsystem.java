package frc.robot.subsystems.InfeedArm;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.DashboardStore;

public class InfeedArmSubsystem extends SubsystemBase {
    private final InfeedArm m_infeed;

    private final double STOW_POSITION = -1;
    private final double DEPLOY_POSITION = -1;

    // 0 is stow, 1 is deploy
    private int lastPosition = 0;

    public InfeedArmSubsystem() {
        m_infeed = new InfeedArmSparkMAX();
        // m_infeed = new InfeedTalonFX();
        DashboardStore.add("InfeedArm/Velocity", m_infeed::getVelocityRPM);
        DashboardStore.add("InfeedArm/Current", m_infeed::getSupplyCurrent);
        DashboardStore.add("InfeedArm/Position", m_infeed::getPosition);

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

    public Command goToDeploy() {
        return run(() -> m_infeed.runToPosition(DEPLOY_POSITION));
    }

    public Command goToStow() {
        return run(() -> m_infeed.runToPosition(STOW_POSITION));
    }

    public Command switchPosition() {
        if (lastPosition == 0) {
            lastPosition = 1;
            return run(() -> m_infeed.runToPosition(DEPLOY_POSITION));
        } else {
            lastPosition = 0;
            return run(() -> m_infeed.runToPosition(STOW_POSITION));
        }
    }

    public Command stopCommand() {
        return runOnce(this::stop);
    }
}
