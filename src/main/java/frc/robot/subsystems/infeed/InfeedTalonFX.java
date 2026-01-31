package frc.robot.subsystems.infeed;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class InfeedTalonFX implements InfeedMotor {
    private static final int LEADER_ID = -1;

    final static CANBus kCANBus = CANBus.roboRIO();

    private static final TalonFX m_leaderMotor = new TalonFX(LEADER_ID, kCANBus);

    private static final TalonFXConfiguration m_motorCfg = new TalonFXConfiguration();
    private static final MotorOutputConfigs m_motorOutputCfg = m_motorCfg.MotorOutput;

    private static final double TIMEOUT_SECONDS = 0.050;

    InfeedTalonFX() {

        m_motorOutputCfg.NeutralMode = NeutralModeValue.Coast;
        // apply all configs, 50 ms total timeout
        m_leaderMotor.getConfigurator().apply(m_motorCfg, TIMEOUT_SECONDS);
    }

    @Override
    public void setPercent(double percent) {
        m_leaderMotor.set(percent);
    }

    @Override
    public double getVelocityRPM() {
        return m_leaderMotor.getVelocity().getValue().in(RPM);
    }

    @Override
    public void stop() {
        setPercent(0);
    }

}
