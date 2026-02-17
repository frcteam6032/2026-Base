package frc.robot.subsystems.infeed;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class InfeedTalonFX implements InfeedMotor {
    private static final int LEADER_ID = 9;

    private final CANBus kCANBus = CANBus.roboRIO();

    private final TalonFX m_motor = new TalonFX(LEADER_ID, kCANBus);

    private final TalonFXConfiguration m_motorCfg = new TalonFXConfiguration();
    private final MotorOutputConfigs m_motorOutputCfg = m_motorCfg.MotorOutput;

    private final double TIMEOUT_SECONDS = 0.050;

    InfeedTalonFX() {
        m_motorOutputCfg.NeutralMode = NeutralModeValue.Coast;
        m_motorOutputCfg.Inverted = InvertedValue.Clockwise_Positive;
        // apply all configs, 50 ms total timeout
        m_motor.getConfigurator().apply(m_motorCfg, TIMEOUT_SECONDS);
    }

    @Override
    public void setPercent(double percent) {
        m_motor.set(percent);
    }

    @Override
    public double getVelocityRPM() {
        return m_motor.getVelocity().getValue().in(RPM);
    }

    @Override
    public void stop() {
        setPercent(0);
    }

    @Override
    public double getSupplyCurrent() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getSupplyCurrent'");
    }

}
