package frc.robot.subsystems.infeed;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkFlex;

public class InfeedSparkFlex implements InfeedMotor {
    private static final int MOTOR_ID = 12;

    private final SparkFlex m_motor = new SparkFlex(MOTOR_ID, MotorType.kBrushless);
    private final SparkMaxConfig m_config = new SparkMaxConfig();
    private final RelativeEncoder m_encoder = m_motor.getEncoder();

    public InfeedSparkFlex() {
        m_config.idleMode(IdleMode.kCoast)
                .smartCurrentLimit(60)
                .inverted(true)
                .voltageCompensation(12.0);
        m_motor.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setPercent(double percent) {
        m_motor.set(percent);
    }

    @Override
    public double getVelocityRPM() {
        // Max. RPM: 6740
        return m_encoder.getVelocity();
    }

    @Override
    public void stop() {
        setPercent(0.0);
    }

    @Override
    public double getSupplyCurrent() {
        return m_motor.getOutputCurrent();
    }

}
