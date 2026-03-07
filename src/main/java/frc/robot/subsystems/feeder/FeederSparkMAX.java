package frc.robot.subsystems.feeder;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class FeederSparkMAX implements FeederMotor {
    private static final int MOTOR_ID = 10;

    private final SparkMax m_motor = new SparkMax(MOTOR_ID, MotorType.kBrushless);
    private final SparkMaxConfig m_config = new SparkMaxConfig();
    private final RelativeEncoder m_encoder;

    @SuppressWarnings("removal")
    public FeederSparkMAX() {
        m_config.idleMode(IdleMode.kBrake).smartCurrentLimit(40).inverted(true);
        m_motor.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_encoder = m_motor.getEncoder();
    }

    @Override
    public void setPercent(double percent) {
        m_motor.set(percent);
    }

    @Override
    public double getVelocityRPM() {
        return m_encoder.getVelocity();
    }

    @Override
    public void stop() {
        m_motor.set(0);
    }

    @Override
    public double getSupplyCurrent() {
        return m_motor.getOutputCurrent();
    }
}
