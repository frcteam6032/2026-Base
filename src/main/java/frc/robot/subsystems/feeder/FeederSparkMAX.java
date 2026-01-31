package frc.robot.subsystems.feeder;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class FeederSparkMAX implements FeederMotor {
    private final SparkMax m_motor;
    private final RelativeEncoder m_encoder;

    private static final int MOTOR_ID = -1;

    @SuppressWarnings("removal")
    public FeederSparkMAX() {
        m_motor = new SparkMax(MOTOR_ID, MotorType.kBrushless);

        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.idleMode(IdleMode.kBrake).smartCurrentLimit(30).inverted(false);

        m_motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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
}
