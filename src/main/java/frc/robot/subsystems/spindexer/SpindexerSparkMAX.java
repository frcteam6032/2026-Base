package frc.robot.subsystems.spindexer;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class SpindexerSparkMAX implements SpindexerMotor {
    private static final int MOTOR_ID = -1;

    private final SparkMax m_motor = new SparkMax(MOTOR_ID, MotorType.kBrushless);
    private final SparkMaxConfig m_config = new SparkMaxConfig();
    private final RelativeEncoder m_encoder = m_motor.getEncoder();

    public SpindexerSparkMAX() {
        m_config.idleMode(IdleMode.kCoast).smartCurrentLimit(30).inverted(false);
        m_motor.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getSupplyCurrent'");
    }
}
