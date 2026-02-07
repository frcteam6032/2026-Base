package frc.robot.subsystems.infeed;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;

public class InfeedSparkFlex implements InfeedMotor {

    private static final int MOTOR_ID = -1;

    private final SparkFlex m_motor = new SparkFlex(MOTOR_ID, MotorType.kBrushless);
    private final SparkMaxConfig m_config = new SparkMaxConfig();

    @SuppressWarnings("removal")
    public InfeedSparkFlex() {
        m_config.idleMode(IdleMode.kBrake).smartCurrentLimit(30).inverted(false);
        m_motor.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setPercent(double percent) {
        return;
    }

    @Override
    public double getVelocityRPM() {
        return 0;
    }

    @Override
    public void stop() {
        return;
    }

}
