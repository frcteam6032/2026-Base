package frc.robot.subsystems.motors;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.FeedbackSensor;

import frc.robot.Constants;

public class InfeedSparkMAX implements InfeedMotor {
    private final SparkMax m_motor;
    private final RelativeEncoder m_encoder;
    private final SparkClosedLoopController m_closedLoop;

    @SuppressWarnings("removal")
    public InfeedSparkMAX(int canId) {
        m_motor = new SparkMax(canId, MotorType.kBrushless);

        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.idleMode(IdleMode.kBrake).smartCurrentLimit(30).inverted(false);

        cfg.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(Constants.InfeedConstants.INFEED_P,
                Constants.InfeedConstants.INFEED_I, Constants.InfeedConstants.INFEED_D).outputRange(-1, 1);

        m_motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_encoder = m_motor.getEncoder();
        m_closedLoop = m_motor.getClosedLoopController();
    }

    @Override
    public void setPercent(double percent) {
        m_closedLoop.setSetpoint(percent, ControlType.kDutyCycle);
    }

    @Override
    public double getVelocityRPM() {
        return m_encoder.getVelocity();
    }

    @Override
    public void stop() {
        m_closedLoop.setSetpoint(0.0, ControlType.kDutyCycle);
    }
}
