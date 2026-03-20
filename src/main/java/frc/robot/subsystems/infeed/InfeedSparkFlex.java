package frc.robot.subsystems.infeed;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;

public class InfeedSparkFlex implements InfeedMotor {
    private static final double kP = 0.00005 * 2.0;
    private static final double kFF = 13.0 / 6740.;
    private static final int MOTOR_ID = 12;

    private final SparkFlex m_motor = new SparkFlex(MOTOR_ID, MotorType.kBrushless);
    private final SparkMaxConfig m_config = new SparkMaxConfig();
    private final RelativeEncoder m_encoder = m_motor.getEncoder();

    private final SparkClosedLoopController m_closedLoop = m_motor.getClosedLoopController();
    private final FeedForwardConfig m_feedforward = new FeedForwardConfig().kV(kFF);

    public InfeedSparkFlex() {
        m_config.idleMode(IdleMode.kCoast)
                .smartCurrentLimit(90)
                .inverted(true)
                .voltageCompensation(12.0);

        m_config.closedLoop.p(kP).apply(m_feedforward);
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

    @Override
    public void setVelocityRPM(double rpm) {
        m_closedLoop.setSetpoint(rpm, ControlType.kVelocity);
    }

}
