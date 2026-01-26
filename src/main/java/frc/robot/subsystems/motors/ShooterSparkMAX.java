package frc.robot.subsystems.motors;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.FeedbackSensor;

import frc.robot.Constants;

/**
 * Spark MAX implementation of {@link ShooterMotor} using the REV Spark wrapper
 * present in this project.
 */
public class ShooterSparkMAX implements ShooterMotor {
    private final SparkMax m_motor;
    private final RelativeEncoder m_encoder;
    private final SparkClosedLoopController m_closedLoop;

    @SuppressWarnings("removal")
    public ShooterSparkMAX(int canId) {
        m_motor = new SparkMax(canId, MotorType.kBrushless);

        SparkMaxConfig cfg = new SparkMaxConfig();
        FeedForwardConfig ff = new FeedForwardConfig().kV(Constants.ShooterConstants.SHOOTER_FF);

        cfg.idleMode(IdleMode.kCoast)
                .smartCurrentLimit(40)
                .inverted(false);

        cfg.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(Constants.ShooterConstants.SHOOTER_P, Constants.ShooterConstants.SHOOTER_I,
                        Constants.ShooterConstants.SHOOTER_D)
                .apply(ff)
                .outputRange(-1, 1);

        m_motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_encoder = m_motor.getEncoder();
        m_closedLoop = m_motor.getClosedLoopController();
    }

    public void setPercent(double percent) {
        m_closedLoop.setSetpoint(percent, ControlType.kDutyCycle);
    }

    public void setVelocityRPM(double rpm) {
        // Spark wrapper expects velocity in the same units as encoder (RPM by default
        // unless
        // conversion factors applied). We'll set the closed-loop velocity setpoint.
        m_closedLoop.setSetpoint(rpm, ControlType.kVelocity);
    }

    public double getVelocityRPM() {
        return m_encoder.getVelocity();
    }

    public void stop() {
        m_closedLoop.setSetpoint(0.0, ControlType.kDutyCycle);
    }
}
