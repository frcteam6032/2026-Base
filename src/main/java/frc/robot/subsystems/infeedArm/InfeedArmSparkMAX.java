package frc.robot.subsystems.infeedArm;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.subsystems.shooter.ShooterMotor;

import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.FeedbackSensor;

/**
 * Spark MAX implementation of {@link ShooterMotor} using the REV Spark wrapper
 * present in this project.
 */
public class InfeedArmSparkMAX implements InfeedArm {
    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kFF = 1.0 / 5676.0; // rough kV (1 / freeSpeedRpm)

    public static final int MOTOR_ID = 9;

    private static final double FORWARD_LIMIT = 28;
    private static final double REVERSE_LIMIT = -2;

    private double m_target = 0;

    private final SparkMax m_motor = new SparkMax(MOTOR_ID, MotorType.kBrushless);
    private final SparkMaxConfig m_config = new SparkMaxConfig();
    private final FeedForwardConfig m_ff = new FeedForwardConfig();
    private final RelativeEncoder m_encoder = m_motor.getEncoder();
    private final SparkClosedLoopController m_closedLoop = m_motor.getClosedLoopController();

    @SuppressWarnings("removal")
    public InfeedArmSparkMAX() {
        m_ff.kV(kFF);

        m_config.idleMode(IdleMode.kCoast)
                .smartCurrentLimit(60)
                .inverted(false);

        m_config.softLimit
                .forwardSoftLimit(FORWARD_LIMIT)
                .reverseSoftLimit(REVERSE_LIMIT);

        m_config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(kP, kI, kD)
                .apply(m_ff)
                .outputRange(-1, 1);

        m_motor.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setPercent(double percent) {
        m_motor.set(percent);
    }

    public void setVelocityRPM(double rpm) {
        m_closedLoop.setSetpoint(rpm, ControlType.kVelocity);
    }

    public double getVelocityRPM() {
        return m_encoder.getVelocity();
    }

    public void stop() {
        m_motor.set(0);
    }

    public void runToPosition(double position) {
        m_target = position;
        m_closedLoop.setSetpoint(m_target, ControlType.kPosition);
    }

    public double getPosition() {
        return m_encoder.getPosition();
    }

    public DoubleSupplier positionSupplier() {
        return this::getPosition;
    }

    @Override
    public double getSupplyCurrent() {
        return m_motor.getOutputCurrent();
    }
}
