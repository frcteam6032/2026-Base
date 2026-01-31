package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ShooterTalonFX implements ShooterMotor {
    private static final int LEADER_ID = -1;
    private static final int FOLLOWER_ID = -1;

    final static CANBus kCANBus = CANBus.roboRIO();

    private static final TalonFX m_leaderMotor = new TalonFX(LEADER_ID, kCANBus);
    private static final TalonFX m_followerMotor = new TalonFX(FOLLOWER_ID, kCANBus);

    private static final TalonFXConfiguration m_motorCfg = new TalonFXConfiguration();
    private static final MotorOutputConfigs m_motorOutputCfg = m_motorCfg.MotorOutput;

    private static final VelocityDutyCycle m_request = new VelocityDutyCycle(0.0);
    private static final Follower m_followerRequest = new Follower(LEADER_ID, MotorAlignmentValue.Opposed);

    private static final double kP = 0.0005;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    private static final double TIMEOUT_SECONDS = 0.050;

    private static final double kFF = 1.0 / 6380.0; // rough kV (1 / freeSpeedRpm)

    ShooterTalonFX() {
        // set slot 0 gains and leave every other config factory-default
        var slot0Configs = m_motorCfg.Slot0;
        slot0Configs.kV = kFF;
        slot0Configs.kP = kP;
        slot0Configs.kI = kI;
        slot0Configs.kD = kD;
        m_motorOutputCfg.NeutralMode = NeutralModeValue.Coast;
        // apply all configs, 50 ms total timeout
        m_leaderMotor.getConfigurator().apply(m_motorCfg, TIMEOUT_SECONDS);
        m_followerMotor.getConfigurator().apply(m_motorCfg, TIMEOUT_SECONDS);
    }

    @Override
    public void setPercent(double percent) {
        m_leaderMotor.set(percent);
        m_followerMotor.setControl(m_followerRequest);
    }

    @Override
    public void setVelocityRPM(double rpm) {
        m_leaderMotor.setControl(m_request.withVelocity(rpm / 60));
        m_followerMotor.setControl(m_followerRequest);
    }

    @Override
    public double getVelocityRPM() {
        return m_leaderMotor.getVelocity().getValue().in(RPM);
    }

    @Override
    public void stop() {
        setVelocityRPM(0);
    }

    

}
