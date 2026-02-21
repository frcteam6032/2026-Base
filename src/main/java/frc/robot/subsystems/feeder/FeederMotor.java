package frc.robot.subsystems.feeder;

public interface FeederMotor {
    void setPercent(double percent);

    double getVelocityRPM();
    double getSupplyCurrent();

    void stop();
}
