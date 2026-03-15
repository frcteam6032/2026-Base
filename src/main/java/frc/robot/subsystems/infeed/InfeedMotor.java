package frc.robot.subsystems.infeed;

public interface InfeedMotor {
    void setPercent(double percent);

    double getVelocityRPM();
    double getSupplyCurrent();

    void setVelocityRPM(double rpm);

    void stop();
}
