package frc.robot.subsystems.infeed;

public interface InfeedMotor {
    void setPercent(double percent);

    double getVelocityRPM();

    void stop();
}
