package frc.robot.subsystems.motors;

public interface InfeedMotor {
    void setPercent(double percent);

    double getVelocityRPM();

    void stop();
}
