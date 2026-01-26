package frc.robot.subsystems.motors;

public interface ShooterMotor {
    void setPercent(double percent);

    void setVelocityRPM(double rpm);

    double getVelocityRPM();

    void stop();
}
