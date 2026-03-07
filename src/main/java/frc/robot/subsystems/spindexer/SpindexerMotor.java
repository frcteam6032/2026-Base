package frc.robot.subsystems.spindexer;

public interface SpindexerMotor {
    void setPercent(double percent);

    double getVelocityRPM();
    double getSupplyCurrent();

    void stop();
}
