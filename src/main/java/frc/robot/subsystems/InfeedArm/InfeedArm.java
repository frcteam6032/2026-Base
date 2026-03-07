package frc.robot.subsystems.InfeedArm;

public interface InfeedArm {
    void setPercent(double percent);

    double getVelocityRPM();

    double getSupplyCurrent();

    void stop();

    void runToPosition(double position);


    double getPosition();

}
