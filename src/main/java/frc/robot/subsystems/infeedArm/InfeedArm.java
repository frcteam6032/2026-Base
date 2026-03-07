package frc.robot.subsystems.infeedArm;

public interface InfeedArm {
    void setPercent(double percent);

    double getVelocityRPM();

    double getSupplyCurrent();

    void stop();

    void runToPosition(double position);


    double getPosition();

}
