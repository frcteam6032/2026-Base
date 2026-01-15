package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    private final NetworkTable m_limelightTable;

    /**
     * Creates a new Limelight.
     */
    public Limelight() {
        m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    // Degrees offset on the X axis from the target origin
    public double getXOffset() {
        return m_limelightTable.getEntry("tx").getDouble(0);
    }

    // Degrees offset on the Y axis from the target origin
    public double getYOffset() {
        return m_limelightTable.getEntry("ty").getDouble(0);
    }

    // Returns if a target is found (-29.8 to 29.8 degrees in any axis)
    public boolean targetValid() {
        return (m_limelightTable.getEntry("tv").getDouble(0) == 1.0);
    }

    // Estimates in order: x, y, z, roll, pitch, yaw
    public Pose2d getBotPose() {
        var botPose = m_limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        return new Pose2d(botPose[0], botPose[1], Rotation2d.fromDegrees(botPose[5]));
    }

    // Latency (ms)
    public double getLatency() {
        double tl = m_limelightTable.getEntry("tl").getDouble(0); // Pipeline latency in milliseconds
        double cl = m_limelightTable.getEntry("cl").getDouble(0); // Capture latency in milliseconds
        return tl + cl;

    }

    public boolean getSide() {
        // True = left, false = right
        // Negate to correct side

        return (-getXOffset() < 0 ? true : false);
    }

    public double getDistance() {
        // Using the height of the target and the height of the camera, we can calculate
        // the distance
        // to the target using the formula:
        // distance = (targetHeight - cameraHeight) / tan(cameraAngle + targetAngle)
        // 22 inches from the floor to the camera
        // 26.5 inches from the floor to the target
        double targetHeightMeters = Units.inchesToMeters(12.125);
        double cameraHeightMeters = Units.inchesToMeters(19);
        double cameraAngle = -24.1;
        double targetAngle = getYOffset();
        double distance = (targetHeightMeters - cameraHeightMeters)
                / Math.tan(Math.toRadians(cameraAngle + targetAngle));

        return distance;
    }

    public double getTagId() {
        return m_limelightTable.getEntry("tid").getDouble(0);
    }

}