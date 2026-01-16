package frc.robot.vision;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    private final NetworkTable m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

    // height of the center of the lens from the floor
    private static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(19);

    // angle of camera from vertical
    // positive is pointing down
    private static final double CAMERA_ANGLE = -24.1;

    private final AprilTagFieldLayout m_layout;

    /**
     * Creates a new Limelight.
     */
    public Limelight() {
        // TODO: WPILib doesn't have 2026 field yet. It should be in 2026.1.2
        // cc: https://git.crueter.xyz/mirror/allwpilib/commit/18249badc0
        m_layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    }

    private double getDouble(String key) {
        return m_limelightTable.getEntry(key).getDouble(0);
    }

    /**
     * Get the X offset of the current target.
     * @return The X offset in degrees.
     */
    public double getXOffset() {
        return getDouble("tx");
    }

    /**
     * Get the Y offset of the current target.
     * @return The Y offset in degrees.
     */
    public double getYOffset() {
        return getDouble("ty");
    }

    /**
     * Get the ID of the currently targeted fiducial.
     * @return The tag ID of the current fiducial target.
     */
    public double getFiducialID() {
        return getDouble("tid");
    }

    /**
     * Whether or not there is a valid target on screen.
     * @return true if there is a valid target, false otherwise
     */
    public boolean targetValid() {
        return (getDouble("tv") == 1.0);
    }

    // Estimates in order: x, y, z, roll, pitch, yaw
    public Pose2d getBotPose() {
        // TODO: red/blue stuff
        var botPose = m_limelightTable.getEntry("botpose_orb_wpiblue").getDoubleArray(new double[6]);
        return new Pose2d(botPose[0], botPose[1], Rotation2d.fromDegrees(botPose[5]));
    }

    // Latency (ms)
    public double getLatency() {
        double tl = m_limelightTable.getEntry("tl").getDouble(0); // Pipeline latency in milliseconds
        double cl = m_limelightTable.getEntry("cl").getDouble(0); // Capture latency in milliseconds
        return tl + cl;
    }

    /**
     * Sets the robot's orientation on the field.
     * Use this for MegaTag2. This must be called at least once per frame
     * @param angle The {@link Rotation2d} representing the robot's orientation on the field.
     */
    public void setRobotOrientation(Rotation2d angle) {
        // robot_orientation_set is a double array with 6 entries
        // The only one *actually* used is the first one (yaw)
        double[] entries = new double[6];
        entries[0] = angle.getDegrees();
        m_limelightTable.getEntry("robot_orientation_set").setDoubleArray(entries);
    }

    /**
     * Sets the robot's orientation on the field.
     * Use this for MegaTag2. This must be called at least once per frame
     * @param angle The angle in degrees representing the robot's orientation on the field.
     */
    public void setRobotOrientation(double angle) {
        setRobotOrientation(Rotation2d.fromDegrees(angle));
    }

    public boolean getSide() {
        // True = left, false = right
        // Negate to correct side

        return (-getXOffset() < 0 ? true : false);
    }

    public double getDistance() {
        // Using the height of the target and the height of the camera, we can calculate
        // the distance to the target using the formula:
        // distance = (targetHeight - cameraHeight) / tan(cameraAngle + targetAngle)
        Optional<Pose3d> tagPose = m_layout.getTagPose((int) getFiducialID());
        if (tagPose.isEmpty()) return -1;

        // Z is height, positive is up (thank God)
        double targetHeightMeters = tagPose.get().getZ();
        double targetAngle = getYOffset();

        return (targetHeightMeters - CAMERA_HEIGHT_METERS)
                / Math.tan(Math.toRadians(CAMERA_ANGLE + targetAngle));
    }
}