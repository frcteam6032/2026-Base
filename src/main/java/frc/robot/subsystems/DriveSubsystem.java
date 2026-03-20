// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.utils.DashboardStore;
import frc.robot.vision.Limelight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public class DriveSubsystem extends SubsystemBase {
    public static final double ROTATE_kP = 1.0 / 90.0;
    public static final double ROTATE_kD = 0.000;

    public static final PIDController controller = new PIDController(DriveSubsystem.ROTATE_kP, 0.0,
            DriveSubsystem.ROTATE_kD);

    // Create MAXSwerveModules
    private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
            DriveConstants.kFrontLeftDrivingCanId,
            DriveConstants.kFrontLeftTurningCanId,
            DriveConstants.kFrontLeftChassisAngularOffset);

    private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
            DriveConstants.kFrontRightDrivingCanId,
            DriveConstants.kFrontRightTurningCanId,
            DriveConstants.kFrontRightChassisAngularOffset);

    private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
            DriveConstants.kRearLeftDrivingCanId,
            DriveConstants.kRearLeftTurningCanId,
            DriveConstants.kBackLeftChassisAngularOffset);

    private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
            DriveConstants.kRearRightDrivingCanId,
            DriveConstants.kRearRightTurningCanId,
            DriveConstants.kBackRightChassisAngularOffset);

    // The gyro sensor
    private final Pigeon2 m_gyro;

    // The Limelight vision system
    private final Limelight m_limelight = new Limelight();

    // Odometry class for tracking robot pose
    private final SwerveDrivePoseEstimator m_poseEstimator;

    // Standard deviations of model states and vision measurements (default)
    private static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.05, 0.05, 0.01);
    private static final Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(2.5, 2.5, 2.5);

    // Telemetry
    private final Field2d m_field = new Field2d();

    private Rotation2d m_rotationTarget = new Rotation2d();

    private boolean m_isRotationActive = false;

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {
        m_gyro = new Pigeon2(Constants.DriveConstants.kGyroCanId);
        m_gyro.setYaw(0);

        controller.enableContinuousInput(-180, 180);

        // Initialize the pose estimator
        m_poseEstimator = new SwerveDrivePoseEstimator(
                DriveConstants.kDriveKinematics,
                m_gyro.getRotation2d(),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                },
                new Pose2d(),
                stateStdDevs,
                visionMeasurementStdDevs);

        ModuleConfig moduleConfig = new ModuleConfig(ModuleConstants.kWheelDiameterMeters / 2,
                DriveConstants.kMaxSpeedMetersPerSecond, 1.2, DCMotor.getNEO(1), ModuleConstants.kDrivingMotorReduction,
                1);

        RobotConfig config = new RobotConfig(70, 6.8, moduleConfig, DriveConstants.kDriveKinematics.getModules());

        try {
            config = RobotConfig.fromGUISettings();

            // Configure AutoBuilder last
            AutoBuilder.configure(
                    this::getRobotPoseEstimate, // Robot pose supplier
                    this::setOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                    this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    (speeds, feedforwards) -> driveAuto(speeds, false),
                    // Method that will drive the robot given ROBOT
                    // RELATIVE ChassisSpeeds. Also optionally outputs
                    // individual module feedforwards
                    new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller
                                                    // for
                                                    // holonomic drive trains
                            new PIDConstants(2, 0.0, 0.0), // Translation PID constants
                            // Maybe put back to 2??
                            new PIDConstants(1, 0.0, 0) // Rotation PID constants
                    ),
                    config, // The robot configuration
                    () -> {
                        // Boolean supplier that controls when the path will be mirrored for the red
                        // alliance
                        // This will flip the path being followed to the red side of the field.
                        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    this // Reference to this subsystem to set requirements
            );
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
            System.exit(1);
        }

        setupDashboard();
    }

      public boolean isRotating() {
        return m_isRotationActive;
    }

    private void setupDashboard() {
        DashboardStore.add("Drive/X Velocity", () -> getChassisSpeeds().vxMetersPerSecond);
        DashboardStore.add("Drive/Y Velocity", () -> getChassisSpeeds().vyMetersPerSecond);
        DashboardStore.add("Drive/Angular Velocity", () -> getChassisSpeeds().omegaRadiansPerSecond);

        DashboardStore.add("Drive/X Position", () -> getRobotPoseEstimate().getX());
        DashboardStore.add("Drive/Y Position", () -> getRobotPoseEstimate().getY());

        DashboardStore.add("Drive/Angle/FL", () -> m_frontLeft.getPosition().angle.getDegrees());
        DashboardStore.add("Drive/Angle/FR", () -> m_frontRight.getPosition().angle.getDegrees());
        DashboardStore.add("Drive/Angle/BL", () -> m_rearLeft.getPosition().angle.getDegrees());
        DashboardStore.add("Drive/Angle/BR", () -> m_rearRight.getPosition().angle.getDegrees());

        DashboardStore.add("Drive/Heading", () -> getHeading());

        DashboardStore.add("Drive/Rotating", () -> isRotating());

        DashboardStore.add("Drive/Raw Heading", () -> m_gyro.getRotation2d().getDegrees());
        DashboardStore.addCustom(() -> {
            m_field.setRobotPose(getRobotPoseEstimate());
            SmartDashboard.putData("Field", m_field);
        });
    }

    @Override
    public void periodic() {
        // Update the pose estimator with sensor data
        m_poseEstimator.update(
                m_gyro.getRotation2d(),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                });

        if (m_limelight.targetValid()) {
            Pose2d mt2 = m_limelight.getBotPose();
            double latency = m_limelight.getLatency() / 1000.0;
            double timestamp = Timer.getFPGATimestamp();
            double correctedTimestamp = timestamp - latency;
            m_poseEstimator.addVisionMeasurement(
                    mt2,
                    correctedTimestamp);
        }

    }

    Rotation2d getRotation2D() {
        return getRobotPoseEstimate().getRotation();
    }

    void encoderReset() {
        m_frontLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_rearLeft.resetEncoders();
        m_rearRight.resetEncoders();

    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void setOdometry(Pose2d pose) {
        m_poseEstimator.resetPosition(
                m_gyro.getRotation2d(),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                },
                pose);
    }

    public void zero() {
        setOdometry(new Pose2d());
    }

    /**
     * Method to drive the robot via direct speed input.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,
                                getRobotPoseEstimate().getRotation())
                        : new ChassisSpeeds(xSpeed, ySpeed, rot));

        setModuleStates(swerveModuleStates);
    }

    /**
     * Method to drive the robot via the joystick.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    public Command joystickDriveCommand(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rot,
            BooleanSupplier fieldRelative) {
        return run(() -> {
            joystickDrive(xSpeed.getAsDouble(), ySpeed.getAsDouble(), rot.getAsDouble(),
                    fieldRelative.getAsBoolean());
        });
    }

    /**
     * Method to drive the robot via the joystick.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    public void joystickDrive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
        double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
        double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

        SmartDashboard.putNumber("Drive/Requested X", xSpeedDelivered);
        SmartDashboard.putNumber("Drive/Requested Y", ySpeedDelivered);
        SmartDashboard.putNumber("Drive/Requested Theta", rotDelivered);

        drive(xSpeedDelivered, ySpeedDelivered, rotDelivered, fieldRelative);
    }

    /**
     * Method to drive the robot using chassis speeds.
     *
     * @param speeds        The target {@link ChassisSpeeds}.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    public void driveAuto(ChassisSpeeds speeds, boolean fieldRelative) {
        drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, -speeds.omegaRadiansPerSecond, fieldRelative);
    }

    /**
     * Sets the wheels into an X formation
     */
    private void setX() {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    public Command setXCommand() {
        return run(() -> setX());
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    private void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        m_frontLeft.resetEncoders();
        m_rearLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_rearRight.resetEncoders();
    }

    public double getHeading() {
        // Get the heading and normalize it between 0 and 360 to ensure proper readings
        return getRobotPoseEstimate().getRotation().getDegrees();
    }

    public Pose2d getRobotPoseEstimate() {
        // Returns the estimated robot position (x,y,yaw)
        // WARNING the negating of x & y could cause issues with auto driving
        return new Pose2d(m_poseEstimator.getEstimatedPosition().getX(),
                m_poseEstimator.getEstimatedPosition().getY(),
                m_poseEstimator.getEstimatedPosition().getRotation());
    }

    public Pose2d autoPoseEstimate() {
            // Returns the estimated robot position (x,y,yaw)
        // WARNING the negating of x & y could cause issues with auto driving
        return new Pose2d(m_poseEstimator.getEstimatedPosition().getX(),
                m_poseEstimator.getEstimatedPosition().getY(),
                m_poseEstimator.getEstimatedPosition().getRotation().times(-1));
    }

    public ChassisSpeeds getChassisSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(
                m_frontLeft.getState(),
                m_frontRight.getState(),
                m_rearLeft.getState(),
                m_rearRight.getState());
    }

    private void sideAlignment(DoubleSupplier error) {

        if (m_limelight.targetValid() == false) {
            return;
        }

        if (Math.abs(error.getAsDouble()) > 1.5) {
            joystickDrive(0, -controller.calculate(error.getAsDouble()) / DriveConstants.kMaxSpeedMetersPerSecond, 0,
                    false);
        }
    }

    public Command sideAlignmentCommand(DoubleSupplier error) {
        return run(() -> sideAlignment(error));
    }

    private void distanceCorrection(double currentDistance) {
        if (currentDistance > 1) {
            joystickDrive(0, 0.1, 0, false);
        }
    }

    public Command distanceCorrectionCommand(double currentDistance) {
        return runOnce(() -> distanceCorrection(currentDistance));
    }

    /**
     * Rotate based on a provided offset.
     * 
     * @param xSpeed      The joystick X speed.
     * @param ySpeed      The joystick Y speed.
     * @param offset      Provides the offset.
     * @param validTarget Whether or not the offset is valid.
     * @return A command that rotates based on the offset.
     */
    public Command rotateOffsetCommand(DoubleSupplier xSpeed, DoubleSupplier ySpeed,
            Supplier<Rotation2d> offset, BooleanSupplier validTarget) {

        // This only updates the rotation target IF there's a valid target.
        // rotateToAngleCommand uses a rotation target, NOT an offset.
        // So here we convert the vision offset to a rotational target only if
        // the target is valid.
        return Commands.run(() -> {
            if (validTarget.getAsBoolean()) {
                m_rotationTarget = offset.get().plus(getRotation2D());
            }
        }).alongWith(rotateToAngleCommand(xSpeed, ySpeed, () -> m_rotationTarget))
                .beforeStarting(() -> m_rotationTarget = getRobotPoseEstimate().getRotation());
    }

    /**
     * Rotate via a Vision System.
     * 
     * @param vision What vision system to use.
     * @param xSpeed The joystick X speed.
     * @param ySpeed The joystick Y speed.
     * @return A command that rotates according to the vision system's offset.
     */
    public Command visionRotateCommand(Limelight vision, DoubleSupplier xSpeed, DoubleSupplier ySpeed) {
        // If the offset is less than 10, move to the right by some angle
        // If the offset is greater than 10, move to the left by some angle
        // If the angle is between -10 and 10
        return rotateOffsetCommand(xSpeed, ySpeed, () -> Rotation2d.fromDegrees(-vision.getXOffset()),
                vision::targetValid);
    }

  
    /**
     * Rotates the robot to an angle.
     * 
     * @param xSpeed The joystick X speed.
     * @param ySpeed The joystick Y speed.
     * @param target Provides the target to rotate to.
     * @return A command that rotates the robot to the specified angle.
     */
    public Command rotateToAngleCommand(DoubleSupplier xSpeed, DoubleSupplier ySpeed, Supplier<Rotation2d> target) {
        return run(() -> {
            m_isRotationActive = true;
            Rotation2d targetRotation = target.get();
            double offset = (targetRotation.minus(getRotation2D())).getDegrees();
            // double offset = getRotation2D().minus(targetRotation).getDegrees();

            var output = controller.calculate(offset);
     

            joystickDrive(xSpeed.getAsDouble(), ySpeed.getAsDouble(),
                    output,
                    true);
        }).finallyDo(() -> {
            m_isRotationActive = false;
            controller.reset();
        });
    }
}