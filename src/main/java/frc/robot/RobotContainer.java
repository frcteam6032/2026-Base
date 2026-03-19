package frc.robot;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.infeed.InfeedSubsystem;
import frc.robot.subsystems.infeedArm.InfeedArmSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import frc.robot.utils.GameData;
import frc.robot.utils.MathUtils;
import frc.robot.vision.Limelight;

public class RobotContainer {
    // Create the robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final Limelight m_limelight = new Limelight();
    private final ShooterSubsystem m_shooter = new ShooterSubsystem(m_limelight);
    private final InfeedSubsystem m_infeed = new InfeedSubsystem();
    private final FeederSubsystem m_feeder = new FeederSubsystem();
    private final InfeedArmSubsystem m_infeedArm = new InfeedArmSubsystem();
    private final SpindexerSubsystem m_spindexer = new SpindexerSubsystem();

    // Create the driver controller
    private final CommandXboxController m_driverController = new CommandXboxController(
            OIConstants.DRIVER_CONTROLLER);
    // Create operator controller
    private final CommandXboxController m_operatorController = new CommandXboxController(
            OIConstants.OPERATOR_CONTROLLER);

    // COMMAND CONSTANTS //
    private static final double INFEED_SPEED = 4700.0; // 0.75;
    private static final double FEEDER_SPEED = 1.0;
    private static final double SPINDEXER_SPEED = 0.9;
    private static final double SHOOTER_SPIT_SPEED = 3000;
    private static final double VACUUM_TRANSLATION_DEADBAND = 0.05;

    // POSE CONSTANTS //
    private static final Pose2d HUB_TARGET_POSE = new Pose2d(4.57, 4.04, new Rotation2d());
    private static final Pose2d SHUTTLE_POSE_1 = new Pose2d(1.5, 1.0, new Rotation2d());
    private static final Pose2d SHUTTLE_POSE_2 = new Pose2d(1.5, 7.0, new Rotation2d());

    private double m_targetDistance = 0.0;

    // AUTO/DRIVER STUFF //
    private SendableChooser<Command> autoChooser;

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(8.);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(8.);
    private final SlewRateLimiter thetaLimiter = new SlewRateLimiter(6.);

    private Rotation2d m_vacuumHeadingTarget = new Rotation2d();
    private Rotation2d m_overBumpHeadingTarget = new Rotation2d();

    private double getRotationSpeed() {
        return MathUtil.applyDeadband(
                MathUtils.scaleDriverController(m_driverController.getRightX(), thetaLimiter,
                        m_driverController.getRightTriggerAxis()),
                OIConstants.DRIVE_DEADBAND);
    }

    private double getYSpeed() {
        return -MathUtil.applyDeadband(MathUtils.scaleDriverController(-m_driverController.getLeftX(), yLimiter,
                m_driverController.getRightTriggerAxis()), OIConstants.DRIVE_DEADBAND)
                * GameData.shouldInvertControls();
    }

    private double getXSpeed() {
        return -MathUtil.applyDeadband(MathUtils.scaleDriverController(-m_driverController.getLeftY(), xLimiter,
                m_driverController.getRightTriggerAxis()), OIConstants.DRIVE_DEADBAND)
                * GameData.shouldInvertControls();
    }

    private Command createNormalDriveCommand() {
        return m_robotDrive.joystickDriveCommand(
                () -> getXSpeed(),
                () -> getYSpeed(),
                () -> getRotationSpeed(),
                () -> true);
    }

    private Command createVacuumDriveCommand() {
        return m_robotDrive.rotateToAngleCommand(
                () -> getXSpeed(),
                () -> getYSpeed(),
                () -> getVacuumHeadingTarget())
                .beforeStarting(() -> m_vacuumHeadingTarget = m_robotDrive.getRobotPoseEstimate().getRotation());
    }

    private Command createOverBumpDriveCommand() {
        return m_robotDrive.rotateToAngleCommand(
                () -> getXSpeed(),
                () -> getYSpeed(),
                () -> m_overBumpHeadingTarget)
                .beforeStarting(() -> updateOverBumpTarget());
    }

    private Rotation2d getVacuumHeadingTarget() {
        double x = getXSpeed();
        double y = getYSpeed();
        double magnitude = Math.hypot(x, y);

        if (magnitude > VACUUM_TRANSLATION_DEADBAND) {
            m_vacuumHeadingTarget = new Rotation2d(x, y);
        }

        return m_vacuumHeadingTarget;
    }

    private void updateOverBumpTarget() {
        double yaw = m_robotDrive.getHeading();
        yaw = ((yaw % 360.0) + 360.0) % 360.0;

        double deltaTo0 = Math.min(yaw, 360.0 - yaw);
        double deltaTo180 = Math.min(Math.abs(yaw - 180.0), 360.0 - Math.abs(yaw - 180.0));

        double lockedHeading = (deltaTo0 <= deltaTo180) ? 0.0 : 180.0;
        m_overBumpHeadingTarget = Rotation2d.fromDegrees(lockedHeading);
    }

    public Command alignAndShootCommand() {
        Command cmd = pointAtHubCommand(() -> 0, () -> 0)
                .alongWith(m_feeder.intakeCommand(FEEDER_SPEED))
                .alongWith(m_spindexer.spinCommand(SPINDEXER_SPEED))
                .alongWith(m_shooter.automaticHubShooter(() -> m_targetDistance));
        return cmd;
    }

    public Command autoIntakeCommand() {
        Command deployAndIntake = m_infeedArm.switchPositionCommand()
                .andThen(m_infeed.intakeRPMCommand(INFEED_SPEED));
        return deployAndIntake;
    }

    public RobotContainer() {
        SmartDashboard.putNumber("Auto Delay", 0.0);
        configureNamedCommands();

        // Configure the buttons & default commands
        configureButtonBindings();

        // Config buttons
        initAutoChooser();
    }

    // TODO
    private void configureNamedCommands() {
        NamedCommands.registerCommand("Align, Feed, Spindexer, Shoot", alignAndShootCommand());
        NamedCommands.registerCommand("Intake", autoIntakeCommand());
    }

    private void initAutoChooser() {
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureButtonBindings() {
        // DEFAULT COMMANDS handled via drive mode management
        m_robotDrive.setDefaultCommand(createNormalDriveCommand());

        // shooter should always be running due to inertia
        m_shooter.setDefaultCommand(m_shooter.coastCommand());
        m_infeed.setDefaultCommand(m_infeed.stopCommand());
        m_feeder.setDefaultCommand(m_feeder.stopCommand());
        m_spindexer.setDefaultCommand(m_spindexer.stopCommand());

        // ====== //
        // DRIVER //
        // ====== //
        m_driverController.start().onTrue(Commands.runOnce(() -> m_robotDrive.zero()));

        // INFEED CONTROL //
        m_driverController.leftTrigger().whileTrue(m_infeed.intakeRPMCommand(INFEED_SPEED));
        m_driverController.leftBumper().whileTrue(m_infeed.intakeCommand(-INFEED_SPEED));
        m_driverController.rightBumper().onTrue(m_infeedArm.switchPositionCommand());
      //  m_driverController.x().whileTrue(m_infeedArm.oscillateArmCommand());

        m_driverController.a().toggleOnTrue(createVacuumDriveCommand());
        m_driverController.y().toggleOnTrue(createOverBumpDriveCommand());

        // ======== //
        // OPERATOR //
        // ======== //

        // SHOOTER //

        m_operatorController.x().toggleOnTrue(pointAtHubCommand(this::getXSpeed, this::getYSpeed)
                .alongWith(m_shooter.automaticHubShooter(() -> m_targetDistance)));

        // Shuttle (move to A/B probably?)
        m_operatorController.rightBumper()
                .toggleOnTrue(pointToBestShuttleCommand(() -> getXSpeed(), () -> getYSpeed())
                        .alongWith(m_shooter.automaticShuttle(() -> m_targetDistance)));

        // a or b should be autoshuttle
        m_operatorController.a().toggleOnTrue(m_shooter.runRPMCommand(6000));

        // m_operatorController.b().toggleOnTrue(m_shooter.runRPMCommand(SHOOTER_SPIT_SPEED));
        m_operatorController.b().toggleOnTrue(m_shooter.runTargetCommmandRPM());

        // INFEED ARM, MANUAL OVERRIDES //
        m_operatorController.leftTrigger().onTrue(m_infeedArm.bumpCommand(-2.5));
        m_operatorController.rightTrigger().onTrue(m_infeedArm.bumpCommand(2.5));

        // FEED TO SHOOTER //
        m_operatorController.y().whileTrue(m_feeder.intakeCommand(FEEDER_SPEED)
                .alongWith(m_spindexer.spinCommand(SPINDEXER_SPEED)));

        // OUTTAKE //
        m_operatorController.leftBumper().whileTrue(m_feeder.intakeCommand(-FEEDER_SPEED)
                .alongWith(m_spindexer.spinCommand(-SPINDEXER_SPEED)));
    }

    // Get the selected auto command
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public double getDelay() {
        return SmartDashboard.getNumber("Auto Delay", 0);
    }

    /**
     * Feed the current angle of the drivetrain into the Limelight.
     */
    public void feedLimelight() {
        m_limelight.setRobotOrientation(m_robotDrive.getHeading());
    }

    public Pose2d convertToRed(Pose2d pose) {
        double fieldLength = 16.54; // meters
        return new Pose2d(fieldLength - pose.getX(), pose.getY(),
                new Rotation2d(Math.PI).minus(pose.getRotation()));
    }

    public Pose2d twistToLocation(Pose2d targetPose) {
        if (GameData.getIsRed()) {
            targetPose = convertToRed(targetPose);
        }

        Pose2d robotPose = m_robotDrive.getRobotPoseEstimate();

        ChassisSpeeds speeds = m_robotDrive.getChassisSpeeds();
        Pose2d poseForCalculation = robotPose;

        // Delete this block to not use prediction, won't break anything
        if (speeds != null) {
            double lookAheadSeconds = MathUtils.getTwistLookAheadSeconds(speeds);
            poseForCalculation = MathUtils.predictPose(robotPose, speeds, lookAheadSeconds);
        }

        double deltaX = targetPose.getX() - poseForCalculation.getX();
        double deltaY = targetPose.getY() - poseForCalculation.getY();
        Rotation2d targetAngle = new Rotation2d(Math.atan2(deltaY, deltaX));

        // add 180 deg, because we always want the shooter pointing towards the twist
        // point.
        return new Pose2d(deltaX, deltaY, targetAngle.plus(new Rotation2d(Math.PI)));
    }

    public Pose2d closestShuttleTwist() {
        Pose2d shuttle1Twist = twistToLocation(SHUTTLE_POSE_1);
        Pose2d shuttle2Twist = twistToLocation(SHUTTLE_POSE_2);

        // Get the distance from the robot pose to each shuttle and pick the closer one
        if (shuttle1Twist.getTranslation().getNorm() < shuttle2Twist.getTranslation().getNorm()) {
            return shuttle1Twist;
        } else {
            return shuttle2Twist;
        }
    }

    /**
     * x and y are joystick commands
     */
    private Command suppliedPointCommand(DoubleSupplier x, DoubleSupplier y, Supplier<Pose2d> twistSupplier) {
        Supplier<Rotation2d> targetSupplier = () -> {
            var twist = twistSupplier.get();

            m_targetDistance = twist.getTranslation().getNorm();

            SmartDashboard.putNumber("Target Angle", twist.getRotation().getDegrees());
            SmartDashboard.putNumber("Target Distance", Units.metersToFeet(m_targetDistance));
            return twist.getRotation();
        };

        DoubleSupplier xSupplier = () -> x.getAsDouble();
        DoubleSupplier ySupplier = () -> y.getAsDouble();

        return m_robotDrive.rotateToAngleCommand(xSupplier, ySupplier, targetSupplier);
    }

    public Command pointAtHubCommand(DoubleSupplier x, DoubleSupplier y) {
        return suppliedPointCommand(x, y, () -> twistToLocation(HUB_TARGET_POSE));
    }

    public Command pointToBestShuttleCommand(DoubleSupplier x, DoubleSupplier y) {
        return suppliedPointCommand(x, y, this::closestShuttleTwist);
    }
}