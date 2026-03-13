package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.Timer;
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
        private enum PredictionTarget {
                HUB,
                SHUTTLE
        }

        private static final double SHOOTER_COAST = 0.1;
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
        private static final double INFEED_SPEED = 0.75;
        private static final double FEEDER_SPEED = 0.9;
        private static final double SPINDEXER_SPEED = 0.9;
        private static final double SHOOTER_SPIT_SPEED = 1500; // 1500;

        // Update these and make sure to account for red or blue alliance4
        // From orientation of the blue alliance
        // private static final Pose2d HUB_TARGET_POSE = new Pose2d(4.620, 4.030, new
        // Rotation2d()); // Good
        // private static final Pose2d SHUTTLE_TARGET_POSE1 = new Pose2d(1.490, 1.053,
        // new Rotation2d()); // Good
        // private static final Pose2d SHUTTLE_TARGET_POSE2 = new Pose2d(2.0, 7.0, new
        // Rotation2d());

        // Prediction state
        private Pose2d m_lastPose = null;
        private double m_lastTime = -1.0;
        private double m_vx = 0.0;
        private double m_vy = 0.0;
        private static final double kVelSmoothingAlpha = 0.2;

        private Pose2d m_predictedPose = new Pose2d();
        private Pair<Double, Rotation2d> m_currentHubAlignment = new Pair<Double, Rotation2d>(0.0, new Rotation2d());
        private Pair<Double, Rotation2d> m_currentShuttleAlignment = new Pair<Double, Rotation2d>(0.0,
                        new Rotation2d());
        private boolean m_hasPrediction = false;
        private PredictionTarget m_lastPredictionTarget = null;
        private double m_lastPredictionUpdateTime = -1.0;

        private static final double kLookAheadBase = 0.1; // seconds
        private static final double kLookAheadMax = 0.3; // seconds
        private static final double kPredictionUpdatePeriod = 0.05; // seconds
        private static final double kLimelightOrientationFeedPeriod = 0.05; // seconds
        // private static final double kTransKp = 0.6;
        private static final double kRotKp = 0.015;
        private double m_lastLimelightFeedTime = -1.0;

        // AUTO/DRIVER STUFF //
        private SendableChooser<Command> autoChooser;

        private final SlewRateLimiter xLimiter = new SlewRateLimiter(8.);
        private final SlewRateLimiter yLimiter = new SlewRateLimiter(8.);
        private final SlewRateLimiter thetaLimiter = new SlewRateLimiter(6.);

        private double getRotationSpeed() {
                return MathUtil.applyDeadband(
                                MathUtils.scaleDriverController(m_driverController.getRightX(), thetaLimiter,
                                                m_driverController.getRightTriggerAxis()),
                                OIConstants.DRIVE_DEADBAND);
        }

        private double getYSpeed() {
                return MathUtil.applyDeadband(MathUtils.scaleDriverController(-m_driverController.getLeftX(), yLimiter,
                                m_driverController.getRightTriggerAxis()), OIConstants.DRIVE_DEADBAND);
        }

        private double getXSpeed() {
                return MathUtil.applyDeadband(MathUtils.scaleDriverController(-m_driverController.getLeftY(), xLimiter,
                                m_driverController.getRightTriggerAxis()), OIConstants.DRIVE_DEADBAND);
        }

        public Command alignAndShootCommand() {
                Command cmd = driveToPredictedHubPointCommand(getXSpeed(), getYSpeed())
                                .alongWith(m_shooter.automaticHubShooter(m_currentHubAlignment.getFirst()))
                                .alongWith(m_feeder.intakeCommand(FEEDER_SPEED))
                                .alongWith(m_spindexer.spinCommand(SPINDEXER_SPEED));
                return cmd;
        }

        public Command autoIntakeCommand() {
                return m_infeed.intakeCommand(INFEED_SPEED);
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

        public Pair<Double, Rotation2d> getDistAndAnglePoint(double x, double y) {
                // Use odometry to calculate vector to a point from the robot pose
                var robotPose = m_robotDrive.getRobotPoseEstimate();
                var robotX = robotPose.getX();
                var robotY = robotPose.getY();
                var robotAngle = robotPose.getRotation().getDegrees();

                var deltaX = x - robotX;
                var deltaY = y - robotY;

                var distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
                var angleToPoint = Math.toDegrees(Math.atan2(deltaY, deltaX));

                var angleDelta = angleToPoint - robotAngle;

                return new Pair<Double, Rotation2d>(distance, Rotation2d.fromDegrees(angleDelta));

        }

        private void configureButtonBindings() {
                // DEFAULT COMMANDS //
                m_robotDrive.setDefaultCommand(
                                new RunCommand(
                                                () -> m_robotDrive.joystickDrive(
                                                                getXSpeed() * GameData.shouldInvertControls(),
                                                                getYSpeed() * GameData.shouldInvertControls(),
                                                                -getRotationSpeed(),
                                                                true),
                                                m_robotDrive));

                // shooter should always be running due to epic inertia
                m_shooter.setDefaultCommand(m_shooter.runPercentCommand(() -> SHOOTER_COAST));
                m_infeed.setDefaultCommand(m_infeed.stopCommand());
                m_feeder.setDefaultCommand(m_feeder.stopCommand());
                m_spindexer.setDefaultCommand(m_spindexer.stopCommand());

                // ====== //
                // DRIVER //
                // ====== //
                m_driverController.start().onTrue(Commands.runOnce(() -> m_robotDrive.zero()));

                // INFEED //
                m_driverController.leftTrigger().whileTrue(m_infeed.intakeCommand(INFEED_SPEED));
                m_driverController.rightBumper().onTrue(m_infeedArm.switchPositionCommand());

                // TESTING, PLEASE REMOVE //
                // m_driverController.x().whileTrue(m_shooter.runTargetCommand());
                m_driverController.y().onTrue(m_robotDrive.readyBump()).onFalse(m_robotDrive.disableOverBump());

                // ======== //
                // OPERATOR //
                // ======== //

                // SHOOTER //

                m_operatorController.x().whileTrue(
                                driveToPredictedHubPointCommand(getXSpeed(), getYSpeed())
                                                .alongWith(m_shooter.automaticHubShooter(
                                                                m_currentHubAlignment.getFirst())));

                m_operatorController.rightBumper()
                                .whileTrue(pointToSelectedShuttlePosition(getXSpeed(), getYSpeed())
                                                .alongWith(m_shooter.automaticShuttle(
                                                                m_currentShuttleAlignment.getFirst())));

                // m_operatorController.a().toggleOnTrue(m_shooter.automaticShuttle());

                m_operatorController.b().toggleOnTrue(m_shooter.runRPMCommand(SHOOTER_SPIT_SPEED));
                // m_operatorController.b().toggleOnTrue(m_shooter.runTargetCommmandRPM());

                // INFEED ARM, MANUAL OVERRIDES //
                m_operatorController.leftTrigger().onTrue(m_infeedArm.bumpCommand(-2.5));
                m_operatorController.rightTrigger().onTrue(m_infeedArm.bumpCommand(2.5));

                // FEED TO SHOOTER //
                m_operatorController.y()
                                .whileTrue(m_feeder.intakeCommand(FEEDER_SPEED)
                                                .alongWith(m_spindexer.spinCommand(SPINDEXER_SPEED)));

                m_operatorController.a().whileTrue(m_feeder.intakeCommand(FEEDER_SPEED));
                // OUTTAKE //
                m_operatorController.leftBumper().whileTrue(
                                m_infeed.intakeCommand(-INFEED_SPEED)
                                                .alongWith(m_feeder.intakeCommand(-FEEDER_SPEED))
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
                double now = Timer.getFPGATimestamp();
                if (m_lastLimelightFeedTime > 0.0 && now - m_lastLimelightFeedTime < kLimelightOrientationFeedPeriod) {
                        return;
                }
                m_limelight.setRobotOrientation(m_robotDrive.getHeading());
                m_lastLimelightFeedTime = now;
        }

        private double getDynamicLookAhead() {
                double speed = Math.hypot(m_vx, m_vy);
                double lookAhead = kLookAheadBase + speed * 0.01;
                return Math.min(lookAhead, kLookAheadMax);
        }

        public Pose2d convertToRedAlliance(Pose2d pose) {
                double fieldWidth = 16.54;
                return new Pose2d(fieldWidth - pose.getX(), pose.getY(), pose.getRotation());
        }

        private void updatePredictionState() {
                Pose2d currentPose = m_robotDrive.getRobotPoseEstimate();
                double now = Timer.getFPGATimestamp();

                if (m_lastPose != null && (now - m_lastTime) > 1e-6) {
                        double dt = now - m_lastTime;
                        double rawVx = (currentPose.getX() - m_lastPose.getX()) / dt;
                        double rawVy = (currentPose.getY() - m_lastPose.getY()) / dt;

                        m_vx = kVelSmoothingAlpha * rawVx + (1.0 - kVelSmoothingAlpha) * m_vx;
                        m_vy = kVelSmoothingAlpha * rawVy + (1.0 - kVelSmoothingAlpha) * m_vy;
                }

                m_lastPose = currentPose;
                m_lastTime = now;

                double lookAhead = getDynamicLookAhead();

                m_predictedPose = new Pose2d(
                                currentPose.getX() + m_vx * lookAhead,
                                currentPose.getY() + m_vy * lookAhead,
                                currentPose.getRotation());
        }

        public void updateShotPrediction() {
                updatePredictionState();
                Pose2d currentPose = m_robotDrive.getRobotPoseEstimate();

                Pose2d HubPose = new Pose2d(4.620, 4.030, new Rotation2d()); // Good

                if (GameData.getIsRed()) {
                        HubPose = convertToRedAlliance(HubPose);
                }

                double dx = HubPose.getX() - m_predictedPose.getX();
                double dy = HubPose.getY() - m_predictedPose.getY();

                double distance = Math.hypot(dx, dy);
                double angleToPoint = Math.toDegrees(Math.atan2(dy, dx));
                double angleDelta = angleToPoint - currentPose.getRotation().getDegrees();

                m_currentHubAlignment = new Pair<Double, Rotation2d>(distance, Rotation2d.fromDegrees(angleDelta));
                m_hasPrediction = true;
                m_lastPredictionTarget = PredictionTarget.HUB;
                m_lastPredictionUpdateTime = Timer.getFPGATimestamp();
        }

        private void refreshPredictionIfNeeded(PredictionTarget target) {
                double now = Timer.getFPGATimestamp();
                boolean targetChanged = m_lastPredictionTarget != target;
                boolean predictionStale = !m_hasPrediction
                                || m_lastPredictionUpdateTime < 0.0
                                || (now - m_lastPredictionUpdateTime) >= kPredictionUpdatePeriod;

                if (!targetChanged && !predictionStale) {
                        return;
                }

                if (target == PredictionTarget.HUB) {
                        updateShotPrediction();
                } else {
                        updateShuttlePrediction();
                }
        }

        public Command driveToPredictedHubPointCommand(double xSpeed, double ySpeed) {
                return Commands.run(() -> {
                        refreshPredictionIfNeeded(PredictionTarget.HUB);

                        double rotCmd = MathUtil.clamp(m_currentHubAlignment.getSecond().getDegrees() * kRotKp, -1.0,
                                        1.0);

                        m_robotDrive.joystickDrive(xSpeed, ySpeed, rotCmd, true);
                }, m_robotDrive);
        }

        public void updateShuttlePrediction() {
                updatePredictionState();
                Pose2d currentPose = m_robotDrive.getRobotPoseEstimate();

                Pose2d ShuttleTarget1 = new Pose2d(1.490, 1.053, new Rotation2d()); // Good
                Pose2d ShuttleTarget2 = new Pose2d(2.0, 7.0, new Rotation2d()); // Good

                if (GameData.getIsRed()) {
                        ShuttleTarget1 = convertToRedAlliance(ShuttleTarget1);
                        ShuttleTarget2 = convertToRedAlliance(ShuttleTarget2);
                }

                // Determine which shuttle target is closer
                double dx1 = ShuttleTarget1.getX() - m_predictedPose.getX();
                double dy1 = ShuttleTarget1.getY() - m_predictedPose.getY();
                double dist1 = Math.hypot(dx1, dy1);

                double dx2 = ShuttleTarget2.getX() - m_predictedPose.getX();
                double dy2 = ShuttleTarget2.getY() - m_predictedPose.getY();
                double dist2 = Math.hypot(dx2, dy2);

                if (dist2 < dist1) {
                        dx1 = dx2;
                        dy1 = dy2;
                }

                double angleToPoint = Math.toDegrees(Math.atan2(dy1, dx1));
                double angleDelta = angleToPoint - currentPose.getRotation().getDegrees();

                m_currentShuttleAlignment = new Pair<Double, Rotation2d>(Math.hypot(dx1, dy1),
                                Rotation2d.fromDegrees(angleDelta));
                m_hasPrediction = true;
                m_lastPredictionTarget = PredictionTarget.SHUTTLE;
                m_lastPredictionUpdateTime = Timer.getFPGATimestamp();
        }

        public Command pointToSelectedShuttlePosition(double xSpeed, double ySpeed) {
                return Commands.run(() -> {
                        refreshPredictionIfNeeded(PredictionTarget.SHUTTLE);

                        double rotCmd = MathUtil.clamp(m_currentShuttleAlignment.getSecond().getDegrees() * kRotKp,
                                        -1.0,
                                        1.0);

                        m_robotDrive.joystickDrive(xSpeed, ySpeed, rotCmd, true);
                }, m_robotDrive);
        }

}