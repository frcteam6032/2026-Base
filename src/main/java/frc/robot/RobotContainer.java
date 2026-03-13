package frc.robot;

import javax.xml.xpath.XPath;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
        private static final double SHOOTER_COAST = 0.1;
        // Create the robot's subsystems
        private final DriveSubsystem m_robotDrive = new DriveSubsystem();
        private final Limelight m_limelight = new Limelight();
        private final ShooterSubsystem m_shooter = new ShooterSubsystem(m_limelight, m_robotDrive);
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
        private static final double SHOOTER_SPIT_SPEED = 1500;

        private static final Pose2d HUB_TARGET_POSE = new Pose2d(4.01, 2.64, new Rotation2d()); // GOOD
        private static final Pose2d SHUTTLE_POSE_1 = new Pose2d(1.5, 0.75, new Rotation2d()); // FIX
        private static final Pose2d SHUTTLE_POSE_2 = new Pose2d(1.5, -0.75, new Rotation2d()); // FIX
        // private static final double kTransKp = 0.6;

        private double m_targetDistance = 0.0;

        private static final double kRotKp = 0.015;

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
                Command cmd = pointAtHubCommand(getXSpeed(), getYSpeed())
                                .alongWith(m_shooter.automaticHubShooter(m_targetDistance))
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

        private void configureButtonBindings() {
                // DEFAULT COMMANDS //
                m_robotDrive.setDefaultCommand(
                                new RunCommand(
                                                () -> m_robotDrive.joystickDrive(
                                                                getXSpeed() * GameData.shouldInvertControls(),
                                                                getYSpeed() * GameData.shouldInvertControls(),
                                                                getRotationSpeed(),
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
                                pointAtHubCommand(getXSpeed(), getYSpeed())
                                                .alongWith(m_shooter.automaticHubShooter(
                                                                m_targetDistance)));

                // m_operatorController.a().toggleOnTrue(m_shooter.automaticShuttle());

                m_operatorController.b().toggleOnTrue(m_shooter.runRPMCommand(SHOOTER_SPIT_SPEED));

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
                m_limelight.setRobotOrientation(m_robotDrive.getHeading());
        }

        public Translation2d twistToLocation(Pose2d targetPose) {
                // Get the angle and distance to the target pose from the active bot pose
                var robotPose = m_robotDrive.getRobotPoseEstimate();
                var deltaX = targetPose.getX() - robotPose.getX();
                var deltaY = targetPose.getY() - robotPose.getY();
                var distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
                var angleToTarget = Math.toDegrees(Math.atan2(deltaY, deltaX));
                Rotation2d angleDelta = Rotation2d.fromDegrees(angleToTarget - robotPose.getRotation().getDegrees());
                return new Translation2d(distance, angleDelta);
        }

        public Pose2d decideShuttleSpot() {
                Translation2d shuttle1Twist = twistToLocation(SHUTTLE_POSE_1);
                Translation2d shuttle2Twist = twistToLocation(SHUTTLE_POSE_2);

                // Get the distance from the robot pose to each shuttle and pick the closer one
                if (shuttle1Twist.getX() < shuttle2Twist.getX()) {
                        return SHUTTLE_POSE_1;
                } else {
                        return SHUTTLE_POSE_2;
                }

        }

        public Command pointAtHubCommand(double x, double y) {
                return Commands.run(() -> {
                        Translation2d twist = twistToLocation(HUB_TARGET_POSE);
                        m_targetDistance = twist.getX();
                        m_robotDrive.drive(x, y, kRotKp * twist.getAngle().getDegrees(), true);
                }, m_robotDrive);
        }

        public Command pointToBestShuttleCommand(double x, double y) {
                return Commands.run(() -> {
                        Pose2d targetPose = decideShuttleSpot();
                        Translation2d shuttleTwist = twistToLocation(targetPose);
                        m_targetDistance = shuttleTwist.getX();
                        m_robotDrive.drive(x, y, kRotKp * shuttleTwist.getAngle().getDegrees(), true);
                }, m_robotDrive);
        }

}