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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.infeed.InfeedSubsystem;
import frc.robot.subsystems.infeedArm.InfeedArmSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import frc.robot.utils.DashboardStore;
import frc.robot.utils.GameData;
import frc.robot.utils.MathUtils;
import frc.robot.vision.Limelight;

public class RobotContainer {
    public enum FireType {
        Manual(0),
        Assisted(1),
        Automatic(2);

        public int Type;

        private FireType(int type) {
            Type = type;
        }
    }

    private final SendableChooser<FireType> fireTypeChooser = new SendableChooser<>();

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
    private static final double SPINDEXER_SPEED = 1.0;

    // POSE CONSTANTS //
    private double m_targetDistance = 0.0;

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
                () -> true).beforeStarting(() -> MathUtils.BASE_SPEED = 0.4);
    }

    public RobotContainer() {
        fireTypeChooser.setDefaultOption("Manual", FireType.Manual);
        fireTypeChooser.addOption("Assisted", FireType.Assisted);
        fireTypeChooser.addOption("Automatic", FireType.Automatic);
        SmartDashboard.putData("Fire Type", fireTypeChooser);

        SmartDashboard.putNumber("Auto Delay", 0.0);

        DashboardStore.add("Limelight/Distance", m_limelight::getDistanceToPoint);

        DashboardStore.add("Probability/Shot", () -> m_shooter.getShotProbability() * 100.0);

        // Configure the buttons & default commands
        configureButtonBindings();

        // Config buttons
        initAutoChooser();
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
        // ====== //`
        m_driverController.start().onTrue(Commands.runOnce(() -> m_robotDrive.zero()));

        // INFEED CONTROL //
        m_driverController.leftBumper().whileTrue(m_infeed.intakeRPMCommand(INFEED_SPEED / 2));
        m_driverController.rightBumper().whileTrue(m_infeed.intakeCommand(-INFEED_SPEED / 2));
        m_driverController.y().onTrue(m_infeedArm.switchPositionCommand());

        // SHOOTER //
        m_driverController.rightTrigger()
                .whileTrue(pointAtHubCommand(m_limelight, this::getXSpeed, this::getYSpeed)
                        .alongWith(m_spindexer.spinCommand(SPINDEXER_SPEED))
                        .alongWith(m_feeder.intakeCommand(FEEDER_SPEED))
                        .alongWith(m_shooter.automaticHubShooter(() -> m_targetDistance, getSelectedFireType())));

        // INFEED ARM, MANUAL OVERRIDES //
        // m_driverController.start().whileTrue(m_infeedArm.agitateCommand());

        // FEED TO SHOOTER //
        // Put this back
        // m_driverController.a().whileTrue(feedCommand());

        // OUTTAKE //
        // m_driverController.leftBumper().whileTrue(backspinCommand());
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

    public Command pointAtHubCommand(Limelight limelight, DoubleSupplier x, DoubleSupplier y) {
        m_targetDistance = limelight.getDistanceToPoint();
        return m_robotDrive.visionRotateCommand(limelight, x, y);
    }

    private FireType getSelectedFireType() {
        FireType selected = fireTypeChooser.getSelected();
        return selected != null ? selected : FireType.Manual;
    }

    // Delete this after testing
    public double getTargetDistance() {
        m_targetDistance = m_limelight.getDistanceToPoint();
        return m_targetDistance;
    }
}