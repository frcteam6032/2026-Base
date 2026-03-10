package frc.robot;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.system.Discretization;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
    private static final double SHOOTER_COAST = 0.1;
    // Create the robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final Limelight m_limelight = new Limelight();
    private final ShooterSubsystem m_shooter = new ShooterSubsystem(m_limelight, m_robotDrive);
    private final InfeedSubsystem m_infeed = new InfeedSubsystem();
    private final FeederSubsystem m_feeder = new FeederSubsystem();
    private final InfeedArmSubsystem m_infeedArm = new InfeedArmSubsystem();
    private final SpindexerSubsystem m_spindexer = new SpindexerSubsystem();
    private DriveType m_driveType = DriveType.Normal;

    // Create the driver controller
    private final CommandXboxController m_driverController = new CommandXboxController(
            OIConstants.DRIVER_CONTROLLER);
    // Create operator controller
    private final CommandXboxController m_operatorController = new CommandXboxController(
            OIConstants.OPERATOR_CONTROLLER);

    // COMMAND CONSTANTS //
    private static final double INFEED_SPEED = 0.7;
    private static final double FEEDER_SPEED = 0.9;
    private static final double SPINDEXER_SPEED = 0.5;
    private static final double SHOOTER_SPIT_SPEED = 3200;

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

    }

    private void initAutoChooser() {
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    enum DriveType {
        Normal(0),
        Bump(1),
        Alignment(2);

        public int Type;

        private DriveType(int value) {
            Type = value;
        }
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
        m_driverController.start().onTrue(Commands.run(() -> m_robotDrive.zero()));

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

        // TODO: X should be a command that:
        // - Spins up the shooter, and rotates based on limelight
        // - Reset the shooter based on the new distance
        m_operatorController.x().toggleOnTrue(
                m_shooter.automaticShooter()
                        .alongWith(m_robotDrive.visionRotateCommand(m_limelight,
                                () -> -m_driverController.getLeftX(),
                                () -> -m_driverController.getRightX())));

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

}