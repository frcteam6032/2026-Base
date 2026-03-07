package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.InfeedArm.InfeedArmSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.infeed.InfeedSubsystem;
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

    private SendableChooser<Command> autoChooser;

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(8.);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(8.);
    private final SlewRateLimiter thetaLimiter = new SlewRateLimiter(6.);

    private double getRotationSpeed() {
        return MathUtil.applyDeadband(MathUtils.scaleDriverController(m_driverController.getRightX(), thetaLimiter,
                m_driverController.getRightTriggerAxis()), OIConstants.DRIVE_DEADBAND);
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

    /**
     * DONT CHANGE THESE
     */
    private void configureButtonBindings() {
        // Default drive command
        m_robotDrive.setDefaultCommand(
                new RunCommand(
                        () -> m_robotDrive.joystickDrive(
                                getXSpeed() * GameData.shouldInvertControls(),
                                getYSpeed() * GameData.shouldInvertControls(),
                                getRotationSpeed(),
                                true),
                        m_robotDrive));

        m_driverController.start().onTrue(Commands.run(() -> m_robotDrive.zero()));

        m_infeed.setDefaultCommand(m_infeed.stopCommand());
        m_driverController.leftTrigger().whileTrue(m_infeed.intakeCommand(0.5));
        m_driverController.leftBumper().whileTrue(m_infeed.intakeCommand(-0.8));

        m_feeder.setDefaultCommand(m_feeder.stopCommand());
        m_driverController.y().whileTrue(m_feeder.intakeCommand(0.8));
        m_driverController.rightBumper().whileTrue(m_feeder.intakeCommand(-0.8));

        m_shooter.setDefaultCommand(m_shooter.stopCommand());
        // Change to automatic shooter command for testing
        m_driverController.x().whileTrue(m_shooter.runTargetCommand());

        // TODO set up the spindexer ands infeed arm commands for the driver/operator
        m_driverController.leftTrigger().whileTrue(m_infeed.intakeCommand(0.3));
        m_driverController.rightBumper().onTrue(m_infeedArm.switchPosition());

        m_operatorController.x().onTrue(m_shooter.automaticShooter());

        m_operatorController.leftTrigger().whileTrue(m_infeedArm.goToDeploy());
        m_operatorController.rightTrigger().whileTrue(m_infeedArm.goToStow());

        Command parallelGroup = Commands.parallel(
                m_infeed.intakeCommand(0.3),
                m_spindexer.spinCommand(0.3));

        m_driverController.y()
                .onTrue(parallelGroup);

        m_operatorController.leftBumper().onTrue(m_infeed.intakeCommand(-0.3));

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