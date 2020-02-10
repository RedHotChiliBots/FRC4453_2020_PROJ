/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spinner;

import frc.robot.commands.ShooterShoot;
import frc.robot.commands.ShooterStop;
import frc.robot.commands.SpinnerCountRevs;
import frc.robot.commands.SpinnerStop;
import frc.robot.commands.SpinnerStopOnColor;
import frc.robot.commands.AutonDrive2Point;
import frc.robot.commands.AutonDriveJoystick;
import frc.robot.commands.ClimberExtend;
import frc.robot.commands.ClimberRetract;
import frc.robot.commands.CollectorExtend;
import frc.robot.commands.CollectorRetract;
import frc.robot.commands.CollectorStop;
import frc.robot.commands.DriveTank;
import frc.robot.commands.DriveArcade;
import frc.robot.commands.HopperStop;
import frc.robot.commands.ShooterAim;
import frc.robot.commands.ShooterInit;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Define SubSystems
    public final Chassis chassis = new Chassis();
    public final Spinner spinner = new Spinner();
    public final Shooter shooter = new Shooter();
    public final Climber climber = new Climber();
    public final Collector collector = new Collector();
    public final Hopper hopper = new Hopper();

    XboxController m_driver = new XboxController(OIConstants.kDriverControllerPort);
    XboxController m_operator = new XboxController(OIConstants.kOperatorControllerPort);

    // Define Commands
    private final AutonDriveJoystick autonDriveJoystick = new AutonDriveJoystick(() -> -m_driver.getY(Hand.kLeft),
            () -> m_driver.getX(Hand.kRight), chassis);
    private final AutonDrive2Point autonDrive2Point = new AutonDrive2Point(
            new Pose2d(new Translation2d(7.0, 7.0), new Rotation2d(0.0)), chassis);

    // private final SpinnerStop m_spinnerStop = new SpinnerStop(spinner);
    // private final SpinnerStopOnColor m_spinnerStopOnColor = new
    // SpinnerStopOnColor(spinner);
    // private final SpinnerCountRevs m_spinnerCountRevs = new
    // SpinnerCountRevs(spinner);

    // private final ShooterShoot m_shooterShoot = new ShooterShoot(shooter);
    // private final ShooterStop m_shooterStop = new ShooterStop(shooter);
    // private final ShooterMoveToAngle m_shooterMoveToAngle = new
    // ShooterMoveToAngle(shooter,
    // m_operator.getY(Hand.kLeft) * 35);

    // private final AutonDrive m_auton = new AutonDrive(chassis, spinner);

    // Define chooser for autonomous commands
    SendableChooser<Command> m_chooser = new SendableChooser<>();

    ShuffleboardTab chassisTab = Shuffleboard.getTab("Chassis");
    ShuffleboardTab pneumaticsTab = Shuffleboard.getTab("Pneumatics");
    ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");
    ShuffleboardTab climberTab = Shuffleboard.getTab("Climber");
    ShuffleboardTab spinnerTab = Shuffleboard.getTab("Spinner");
    ShuffleboardTab collectorTab = Shuffleboard.getTab("Collector");
    ShuffleboardTab hopperTab = Shuffleboard.getTab("Hopper");
    ShuffleboardTab visionTab = Shuffleboard.getTab("Vision");

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        // Add subsystems to dashboard
        SmartDashboard.putData("Chassis", chassis);
        SmartDashboard.putData("Shooter", shooter);
        SmartDashboard.putData("Climber", climber);
        SmartDashboard.putData("Spinner", spinner);
        SmartDashboard.putData("Collector", collector);
        SmartDashboard.putData("Hopper", hopper);

        // Configure default commands
        // chassis.setDefaultCommand(
        // new RunCommand(() -> chassis.driveTeleop(m_driver.getY(Hand.kLeft),
        // m_driver.getY(Hand.kRight)), chassis));

        chassis.setDefaultCommand(
                new DriveTank(() -> m_driver.getY(Hand.kLeft), () -> m_driver.getY(Hand.kRight), chassis));
        collector.setDefaultCommand(new CollectorStop(collector));
        hopper.setDefaultCommand(new HopperStop(hopper));
        shooter.setDefaultCommand(new ShooterStop(shooter));
        spinner.setDefaultCommand(new SpinnerStop(spinner));

        // A chooser for autonomous commands
        m_chooser.addOption("Auton Joystick", autonDriveJoystick);
        m_chooser.addOption("Auton Drive2Point", autonDrive2Point);

        SmartDashboard.putData("Auton Chooser", m_chooser);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        // Define Operator controls
        new JoystickButton(m_driver, Button.kA.value)
                .whenPressed(new DriveTank(() -> m_driver.getY(Hand.kLeft), () -> m_driver.getY(Hand.kRight), chassis));

        new JoystickButton(m_driver, Button.kB.value).whenPressed(
                new DriveArcade(() -> m_driver.getY(Hand.kLeft), () -> m_driver.getX(Hand.kRight), chassis));

        new JoystickButton(m_driver, Button.kX.value).whenPressed(autonDriveJoystick);

        new JoystickButton(m_driver, Button.kY.value).whenPressed(autonDrive2Point);

        // new JoystickButton(m_driver, Button.kY.value)
        // .whenPressed(new AutoShooterAim(shooter, () -> shooter.getX(), () ->
        // shooter.getY()));

        // new JoystickButton(m_driver, Button.kX.value).whenPressed(new
        // ShooterAimStop(shooter));

        new JoystickButton(m_driver, Button.kBumperRight.value).whenPressed(new CollectorExtend(collector));
        new JoystickButton(m_driver, Button.kBumperLeft.value).whenPressed(new CollectorRetract(collector));

        new JoystickButton(m_operator, Button.kA.value).whenPressed(new SpinnerCountRevs(spinner));
        new JoystickButton(m_operator, Button.kB.value).whenPressed(new SpinnerStopOnColor(spinner));
        new JoystickButton(m_operator, Button.kX.value).whenPressed(new SpinnerStop(spinner));

        new JoystickButton(m_operator, Button.kY.value).whenHeld(new ShooterShoot(shooter));

        new JoystickButton(m_operator, Button.kStart.value).whenPressed(new ShooterStop(shooter));

        for (int i = 0; i < 8; i++) {
            new POVButton(m_operator, i * 45).whenHeld(new ShooterAim(shooter, i));
        }

        new JoystickButton(m_operator, Button.kBumperRight.value).whenPressed(new ClimberExtend(climber));
        new JoystickButton(m_operator, Button.kBumperLeft.value).whenPressed(new ClimberRetract(climber));

        // new JoystickButton(m_driver, Button.kA.value)
        // .whenPressed(new InstantCommand(m_hatchSubsystem::grabHatch,
        // m_hatchSubsystem));
        // Release the hatch when the 'B' button is pressed.
        // new JoystickButton(m_driver, Button.kB.value)
        // .whenPressed(new InstantCommand(m_hatchSubsystem::releaseHatch,
        // m_hatchSubsystem));
        // While holding the shoulder button, drive at half speed
        // new JoystickButton(m_driver, Button.kBumperRight.value)
        // .whenPressed(() -> m_robotDrive.setMaxOutput(0.5))
        // .whenReleased(() -> m_robotDrive.setMaxOutput(1));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new SequentialCommandGroup(new ParallelCommandGroup(new ShooterInit(shooter)), m_chooser.getSelected());
    }
}
