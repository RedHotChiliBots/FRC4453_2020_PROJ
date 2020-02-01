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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spinner;

import frc.robot.commands.ShooterMoveToAngle;
import frc.robot.commands.ShooterShoot;
import frc.robot.commands.ShooterStop;
import frc.robot.commands.SpinnerCountRevs;
import frc.robot.commands.SpinnerStopOnColor;
import frc.robot.commands.SpinnerStow;
import frc.robot.commands.AutonDrive;
import frc.robot.commands.ClimberExtend;
import frc.robot.commands.ClimberRetract;
import frc.robot.commands.CollectorExtend;
import frc.robot.commands.CollectorRetract;
import frc.robot.commands.ShooterAim;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Define SubSystems
  private final Chassis chassis = new Chassis();
  private final Spinner spinner = new Spinner();
  private final Shooter shooter = new Shooter();
  private final Climber climber = new Climber();
  private final Collector collector = new Collector();

  XboxController m_driver = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operator = new XboxController(OIConstants.kOperatorControllerPort);

  // Define Commands
  private final SpinnerStow m_spinnerStow = new SpinnerStow(spinner);
  private final SpinnerStopOnColor m_spinnerStopOnColor = new SpinnerStopOnColor(spinner);
  private final SpinnerCountRevs m_spinnerCountRevs = new SpinnerCountRevs(spinner);

  private final ShooterShoot m_shooterShoot = new ShooterShoot(shooter);
  private final ShooterStop m_shooterStop = new ShooterStop(shooter);
  private final ShooterMoveToAngle m_shooterMoveToAngle = new ShooterMoveToAngle(shooter,
      m_operator.getY(Hand.kLeft) * 35);

  private final AutonDrive m_auton = new AutonDrive(chassis, spinner);

  // Define chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

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

    // Configure default commands
    chassis.setDefaultCommand(
        new RunCommand(() -> chassis.driveTeleop(m_driver.getY(Hand.kLeft), m_driver.getY(Hand.kRight)), chassis));

    spinner.setDefaultCommand(new RunCommand(() -> spinner.setRPMs(Constants.SpinnerConstants.kStopRPMs), spinner));

    shooter.setDefaultCommand(new RunCommand(() -> shooter.stopShoot(), shooter));

    // shooter.setDefaultCommand(new ShooterStop(shooter));

    // A chooser for autonomous commands
    m_chooser.addOption("Auton", m_auton);

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
    new JoystickButton(m_operator, Button.kA.value).whenPressed(new SpinnerCountRevs(spinner));
    new JoystickButton(m_operator, Button.kB.value).whenPressed(new SpinnerStopOnColor(spinner));
    new JoystickButton(m_operator, Button.kX.value).whenPressed(new SpinnerStow(spinner));

    new JoystickButton(m_operator, Button.kY.value).whenHeld(new ShooterShoot(shooter));

    new JoystickButton(m_operator, Button.kStart.value).whenPressed(new SpinnerStow(spinner));

    for (int i = 0; i < 8; i++) {
      new POVButton(m_operator, i * 45).whenHeld(new ShooterAim(shooter, i));
    }
    // new POVButton(m_operator, 0).whenHeld(new ShooterAim(shooter, 0));
    // new POVButton(m_operator, 1).whenHeld(new AimShooter(shooter, 1));
    // new POVButton(m_operator, 2).whenHeld(new AimShooter(shooter, 2));
    // new POVButton(m_operator, 3).whenHeld(new AimShooter(shooter, 3));
    // new POVButton(m_operator, 4).whenHeld(new AimShooter(shooter, 4));
    // new POVButton(m_operator, 5).whenHeld(new AimShooter(shooter, 5));
    // new POVButton(m_operator, 6).whenHeld(new AimShooter(shooter, 6));
    // new POVButton(m_operator, 7).whenHeld(new AimShooter(shooter, 7));

    new JoystickButton(m_operator, Button.kBumperRight.value).whenPressed(new ClimberExtend(climber));
    new JoystickButton(m_operator, Button.kBumperLeft.value).whenPressed(new ClimberRetract(climber));

    new JoystickButton(m_driver, Button.kBumperRight.value).whenPressed(new CollectorExtend(collector));
    new JoystickButton(m_driver, Button.kBumperLeft.value).whenPressed(new CollectorRetract(collector));

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
    return m_chooser.getSelected();
  }
}
