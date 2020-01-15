/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SpinnerConstants;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spinner;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SpinnerCountRevs;
import frc.robot.commands.SpinnerStopOnColor;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Chassis chassis = new Chassis();
  private final Spinner spinner = new Spinner();
  private final Shooter shooter = new Shooter();

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  // The driver and operator controllers
  XboxController m_driver = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operator = new XboxController(OIConstants.kOperatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    // A split-stick arcade command, with forward/backward controlled by the left
    // hand, and turning controlled by the right.
    chassis.setDefaultCommand(new RunCommand(
        () -> chassis.drive(m_driver.getY(GenericHID.Hand.kLeft), m_driver.getX(GenericHID.Hand.kRight)), chassis));

    spinner.setDefaultCommand(new RunCommand(() -> spinner.setSetPoint(SpinnerConstants.kStopRPMs), spinner));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Grab the hatch when the 'A' button is pressed.
    new JoystickButton(m_operator, Button.kA.value).whenPressed(new SpinnerCountRevs(spinner));
    new JoystickButton(m_operator, Button.kB.value).whenPressed(new SpinnerStopOnColor(spinner));

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
    return m_autoCommand;
  }
}
