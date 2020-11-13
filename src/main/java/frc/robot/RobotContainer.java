/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AUTONRENDEZVOUS;
import frc.robot.commands.AUTONRENDEZVOUSTRENCH;
import frc.robot.commands.AUTONTRENCH;
import frc.robot.commands.AUTONTRENCHRENDEZVOUS;
import frc.robot.commands.AutoTurretAim;
import frc.robot.commands.AutonDrive2Point;
import frc.robot.commands.AutonDriveJoystick;
import frc.robot.commands.AutonDriveTrajectory;
import frc.robot.commands.ClimberClimb;
import frc.robot.commands.ClimberLevel;
import frc.robot.commands.ClimberStop;
import frc.robot.commands.CollectorArmInit;
import frc.robot.commands.CollectorExtend;
import frc.robot.commands.CollectorRetract;
import frc.robot.commands.CollectorStop;
import frc.robot.commands.CollectorIntake;
import frc.robot.commands.DriveArcade;
import frc.robot.commands.DriveTank;
import frc.robot.commands.HopperShoot;
import frc.robot.commands.HopperStop;
import frc.robot.commands.TurretAim;
import frc.robot.commands.TurretAimJoystick;
import frc.robot.commands.TurretAimStop;
import frc.robot.commands.TurretAngleDeg;
import frc.robot.commands.TurretAngleInit;
import frc.robot.commands.ShooterInit;
import frc.robot.commands.ShooterStop;
import frc.robot.commands.TurretTiltInit;
import frc.robot.commands.SpinnerCountRevs;
import frc.robot.commands.SpinnerStop;
import frc.robot.commands.SpinnerStopOnColor;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spinner;
import frc.robot.subsystems.Turret;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

	// =============================================================
	// Initialize SubSystems
	public final Chassis chassis = new Chassis();
	public final Turret turret = new Turret();
	public final Spinner spinner = new Spinner();
	public final Shooter shooter = new Shooter();
	public final Climber climber = new Climber();
	public final Collector collector = new Collector();
	public final Hopper hopper = new Hopper();


	// =============================================================
	// Define Joysticks
	XboxController m_driver = new XboxController(OIConstants.kDriverControllerPort);
	XboxController m_operator = new XboxController(OIConstants.kOperatorControllerPort);

	private static final double DEADZONE = 0.2;

	private boolean force = false;

	// =============================================================
	// Define Commands here to avoid multiple instantiations
	// If commands use Shuffleboard and are instantiated multiple time, an error
	// is thrown on the second instantiation becuase the "title" already exists.
	private final AutonDriveJoystick autonDriveJoystick = new AutonDriveJoystick(() -> -m_driver.getY(Hand.kLeft),
			() -> m_driver.getX(Hand.kRight), chassis);

	private final AutonDrive2Point autonDrive2Point = new AutonDrive2Point(
			new Pose2d(new Translation2d(7.0, 7.0), new Rotation2d(0.0)), chassis);

	private final AutonDriveTrajectory autonDriveTrajectory = new AutonDriveTrajectory(
			chassis.lineToRendezvousTrajectory, chassis);

	private final AUTONRENDEZVOUSTRENCH AUTONRENDEZVOUSTRENCH = new AUTONRENDEZVOUSTRENCH(shooter, collector, hopper,
			chassis);
	private final AUTONRENDEZVOUS AUTONRENDEZVOUS = new AUTONRENDEZVOUS(shooter, collector, hopper, chassis);
	private final AUTONTRENCHRENDEZVOUS AUTONTRENCHRENDEZVOUS = new AUTONTRENCHRENDEZVOUS(shooter, collector, hopper,
			chassis);
	private final AUTONTRENCH AUTONTRENCH = new AUTONTRENCH(shooter, collector, hopper, chassis);
	// Define chooser for autonomous commands
	SendableChooser<Command> m_chooser = new SendableChooser<>();

	// =============================================================
	// Create tabs on shuffleboard for each subsystem
	ShuffleboardTab chassisTab = Shuffleboard.getTab("Chassis");
	ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");
	ShuffleboardTab climberTab = Shuffleboard.getTab("Climber");
	ShuffleboardTab spinnerTab = Shuffleboard.getTab("Spinner");
	ShuffleboardTab collectorTab = Shuffleboard.getTab("Collector");
	ShuffleboardTab hopperTab = Shuffleboard.getTab("Hopper");
	ShuffleboardTab turretTab = Shuffleboard.getTab("Turret");

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {

		// Configure the button bindings
		configureButtonBindings();

		// =============================================================
		// Add subsystems to dashboard
		SmartDashboard.putData("Chassis", chassis);
		SmartDashboard.putData("Turret", turret);
		SmartDashboard.putData("Shooter", shooter);
		SmartDashboard.putData("Climber", climber);
		SmartDashboard.putData("Spinner", spinner);
		SmartDashboard.putData("Collector", collector);
		SmartDashboard.putData("Hopper", hopper);

		// =============================================================
		// Configure default commands for each subsystem
		chassis.setDefaultCommand(
				new DriveTank(() -> m_driver.getY(Hand.kLeft), () -> m_driver.getY(Hand.kRight), chassis));
		climber.setDefaultCommand(new ClimberStop(climber));
		collector.setDefaultCommand(new CollectorStop(collector));
		hopper.setDefaultCommand(new HopperStop(hopper));
		shooter.setDefaultCommand(new ShooterStop(shooter));
//		turret.setDefaultCommand(new AutoTurretAim(() -> turret.getTiltCmd(), () -> turret.getYawCmd(), turret));
		turret.setDefaultCommand(new TurretAimJoystick(() -> getOperatorLY(), () -> getOperatorLX(), turret));
		spinner.setDefaultCommand(new SpinnerStop(spinner));
//		vision.setDefaultCommand(new VisionSeek(vision));

		// =============================================================
		// Build chooser for autonomous commands
		m_chooser.addOption("Auton Joystick", autonDriveJoystick);
		m_chooser.addOption("Auton Drive to Point", autonDrive2Point);
		m_chooser.addOption("Auton DriveTrajectory", autonDriveTrajectory);
		m_chooser.addOption("Auton Rendezvous", AUTONRENDEZVOUS);
		m_chooser.addOption("Auton Rendezvous to Trench", AUTONRENDEZVOUSTRENCH);
		m_chooser.addOption("Auton Trench", AUTONTRENCH);
		m_chooser.addOption("Auton Trench to Rendezvous", AUTONTRENCHRENDEZVOUS);

		SmartDashboard.putData("Auton Chooser", m_chooser);
		System.out.println("Options added to AutonChooser");
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by instantiating a {@link GenericHID} or one of its subclasses
	 * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
	 * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {

		// =============================================================
		// Define Operator controls, buttons, bumpers, etc.
		new JoystickButton(m_driver, Button.kA.value)
				.whenPressed(new DriveTank(() -> getDriverLY(), () -> getDriverRY(), chassis));
				
		new JoystickButton(m_driver, Button.kB.value)
				.whenPressed(new DriveArcade(() -> getDriverLY(), () -> getDriverLX(), chassis));

		// new JoystickButton(m_driver,
		// Button.kX.value).whenPressed(autonDriveJoystick);

		// new JoystickButton(m_driver, Button.kY.value)
		// .whenPressed(autonDriveTrajectory.andThen(() -> //chassis.driveTank(0, 0)));

		// new JoystickButton(m_driver, Button.kY.value)
		// .whenPressed(new AutoShooterAim(shooter, () -> shooter.getX(), () ->
		// shooter.getY()));

		// new JoystickButton(m_driver, Button.kX.value).whenPressed(new
		// ShooterAimStop(shooter));

		new JoystickButton(m_operator, Button.kX.value).whenPressed(new CollectorArmInit(collector));
		new JoystickButton(m_driver, Button.kBumperRight.value).whenPressed(new CollectorExtend(collector));
		new JoystickButton(m_driver, Button.kBumperLeft.value).whenPressed(new CollectorRetract(collector));

		new JoystickButton(m_operator, Button.kA.value).whenPressed(new SpinnerCountRevs(spinner));
		new JoystickButton(m_operator, Button.kB.value).whenPressed(new SpinnerStopOnColor(spinner));

		for (int i = 0; i < 8; i++) {
			new POVButton(m_operator, i * 45).whenHeld(new TurretAim(turret, i));
		}

		// new JoystickButton(m_operator, Button.kBumperRight.value).whenPressed(new
		// ClimberExtend(climber));
		// new JoystickButton(m_operator, Button.kBumperLeft.value).whenPressed(new
		// ClimberRetract(climber));
		new JoystickButton(m_operator, Button.kBack.value).whenHeld(new ClimberClimb(climber));
		new JoystickButton(m_operator, Button.kStickRight.value)
				.whenPressed(new ClimberLevel(climber, () -> getOperatorRY()));

		// new JoystickButton(m_driver, Button.kBack.value).whenPressed(new
		// AutonDrive(chassis, 20.0));

		// new JoystickButton(m_operator, Button.kBack.value).and(new
		// JoystickButton(m_operator, Button.kStart.value).negate())
		// .whileActiveOnce(new HopperLoad(hopper, false));

		// new JoystickButton(m_operator, Button.kBack.value).and(new
		// JoystickButton(m_operator, Button.kStart.value))
		// .whileActiveOnce(new HopperLoad(hopper, true));

		// new JoystickButton(m_operator, Button.kBack.value).whileActiveOnce(new HopperLoad(hopper, true));

		// new JoystickButton(m_operator, Button.kB.value).whenPressed(new
		// HopperShoot(hopper, shooter));

		new JoystickButton(m_operator, Button.kY.value).whenPressed(new CollectorIntake(collector));

		new JoystickButton(m_driver, Button.kStart.value).whenPressed(new HopperShoot(hopper, shooter));

		new JoystickButton(m_driver, Button.kBack.value).whenPressed(new
		CollectorStop(collector));

		// new JoystickButton(m_driver, Button.kX.value).whenPressed(new
		// CollectorReject(collector));

		new JoystickButton(m_driver, Button.kY.value)
				.whenPressed(new AutoTurretAim(() -> turret.getTiltCmd(), () -> turret.getYawCmd(), turret));

		new JoystickButton(m_driver, Button.kX.value).whenPressed(new TurretAimStop(turret));

		new JoystickButton(m_operator, Button.kBumperRight.value).whenPressed(new TurretAngleInit(turret));
		new JoystickButton(m_operator, Button.kBumperLeft.value).whenPressed(new TurretTiltInit(turret));

		// new JoystickButton(m_operator, Button.kY.value).whenPressed(new ShooterShoot(shooter));

		// new JoystickButton(m_operator, Button.kStart.value).whenPressed(new
		// ShooterStop(shooter));
		// new JoystickButton(m_operator, Button.kStart.value).whenPressed(new
		// ShooterTiltDeg(shooter, 135));
		new JoystickButton(m_operator, Button.kStart.value).whenPressed(new TurretAngleDeg(turret, 10));
		System.out.println("Buttons configured");
	}

	public void setDriverRumble(GenericHID.RumbleType t) {
		m_driver.setRumble(t, 1);
	}

	public void resetDriverRumble(GenericHID.RumbleType t) {
		m_driver.setRumble(t, 0);
	}

	public void setOperatorRumble(GenericHID.RumbleType t) {
		m_operator.setRumble(t, 1);
	}

	public void resetOperatorRumble(GenericHID.RumbleType t) {
		m_operator.setRumble(t, 0);
	}

	public double getDriverLX() {
		double v = m_driver.getX(Hand.kLeft);
		return Math.abs(v) < DEADZONE ? 0.0 : v;
	}

	public double getDriverLY() {
		double v = m_driver.getY(Hand.kLeft);
		return Math.abs(v) < DEADZONE ? 0.0 : v;
	}

	public double getDriverRX() {
		double v = m_driver.getX(Hand.kRight);
		return Math.abs(v) < DEADZONE ? 0.0 : v;
	}

	public double getDriverRY() {
		double v = m_driver.getY(Hand.kRight);
		return Math.abs(v) < DEADZONE ? 0.0 : v;
	}

	public double getOperatorLX() {
		double v = m_operator.getX(Hand.kLeft);
		return Math.abs(v) < DEADZONE ? 0.0 : v;
	}

	public double getOperatorLY() {
		double v = m_operator.getY(Hand.kLeft);
		return Math.abs(v) < DEADZONE ? 0.0 : v;
	}

	public double getOperatorRX() {
		double v = m_operator.getX(Hand.kRight);
		return Math.abs(v) < DEADZONE ? 0.0 : v;
	}

	public double getOperatorRY() {
		double v = m_operator.getY(Hand.kRight);
		return Math.abs(v) < DEADZONE ? 0.0 : v;
	}

	public boolean setForce(boolean force) {
		return this.force = force;
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
