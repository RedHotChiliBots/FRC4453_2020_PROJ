/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;

import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.AnalogIOConstants;
import frc.robot.Constants.CANidConstants;
import frc.robot.Constants.ChassisConstants;

/**
 * Represents a differential drive style drivetrain.
 */
public class Chassis extends SubsystemBase {

	// Define the left side motors, master and follower
	private final CANSparkMax leftMaster = new CANSparkMax(CANidConstants.kLeftMasterMotor, MotorType.kBrushless);
	private final SpeedController m_leftMaster = leftMaster;
	private final SpeedController m_leftFollower = new CANSparkMax(CANidConstants.kLeftFollowerMotor,
			MotorType.kBrushless);

	// Define the right side motors, master and follower
	private final CANSparkMax rightMaster = new CANSparkMax(CANidConstants.kRightMasterMotor, MotorType.kBrushless);
	private final SpeedController m_rightMaster = rightMaster;
	private final SpeedController m_rightFollower = new CANSparkMax(CANidConstants.kRightFollowerMotor,
			MotorType.kBrushless);

	// Group the left and right motors
	private final SpeedControllerGroup m_leftGroup = new SpeedControllerGroup(m_leftMaster, m_leftFollower);
	private final SpeedControllerGroup m_rightGroup = new SpeedControllerGroup(m_rightMaster, m_rightFollower);

	private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftGroup, m_rightGroup);

	// Identify left and rigt encoders
	public final CANEncoder m_leftEncoder = new CANEncoder(leftMaster);
	public final CANEncoder m_rightEncoder = new CANEncoder(rightMaster);

	// Identify left and right PID controllers
	// private final CANPIDController m_leftPIDController = new
	// CANPIDController(leftMaster);
	// private final CANPIDController m_rightPIDController = new
	// CANPIDController(rightMaster);
	private final PIDController m_leftPIDController = new PIDController(ChassisConstants.kP, ChassisConstants.kI,
			ChassisConstants.kD);
	private final PIDController m_rightPIDController = new PIDController(ChassisConstants.kP, ChassisConstants.kI,
			ChassisConstants.kD);

	private final PIDController m_distPIDController = new PIDController(ChassisConstants.kDistP,
			ChassisConstants.kDistI, ChassisConstants.kDistD);

	private final PIDController m_rotPIDController = new PIDController(ChassisConstants.kRotP, ChassisConstants.kRotI,
			ChassisConstants.kRotD);

	private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(
			ChassisConstants.kTrackWidth);

	private final DifferentialDriveOdometry m_odometry;

	private final Trajectory trajectory = generateTrajectory();

	private final RamseteController controller = new RamseteController(2.0, 0.7);

	// Initialize NavX AHRS board
	// Alternatively: I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB
	private final AHRS ahrs = new AHRS(SPI.Port.kMXP);

	// Identify PDP and PCM
	private final PowerDistributionPanel pdp = new PowerDistributionPanel(CANidConstants.kPDP);
	private final Compressor compressor = new Compressor(CANidConstants.kCompressor);

	// Identify compressor hi and lo sensors
	private AnalogInput hiPressureSensor = new AnalogInput(AnalogIOConstants.kHiPressureChannel);
	private AnalogInput loPressureSensor = new AnalogInput(AnalogIOConstants.kLoPressureChannel);

	private final ShuffleboardTab chassisTab = Shuffleboard.getTab("Chassis");
	private NetworkTableEntry sbRobotAngle = chassisTab.addPersistent("Robot Angle", 0).getEntry();
	private NetworkTableEntry sbLeftPos = chassisTab.addPersistent("ML Position", 0).getEntry();
	private NetworkTableEntry sbLeftVel = chassisTab.addPersistent("ML Velocity", 0).getEntry();
	private NetworkTableEntry sbRightPos = chassisTab.addPersistent("MR Position", 0).getEntry();
	private NetworkTableEntry sbRightVel = chassisTab.addPersistent("MR Velocity", 0).getEntry();
	private NetworkTableEntry sbLeftPow = chassisTab.addPersistent("ML Power", 0).getEntry();
	private NetworkTableEntry sbRightPow = chassisTab.addPersistent("MR Power", 0).getEntry();

	private final ShuffleboardTab pneumaticsTab = Shuffleboard.getTab("Pneumatics");
	private NetworkTableEntry sbHiPressure = pneumaticsTab.addPersistent("Hi Pressure", 0).getEntry();
	private NetworkTableEntry sbLoPressure = pneumaticsTab.addPersistent("Lo Pressure", 0).getEntry();

	private final ShuffleboardTab visionTab = Shuffleboard.getTab("Vision");
	private NetworkTableEntry sbLimeLSet = visionTab.addPersistent("Lime LSet", 0).getEntry();
	private NetworkTableEntry sbLimeRSet = visionTab.addPersistent("Lime RSet", 0).getEntry();

	/**
	 * Constructs a differential drive object. Sets the encoder distance per pulse
	 * and resets the gyro.
	 */
	public Chassis() {
		super();
		System.out.println("+++++ Chassis Constructor starting ...");

		pdp.clearStickyFaults();
		compressor.clearAllPCMStickyFaults();

		ahrs.reset();
		ahrs.zeroYaw();

		// m_leftPIDController.reset();
		// m_rightPIDController.reset();

		// m_leftGroup.setInverted(true);
		// m_rightGroup.setInverted(true);

		// m_leftPIDController.setP(ChassisConstants.kP);
		// m_leftPIDController.setI(ChassisConstants.kI);
		// m_leftPIDController.setD(ChassisConstants.kD);
		// m_leftPIDController.setIZone(ChassisConstants.kIz);
		// m_leftPIDController.setFF(ChassisConstants.kFF);
		// m_leftPIDController.setOutputRange(ChassisConstants.kMinSpeedMPS,
		// ChassisConstants.kMaxSpeedMPS);

		// m_rightPIDController.setP(ChassisConstants.kP);
		// m_rightPIDController.setI(ChassisConstants.kI);
		// m_rightPIDController.setD(ChassisConstants.kD);
		// m_rightPIDController.setIZone(ChassisConstants.kIz);
		// m_rightPIDController.setFF(ChassisConstants.kFF);
		// m_rightPIDController.setOutputRange(ChassisConstants.kMinSpeedMPS,
		// ChassisConstants.kMaxSpeedMPS);

		// m_leftEncoder.setInverted(true);
		// m_rightEncoder.setInverted(true);

		// Set the distance per pulse for the drive encoders. We can simply use the
		// distance traveled for one rotation of the wheel divided by the encoder
		// resolution.
		m_leftEncoder.setPositionConversionFactor(ChassisConstants.kPosFactor);
		m_rightEncoder.setPositionConversionFactor(ChassisConstants.kPosFactor);

		m_leftEncoder.setVelocityConversionFactor(ChassisConstants.kVelFactor);
		m_rightEncoder.setVelocityConversionFactor(ChassisConstants.kVelFactor);

		chassisTab.addPersistent("ML Pos Factor", m_leftEncoder.getPositionConversionFactor());
		chassisTab.addPersistent("MR Pos Factor", m_rightEncoder.getPositionConversionFactor());
		chassisTab.addPersistent("ML Vel Factor", m_leftEncoder.getVelocityConversionFactor());
		chassisTab.addPersistent("MR Vel Factor", m_rightEncoder.getVelocityConversionFactor());

		// Reset the current encoder positions to zero
		m_leftEncoder.setPosition(0.0);
		m_rightEncoder.setPosition(0.0);

		m_odometry = new DifferentialDriveOdometry(getAngle());

		resetFieldPosition(0.0, 0.0);
	}

	public DifferentialDriveOdometry getOdometry() {
		return m_odometry;
	}

	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
	}

	public DifferentialDriveKinematics getKinematics() {
		return m_kinematics;
	}

	public PIDController getLeftPID() {
		return m_leftPIDController;
	}

	public PIDController getRightPID() {
		return m_rightPIDController;
	}

	public PIDController getDistPID() {
		return m_distPIDController;
	}

	public PIDController getRotPID() {
		return m_rotPIDController;
	}

	public DifferentialDriveWheelSpeeds getWheelSpeeds() {
		return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
	}

	private Trajectory generateTrajectory() {
		var sideStart = new Pose2d(Units.feetToMeters(1.54), Units.feetToMeters(23.23), Rotation2d.fromDegrees(-180));

		var crossScale = new Pose2d(Units.feetToMeters(23.7), Units.feetToMeters(6.8), Rotation2d.fromDegrees(-160));

		var interiorWaypoints = new ArrayList<Translation2d>();
		interiorWaypoints.add(new Translation2d(Units.feetToMeters(14.54), Units.feetToMeters(23.23)));
		interiorWaypoints.add(new Translation2d(Units.feetToMeters(21.04), Units.feetToMeters(18.23)));

		TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(12), Units.feetToMeters(12));
		config.setReversed(true);

		return TrajectoryGenerator.generateTrajectory(sideStart, interiorWaypoints, crossScale, config);
	}

	public Trajectory getTrajectory() {
		return trajectory;
	}

	public RamseteController getController() {
		return controller;
	}

	public double getDuration(Trajectory t) {
		return t.getTotalTimeSeconds();
	}

	public void periodic() {
		sbRobotAngle.setDouble(-ahrs.getAngle());
		sbLeftPos.setDouble(m_leftEncoder.getPosition());
		sbLeftVel.setDouble(m_leftEncoder.getVelocity());
		sbRightPos.setDouble(m_rightEncoder.getPosition());
		sbRightVel.setDouble(m_rightEncoder.getVelocity());
		sbLeftPow.setDouble(m_leftGroup.get());
		sbRightPow.setDouble(m_rightGroup.get());

		sbHiPressure.setDouble(getHiPressure());
		sbLoPressure.setDouble(getLoPressure());

		// Update field position - for autonomous
		updateOdometry();
		// m_odometry.getPoseMeters();
		// new Pose2d(x, y, rotation);
		// m_odometry.resetPosition(poseMeters, gyroAngle);
		// m_odometry.update(gyroAngle, leftDistanceMeters, rightDistanceMeters);

		// m_kinematics.trackWidthMeters;
		// m_kinematics.toChassisSpeeds(wheelSpeeds);
		// m_kinematics.toWheelSpeeds(chassisSpeeds);
	}

	public void driveTankVolts(double left, double right) {
		m_diffDrive.tankDrive(-left, -right);
	}

	public void driveTank(double left, double right) {
		m_diffDrive.tankDrive(-left, -right);
	}

	public void driveArcade(double spd, double rot) {
		m_diffDrive.arcadeDrive(-spd, rot);
	}

	public void resetFieldPosition(double x, double y) {
		ahrs.zeroYaw();
		m_leftEncoder.setPosition(0.0);
		m_rightEncoder.setPosition(0.0);
		m_odometry.resetPosition(new Pose2d(x, y, getAngle()), getAngle());
	}

	/**
	 * Returns the angle of the robot as a Rotation2d.
	 *
	 * @return The angle of the robot.
	 */
	public Rotation2d getAngle() {
		// Negating the angle because WPILib gyros are CW positive.
		return Rotation2d.fromDegrees(-ahrs.getYaw());
	}

	/**
	 * Sets the desired wheel speeds.
	 *
	 * @param speeds The desired wheel speeds.
	 */
	public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
		double leftFeedforward = 0.0;// m_Feedforward.calculate(speeds.leftMetersPerSecond);
		double rightFeedforward = 0.0;// m_Feedforward.calculate(speeds.rightMetersPerSecond);

		sbLimeLSet.setDouble(speeds.leftMetersPerSecond);
		sbLimeRSet.setDouble(-speeds.rightMetersPerSecond);

		double leftOutput = m_leftPIDController.calculate(m_leftEncoder.getVelocity(), speeds.leftMetersPerSecond);
		double rightOutput = m_rightPIDController.calculate(m_rightEncoder.getVelocity(), -speeds.rightMetersPerSecond);

		m_leftGroup.set(leftOutput + leftFeedforward);
		m_rightGroup.set(rightOutput + rightFeedforward);
	}

	/**
	 * Drives the robot with the given linear velocity and angular velocity.
	 *
	 * @param xSpeed Linear velocity in m/s.
	 * @param rot    Angular velocity in rad/s.
	 */
	// @SuppressWarnings("ParameterName")
	public void drive(double xSpeed, double xRot) {
		var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, -xRot));
		setSpeeds(wheelSpeeds);
	}

	/**
	 * Updates the field-relative position.
	 */
	public void updateOdometry() {
		m_odometry.update(getAngle(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
	}

	public void driveTrajectory(double left, double right) {
		m_leftGroup.set(left);
		m_rightGroup.set(right);
	}

	// public void driveDistanceWithHeading(double distance, double angle) {
	// m_leftPIDController.setReference(distance, ControlType.kPosition);
	// setSetpoint(angle);
	// getPIDController().enable();
	// distancePID.enable();
	// }

	// public void driveDistance(double distance) {
	// m_leftPIDController.setReference(distance, ControlType.kPosition);

	// }

	// public void setLMTgtPosition(double pos) {
	// tgtPosition = pos;
	// pid.setReference(pos * REV_PER_INCH_MOTOR, ControlType.kSmartMotion,
	// RobotMap.kSlot_Position);
	// pid.setReference(pos, ControlType.kSmartMotion, RobotMap.kSlot_Position);
	// // m_leftPIDController.setRefer
	// }

	/**
	 * Get Compressor info
	 */
	// public boolean isCompEnabled() {
	// return compressor.enabled();
	// }

	// public boolean isCompSwitch() {
	// return compressor.getPressureSwitchValue();

	// }

	// public double getCompCurrent() {
	// return compressor.getCompressorCurrent();
	// }

	/**
	 * Get Hi and Lo pressure sensors in PSI
	 */
	public double getLoPressure() {
		return 250.0 * (loPressureSensor.getVoltage() / AnalogIOConstants.kInputVoltage) - 25.0;
	}

	public double getHiPressure() {
		return 250.0 * (hiPressureSensor.getVoltage() / AnalogIOConstants.kInputVoltage) - 25.0;
	}
}