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
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.ChassisConstants;
import frc.robot.Constants.UnitsConstants;

/**
 * Represents a differential drive style drivetrain.
 */
public class Chassis extends SubsystemBase {

	private final CANSparkMax leftMaster = new CANSparkMax(ChassisConstants.kLeftMasterMotor, MotorType.kBrushless);
	private final SpeedController m_leftMaster = leftMaster;
	private final SpeedController m_leftFollower = new CANSparkMax(ChassisConstants.kLeftFollowerMotor,
			MotorType.kBrushless);

	private final CANSparkMax rightMaster = new CANSparkMax(ChassisConstants.kRightMasterMotor, MotorType.kBrushless);
	private final SpeedController m_rightMaster = rightMaster;
	private final SpeedController m_rightFollower = new CANSparkMax(ChassisConstants.kRightFollowerMotor,
			MotorType.kBrushless);

	private final SpeedControllerGroup m_leftGroup = new SpeedControllerGroup(m_leftMaster, m_leftFollower);
	private final SpeedControllerGroup m_rightGroup = new SpeedControllerGroup(m_rightMaster, m_rightFollower);

	private final DifferentialDrive m_tankDrive = new DifferentialDrive(m_leftGroup, m_rightGroup);

	public final CANEncoder m_leftEncoder;
	// = new CANEncoder(leftMaster);
	public final CANEncoder m_rightEncoder;
	// = new CANEncoder(rightMaster);

	private final CANPIDController m_leftPIDController;
	// = new CANPIDController(leftMaster);
	private final CANPIDController m_rightPIDController;
	// = new CANPIDController(rightMaster);

	/* Alternatively: I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB */
	private final AHRS ahrs = new AHRS(SPI.Port.kMXP);

	private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(
			ChassisConstants.kTrackWidth);

	private final DifferentialDriveOdometry m_odometry;

	// private final Compressor compressor = new Compressor(0);
	// (Constants.ChassisConstants.kCompressorChannel);
	private AnalogInput hiPressureSensor = null;
	// = new AnalogInput(Constants.ChassisConstants.kHiPressureChannel);
	private AnalogInput loPressureSensor = null;
	// new AnalogInput(Constants.ChassisConstants.kLoPressureChannel);

	/**
	 * Constructs a differential drive object. Sets the encoder distance per pulse
	 * and resets the gyro.
	 */
	public Chassis() {
		super();
		System.out.println("+++++ Chassis Constructor starting ...");

		ahrs.reset();
		ahrs.zeroYaw();

		// m_leftPIDController.reset();
		// m_rightPIDController.reset();

		m_leftGroup.setInverted(true);
		m_rightGroup.setInverted(true);

		m_leftPIDController = leftMaster.getPIDController();
		m_rightPIDController = rightMaster.getPIDController();

		m_leftEncoder = leftMaster.getEncoder();
		m_rightEncoder = rightMaster.getEncoder();

		m_leftPIDController.setP(ChassisConstants.kP);
		m_leftPIDController.setI(ChassisConstants.kI);
		m_leftPIDController.setD(ChassisConstants.kD);
		m_leftPIDController.setIZone(ChassisConstants.kIz);
		m_leftPIDController.setFF(ChassisConstants.kFF);
		m_leftPIDController.setOutputRange(ChassisConstants.kMinSpeedMPS, ChassisConstants.kMaxSpeedMPS);

		m_rightPIDController.setP(ChassisConstants.kP);
		m_rightPIDController.setI(ChassisConstants.kI);
		m_rightPIDController.setD(ChassisConstants.kD);
		m_rightPIDController.setIZone(ChassisConstants.kIz);
		m_rightPIDController.setFF(ChassisConstants.kFF);
		m_rightPIDController.setOutputRange(ChassisConstants.kMinSpeedMPS, ChassisConstants.kMaxSpeedMPS);

		// Set the distance per pulse for the drive encoders. We can simply use the
		// distance traveled for one rotation of the wheel divided by the encoder
		// resolution.
		m_leftEncoder.setPositionConversionFactor(ChassisConstants.kVelFactor);
		m_rightEncoder.setPositionConversionFactor(ChassisConstants.kVelFactor);

		// m_leftEncoder.reset();
		// m_rightEncoder.reset();

		m_odometry = new DifferentialDriveOdometry(getAngle());

		hiPressureSensor = new AnalogInput(Constants.ChassisConstants.kHiPressureChannel);
		loPressureSensor = new AnalogInput(Constants.ChassisConstants.kLoPressureChannel);

		// compressor.setClosedLoopControl(true);

		// compressor.start();

		// boolean enabled = compressor.enabled();
		// boolean pressureSwitch = compressor.getPressureSwitchValue();
		// double current = compressor.getCompressorCurrent();

		// compressor.setClosedLoopControl(false);

		SmartDashboard.putData("AHRS Angle", ahrs);
		SmartDashboard.putData("Tank Drive", m_tankDrive);
		SmartDashboard.putData("Left Group", m_leftGroup);
		SmartDashboard.putData("Right Group", m_rightGroup);
		// SmartDashboard.putData("Left PID", m_leftPIDController);
		// SmartDashboard.putData("Right PID", m_rightPIDController);
		// SmartDashboard.putData("Left Encoder", m_leftEncoder);
		// SmartDashboard.putData("Right Encoder", m_rightEncoder);

		// SmartDashboard.putData("Compressor", compressor);
	}

	public void periodic() {
		SmartDashboard.putNumber("LM Current", leftMaster.getOutputCurrent());
		SmartDashboard.putNumber("RM Current", rightMaster.getOutputCurrent());
		SmartDashboard.putNumber("LM Temp", leftMaster.getMotorTemperature() * UnitsConstants.kC2F);
		SmartDashboard.putNumber("RM Temp", rightMaster.getMotorTemperature() * UnitsConstants.kC2F);

		SmartDashboard.putNumber("LM Position", m_leftEncoder.getPosition());
		SmartDashboard.putNumber("LM Velocity", m_leftEncoder.getVelocity());
		SmartDashboard.putNumber("RM Position", m_rightEncoder.getPosition());
		SmartDashboard.putNumber("RM Velocity", m_rightEncoder.getVelocity());

		// SmartDashboard.putBoolean("Comp Enabled", isCompEnabled());
		// SmartDashboard.putBoolean("Comp Pressure", isCompSwitch());
		// SmartDashboard.putNumber("Comp Current", getCompCurrent());
		SmartDashboard.putNumber("Hi Pressure", getHiPressure());
		SmartDashboard.putNumber("Lo Pressure", getLoPressure());
		// SmartDashboard.putData("Compressor", compressor);
		// SmartDashboard.putNumber("Lo Pressure Voltage",
		// loPressureSensor.getVoltage());
		// SmartDashboard.putNumber("Hi Pressure Voltage",
		// hiPressureSensor.getVoltage());
	}

	public void driveTeleop(double left, double right) {
		m_tankDrive.tankDrive(left, right);
		// m_leftGroup.set(left);
		// m_rightGroup.set(right);
	}

	/**
	 * Returns the angle of the robot as a Rotation2d.
	 *
	 * @return The angle of the robot.
	 */
	public Rotation2d getAngle() {
		// Negating the angle because WPILib gyros are CW positive.
		return Rotation2d.fromDegrees(-ahrs.getAngle());
	}

	/**
	 * Sets the desired wheel speeds.
	 *
	 * @param speeds The desired wheel speeds.
	 */
	public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
		// double leftOutput =
		// m_leftPIDController.calculate(m_leftEncoder.getVelocity(),
		// speeds.leftMetersPerSecond);
		// double rightOutput =
		// m_rightPIDController.calculate(m_rightEncoder.getVelocity(),
		// speeds.rightMetersPerSecond);
		// m_leftGroup.set(leftOutput);
		// m_rightGroup.set(rightOutput);
	}

	/**
	 * Drives the robot with the given linear velocity and angular velocity.
	 *
	 * @param xSpeed Linear velocity in m/s.
	 * @param rot    Angular velocity in rad/s.
	 */
	// @SuppressWarnings("ParameterName")
	public void drive(double xSpeed, double rot) {
		var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
		setSpeeds(wheelSpeeds);
	}

	/**
	 * Updates the field-relative position.
	 */
	public void updateOdometry() {
		m_odometry.update(getAngle(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
	}

	// public void setLMTgtPosition(double pos) {
	// tgtPosition = pos;
	// // pid.setReference(pos * REV_PER_INCH_MOTOR, ControlType.kSmartMotion,
	// // RobotMap.kSlot_Position);
	// pid.setReference(pos, ControlType.kSmartMotion, RobotMap.kSlot_Position);
	// m_leftPIDController.setRefer
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
		return 250.0 * (loPressureSensor.getVoltage() / Constants.ChassisConstants.kInputVoltage) - 25.0;
	}

	public double getHiPressure() {
		return 250.0 * (hiPressureSensor.getVoltage() / Constants.ChassisConstants.kInputVoltage) - 25.0;
	}
}