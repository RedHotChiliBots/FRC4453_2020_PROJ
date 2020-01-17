/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
// import edu.wpi.first.wpilibj.AnalogGyro;
// import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.ChassisConstants;
import frc.robot.Constants.OIConstants;
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

	// private final Encoder m_leftEncoder = new Encoder(0, 1);
	public final CANEncoder m_leftEncoder = new CANEncoder(leftMaster);
	public final CANEncoder m_rightEncoder = new CANEncoder(rightMaster);
	// private final Encoder m_rightEncoder = new Encoder(2, 3);

	private final SpeedControllerGroup m_leftGroup = new SpeedControllerGroup(m_leftMaster, m_leftFollower);
	private final SpeedControllerGroup m_rightGroup = new SpeedControllerGroup(m_rightMaster, m_rightFollower);

	/* Alternatively: I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB */
	private final AHRS ahrs = new AHRS(SPI.Port.kMXP);
	// private final AnalogGyro m_gyro = new AnalogGyro(0);

	private final CANPIDController m_leftPIDController = new CANPIDController(leftMaster);
	private final CANPIDController m_rightPIDController = new CANPIDController(rightMaster);

	// private final PIDController m_rightPIDController = new PIDController(1, 0,
	// 0);

	// Compressor c = new Compressor(0);

	// c.setClosedLoopControl(true);
	// c.setClosedLoopControl(false);

	// boolean enabled = c.enabled();
	// boolean pressureSwitch = c.getPressureSwitchValue();
	// double current = c.getCompressorCurrent();

	private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(
			ChassisConstants.kTrackWidth);

	private final DifferentialDriveOdometry m_odometry;

	XboxController m_driver = new XboxController(OIConstants.kDriverControllerPort);
	XboxController m_operator = new XboxController(OIConstants.kOperatorControllerPort);

	/**
	 * Constructs a differential drive object. Sets the encoder distance per pulse
	 * and resets the gyro.
	 */
	public Chassis() {
		super();
		System.out.println("Chassis starting ...");

		ahrs.reset();
		ahrs.zeroYaw();

		// m_leftPIDController.reset();
		// m_rightPIDController.reset();

		// Set the distance per pulse for the drive encoders. We can simply use the
		// distance traveled for one rotation of the wheel divided by the encoder
		// resolution.
		m_leftEncoder.setPositionConversionFactor(2 * Math.PI * ChassisConstants.kWheelRadius);

		// .setDistancePerPulse(2 * Math.PI * ChassisConstants.kWheelRadius /
		// ChassisConstants.kEncoderResolution);
		m_rightEncoder.setPositionConversionFactor(2 * Math.PI * ChassisConstants.kWheelRadius);
		// .setDistancePerPulse(2 * Math.PI * ChassisConstants.kWheelRadius /
		// ChassisConstants.kEncoderResolution);

		// m_leftEncoder.reset();
		// m_rightEncoder.reset();

		m_odometry = new DifferentialDriveOdometry(getAngle());

		SmartDashboard.putData("AHRS Angle", ahrs);
		// SmartDashboard.putData("Left PID", m_leftPIDController);
		// SmartDashboard.putData("Right PID", m_rightPIDController);
		// SmartDashboard.putData("Left Master", m_leftEncoder);
		// SmartDashboard.putData("Right Master", m_rightMaster);
		SmartDashboard.putData("Left Group", m_leftGroup);
		SmartDashboard.putData("Right Group", m_rightGroup);

	}

	public void periodic() {
		// SmartDashboard.putData("Left Encoder", leftMaster);

		SmartDashboard.putNumber("Left Y", -m_driver.getY(GenericHID.Hand.kLeft));
		SmartDashboard.putNumber("Right Y", -m_driver.getY(GenericHID.Hand.kRight));

		SmartDashboard.putNumber("LM Current", leftMaster.getOutputCurrent());
		SmartDashboard.putNumber("RM Current", rightMaster.getOutputCurrent());
		SmartDashboard.putNumber("LM Temp", leftMaster.getMotorTemperature() * UnitsConstants.kC2F);
		SmartDashboard.putNumber("RM Temp", rightMaster.getMotorTemperature() * UnitsConstants.kC2F);

		SmartDashboard.putNumber("LM Position", m_leftEncoder.getPosition());
		SmartDashboard.putNumber("LM Velocity", m_leftEncoder.getVelocity());
		SmartDashboard.putNumber("RM Position", m_rightEncoder.getPosition());
		SmartDashboard.putNumber("RM Velocity", m_rightEncoder.getVelocity());

		// Shuffleboard.getTab("Tab3").add("L Encoder Position",
		// m_leftEncoder.getPosition());
		// Shuffleboard.getTab("Tab3").add("R Encoder Position",
		// m_rightEncoder.getPosition());
	}

	public void driveTeleop(double left, double right) {
		m_leftGroup.set(left);
		m_rightGroup.set(right);
		SmartDashboard.putNumber("Teleop Left Y", left);
		SmartDashboard.putNumber("Teleop Right Y", right); 
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

}