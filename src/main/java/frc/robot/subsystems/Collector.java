/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Library;
import frc.robot.Constants.CANidConstants;
import frc.robot.Constants.CollectorConstants;
import frc.robot.Constants.CollectArmConstants;
import frc.robot.Constants.PneumaticConstants;

/**
 * Add your docs here.
 */
public class Collector extends SubsystemBase {

	private final DoubleSolenoid collectorSolenoid = new DoubleSolenoid(PneumaticConstants.kCollectorExtendSolenoid,
			PneumaticConstants.kCollectorRetractSolenoid);

	private final CANSparkMax CollectArmMotor = new CANSparkMax(CANidConstants.kCollectArmMotor, MotorType.kBrushless);

	private final CANPIDController CollectArmPIDController = new CANPIDController(CollectArmMotor);
	private final CANEncoder CollectArmEncoder = new CANEncoder(CollectArmMotor);

	private final TalonSRX collectorMotor = new TalonSRX(CANidConstants.kCollectorMotor);

	private double collectArmSetPoint;
	private double collectorSetPoint;

	// climberSolenoid.set(kOff);
	// climberSolenoid.set(kForward);
	// climberSolenoid.set(kReverse);

	private final ShuffleboardTab collectorTab = Shuffleboard.getTab("Collector");
	private NetworkTableEntry sbCollectArmVel = collectorTab.addPersistent("CollectArmVelocity", 0).getEntry();
	private NetworkTableEntry sbCollectArmSetPoint = collectorTab.addPersistent("CollectArm SetPoint", 0).getEntry();
	private NetworkTableEntry sbCollectorTgt = collectorTab.addPersistent("Collector Target (rpm)", 0).getEntry();
	private NetworkTableEntry sbCollectorVel = collectorTab.addPersistent("Collector Velocity (rpm)", 0).getEntry();

	private NetworkTableEntry sbCollectSolenoid = collectorTab.addPersistent("Collector Solenoid", 0).getEntry();

	private Library lib = new Library();

	public Collector() {
		System.out.println("+++++ Collector Constructor starting ...");

		CollectArmMotor.restoreFactoryDefaults();
		CollectArmMotor.clearFaults();

		CollectArmMotor.setIdleMode(IdleMode.kBrake);
		CollectArmMotor.setInverted(true);

		CollectArmPIDController.setP(CollectArmConstants.kP);
		CollectArmPIDController.setI(CollectArmConstants.kI);
		CollectArmPIDController.setD(CollectArmConstants.kD);
		CollectArmPIDController.setIZone(CollectArmConstants.kIz);
		CollectArmPIDController.setFF(CollectArmConstants.kFF);
		CollectArmPIDController.setOutputRange(CollectArmConstants.kMinOutput, CollectArmConstants.kMaxOutput);

		/* Factory Default all hardware to prevent unexpected behaviour */
		collectorMotor.configFactoryDefault();
		collectorMotor.clearStickyFaults();

		/* Config sensor used for Primary PID [Velocity] */
		collectorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, CANidConstants.kPIDLoopIdx,
				CANidConstants.kTimeoutMs);

		// Conifigure motor controller
		collectorMotor.setSensorPhase(false); // Positive Sensor Reading should match Green (blinking) Leds on Talon
		collectorMotor.setNeutralMode(NeutralMode.Brake); // Brake motor on neutral input
		collectorMotor.setInverted(true); // Run motor in normal rotation with positive input

		/* Config the peak and nominal outputs */
		collectorMotor.configNominalOutputForward(0, CANidConstants.kTimeoutMs);
		collectorMotor.configNominalOutputReverse(0, CANidConstants.kTimeoutMs);
		collectorMotor.configPeakOutputForward(1, CANidConstants.kTimeoutMs);
		collectorMotor.configPeakOutputReverse(-1, CANidConstants.kTimeoutMs);

		/* Config the Velocity closed loop gains in slot0 */
		collectorMotor.config_kF(CANidConstants.kPIDLoopIdx, CollectorConstants.kFF, CANidConstants.kTimeoutMs);
		collectorMotor.config_kP(CANidConstants.kPIDLoopIdx, CollectorConstants.kP, CANidConstants.kTimeoutMs);
		collectorMotor.config_kI(CANidConstants.kPIDLoopIdx, CollectorConstants.kI, CANidConstants.kTimeoutMs);
		collectorMotor.config_kD(CANidConstants.kPIDLoopIdx, CollectorConstants.kD, CANidConstants.kTimeoutMs);

		System.out.println("----- Collector Constructor finished ...");
	}

	// Called once per Robot execution loop - 50Hz
	public void periodic() {

		sbCollectArmVel.setDouble(getCollectArmPosition());
		sbCollectArmSetPoint.setDouble(collectArmSetPoint);
		sbCollectorTgt.setDouble(collectorSetPoint);
		sbCollectorVel.setDouble(getCollectorRPMs());

		String str = "";
		if (collectorSolenoid.get() == CollectorConstants.CollectorExtend) {
			str = "Extend";
		} else if (collectorSolenoid.get() == CollectorConstants.CollectorRetract) {
			str = "Retract";
		} else {
			str = "Unknown";
		}
		sbCollectSolenoid.setString(str);
	}

	public void collectorExtend() {
		collectorSolenoid.set(CollectorConstants.CollectorExtend);
	}

	public void collectorRetract() {
		collectorSolenoid.set(CollectorConstants.CollectorRetract);
	}

	public double getCollectArmPosition() {
		return CollectArmEncoder.getPosition();
	}

	public void setCollectArmPosition(double pos) {
		this.collectArmSetPoint = lib.Clip(pos, CollectArmConstants.kMaxPos, CollectArmConstants.kMinPos);
		CollectArmPIDController.setReference(collectArmSetPoint, ControlType.kPosition);
	}

	/**
	 * Get current speed (rpms) of the Spinner motor
	 * 
	 * @return rpm - scaled speed to rpms
	 */
	public double getCollectorRPMs() {
		return collectorMotor.getSelectedSensorVelocity(CANidConstants.kPIDLoopIdx) / CollectorConstants.kVelFactor;
	}

	/**
	 * Set speed (rpms) of Spinner motor/gearbox.
	 * 
	 * @param rpm - desired speed (rpms) of motor/gearbox
	 */
	public void setCollectorRPMs(double rpm) {
		collectorSetPoint = rpm;
		collectorMotor.set(ControlMode.Velocity, rpm * CollectorConstants.kVelFactor);
	}

	public void stopSpin() {
		collectorMotor.set(ControlMode.PercentOutput, 0.0);
	}

	public void collectorCollect() {
		collectorMotor.set(ControlMode.PercentOutput, .8);
	}

	public void collectorEject() {
		collectorMotor.set(ControlMode.PercentOutput, -.8);
	}

	public void collectorStop() {
		collectorMotor.set(ControlMode.PercentOutput, -0.0);
	}

}
