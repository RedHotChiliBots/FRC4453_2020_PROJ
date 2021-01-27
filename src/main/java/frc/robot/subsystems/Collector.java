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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Library;
import frc.robot.Constants.CANidConstants;
import frc.robot.Constants.CollectorConstants;
import frc.robot.Constants.CollectArmConstants;

/**
 * Add your docs here.
 */
public class Collector extends SubsystemBase {

	// private final DoubleSolenoid collectorSolenoid = new DoubleSolenoid////(PneumaticConstants.kCollectorExtendSolenoid,
	// 		PneumaticConstants.kCollectorRetractSolenoid);

	private final CANSparkMax collectArmMotor = new CANSparkMax(CANidConstants.kCollectArmMotor, MotorType.kBrushless);

	private final CANPIDController collectArmPIDController = collectArmMotor.getPIDController();
	// new CANPIDController(collectArmMotor);
	private final CANEncoder collectArmEncoder = collectArmMotor.getEncoder();
	// new CANEncoder(collectArmMotor);

	private final TalonSRX collectorMotor = new TalonSRX(CANidConstants.kCollectorMotor);

	private double collectorArmTarget;
	private double collectorTarget;

	// climberSolenoid.set(kOff);
	// climberSolenoid.set(kForward);
	// climberSolenoid.set(kReverse);

	private final ShuffleboardTab collectorTab = Shuffleboard.getTab("Collector");
	private NetworkTableEntry sbCollectArmPos = collectorTab.addPersistent("CollectArmPosition", 0).getEntry();
	private NetworkTableEntry sbCollectArmSetPoint = collectorTab.addPersistent("CollectArm SetPoint", 0).getEntry();
	private NetworkTableEntry sbCollectorTgt = collectorTab.addPersistent("Collector Target (rpm)", 0).getEntry();
	private NetworkTableEntry sbCollectorVel = collectorTab.addPersistent("Collector Velocity (rpm)", 0).getEntry();
	private NetworkTableEntry sbCollectArmAmps = collectorTab.addPersistent("Collector Arm Amps", 0).getEntry();

	private Library lib = new Library();

	public Collector() {
		System.out.println("+++++ Collector Constructor starting ...");

		collectArmMotor.restoreFactoryDefaults();
		collectArmMotor.clearFaults();

		collectArmMotor.setIdleMode(IdleMode.kBrake);
		collectArmMotor.setInverted(true);

		collectArmPIDController.setP(CollectArmConstants.kP);
		collectArmPIDController.setI(CollectArmConstants.kI);
		collectArmPIDController.setD(CollectArmConstants.kD);
		collectArmPIDController.setIZone(CollectArmConstants.kIz);
		collectArmPIDController.setFF(CollectArmConstants.kFF);
		collectArmPIDController.setOutputRange(CollectArmConstants.kMinOutput, CollectArmConstants.kMaxOutput);

		collectArmEncoder.setPositionConversionFactor(CollectArmConstants.kRevolutionToDegrees);
		
		/* Factory Default all hardware to prevent unexpected behaviour */
		collectorMotor.configFactoryDefault();
		collectorMotor.clearStickyFaults();

		/* Config sensor used for Primary PID [Velocity] */
		collectorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, CANidConstants.kPIDLoopIdx,
				CANidConstants.kTimeoutMs);

		// Conifigure motor controller
		// collectorMotor.setSensorPhase(false); // Positive Sensor Reading should match Green (blinking) Leds on Talon
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

		collectorStop();
		
		System.out.println("----- Collector Constructor finished ...");
	}

	// Called once per Robot execution loop - 50Hz
	public void periodic() {

		sbCollectArmPos.setDouble(getCollectArmPosition());
		sbCollectArmSetPoint.setDouble(collectorArmTarget);
		sbCollectorTgt.setDouble(collectorTarget);
		sbCollectorVel.setDouble(getCollectorRPMs());
		sbCollectArmAmps.setDouble(getArmAmps());

		// String str = "";
		// if (collectorSolenoid.get() == CollectorConstants.CollectorExtend) {
		// 	str = "Extend";
		// } else if (collectorSolenoid.get() == CollectorConstants.CollectorRetract) {
		// 	str = "Retract";
		// } else {
		// 	str = "Unknown";
		// }
		// sbCollectSolenoid.setString(str);
	}

	// public void collectorExtend() {
	// 	collectorSolenoid.set(CollectorConstants.CollectorExtend);
	// }

	// public void collectorRetract() {
	// 	collectorSolenoid.set(CollectorConstants.CollectorRetract);
	// }

	public double getCollectArmPosition() {
		return collectArmEncoder.getPosition();
	}

	public void setCollectArmPosition(double pos) {
		this.collectorArmTarget = lib.Clip(pos, CollectArmConstants.kMaxPos, CollectArmConstants.kMinPos);
		collectArmPIDController.setReference(collectorArmTarget, ControlType.kPosition);
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
		collectorTarget = rpm;
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
		collectorMotor.set(ControlMode.PercentOutput, 0.0);
	}

	public void moveArmUp(double spd) {
		collectArmMotor.set(-spd);
	}

	public void setArmTarget(double tgt) {
		collectArmPIDController.setReference(tgt, ControlType.kPosition);
	}

	public void setArmPostition(double tgt) {
		collectArmEncoder.setPosition(tgt);
	}

	public double getArmPosition() {
		return collectArmEncoder.getPosition();
	}

	public double getArmAmps() {
		return collectArmMotor.getOutputCurrent();
	}

}
