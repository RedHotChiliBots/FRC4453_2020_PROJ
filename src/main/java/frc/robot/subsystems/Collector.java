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

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CollectorConstants;

/**
 * Add your docs here.
 */
public class Collector extends SubsystemBase {

	DoubleSolenoid collectorSolenoid = new DoubleSolenoid(CollectorConstants.kCollectorExtendSolenoid,
			CollectorConstants.kCollectorRetractSolenoid);

	private final TalonSRX collectorMotor = new TalonSRX(CollectorConstants.kCollectorMotor);

	private double collectorSetPoint;
	// climberSolenoid.set(kOff);
	// climberSolenoid.set(kForward);
	// climberSolenoid.set(kReverse);

	public Collector() {
		System.out.println("+++++ Collector Constructor starting ...");

		SmartDashboard.putData("Collector Solenoid", collectorSolenoid);

		/* Factory Default all hardware to prevent unexpected behaviour */
		collectorMotor.configFactoryDefault();
		collectorMotor.clearStickyFaults();

		/* Config sensor used for Primary PID [Velocity] */
		collectorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
				CollectorConstants.kPIDLoopIdx, CollectorConstants.kTimeoutMs);

		// Conifigure motor controller
		collectorMotor.setSensorPhase(false); // Positive Sensor Reading should match Green (blinking) Leds on Talon
		collectorMotor.setNeutralMode(NeutralMode.Brake); // Brake motor on neutral input
		collectorMotor.setInverted(false); // Run motor in normal rotation with positive input

		/* Config the peak and nominal outputs */
		collectorMotor.configNominalOutputForward(0, CollectorConstants.kTimeoutMs);
		collectorMotor.configNominalOutputReverse(0, CollectorConstants.kTimeoutMs);
		collectorMotor.configPeakOutputForward(1, CollectorConstants.kTimeoutMs);
		collectorMotor.configPeakOutputReverse(-1, CollectorConstants.kTimeoutMs);

		/* Config the Velocity closed loop gains in slot0 */
		collectorMotor.config_kF(CollectorConstants.kPIDLoopIdx, CollectorConstants.kFF, CollectorConstants.kTimeoutMs);
		collectorMotor.config_kP(CollectorConstants.kPIDLoopIdx, CollectorConstants.kP, CollectorConstants.kTimeoutMs);
		collectorMotor.config_kI(CollectorConstants.kPIDLoopIdx, CollectorConstants.kI, CollectorConstants.kTimeoutMs);
		collectorMotor.config_kD(CollectorConstants.kPIDLoopIdx, CollectorConstants.kD, CollectorConstants.kTimeoutMs);

		System.out.println("----- Collector Constructor finished ...");
	}

	public void collectorExtend() {
		collectorSolenoid.set(CollectorConstants.CollectorExtend);
	}

	public void collectorRetract() {
		collectorSolenoid.set(CollectorConstants.CollectorRetract);
	}

	/**
	 * Get current speed (rpms) of the Spinner motor
	 * 
	 * @return rpm - scaled speed to rpms
	 */
	public double getRPMs() {
		return collectorMotor.getSelectedSensorVelocity(CollectorConstants.kPIDLoopIdx) / CollectorConstants.kVelFactor;
	}

	/**
	 * Set speed (rpms) of Spinner motor/gearbox.
	 * 
	 * @param rpm - desired speed (rpms) of motor/gearbox
	 */
	public void setRPMs(double rpm) {
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
