/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
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

	// climberSolenoid.set(kOff);
	// climberSolenoid.set(kForward);
	// climberSolenoid.set(kReverse);

	public Collector() {
		System.out.println("+++++ Collector Constructor starting ...");

		SmartDashboard.putData("Collector Solenoid", collectorSolenoid);

		collectorMotor.configFactoryDefault();
		collectorMotor.clearStickyFaults();

		// Configure Motor
		collectorMotor.setNeutralMode(NeutralMode.Brake);
		collectorMotor.setInverted(false);
		collectorMotor.setSensorPhase(false);

		System.out.println("----- Collector Constructor finished ...");
	}

	public void collectorExtend() {
		collectorSolenoid.set(CollectorConstants.CollectorExtend);
	}

	public void collectorRetract() {
		collectorSolenoid.set(CollectorConstants.CollectorRetract);
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
