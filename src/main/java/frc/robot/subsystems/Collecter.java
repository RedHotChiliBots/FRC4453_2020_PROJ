/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.CollecterConstants;

/**
 * Add your docs here.
 */
public class Collecter extends SubsystemBase {

	DoubleSolenoid collecterSolenoid = new DoubleSolenoid(CollecterConstants.kCollecterExtendSolenoid,
			CollecterConstants.kCollecterRetractSolenoid);

	// climberSolenoid.set(kOff);
	// climberSolenoid.set(kForward);
	// climberSolenoid.set(kReverse);

	public Collecter() {
		System.out.println("+++++ Climber Constructor starting ...");

		SmartDashboard.putData("Climber Solenoid", collecterSolenoid);

		System.out.println("----- Climber Constructor finished ...");
	}

	public void collecterExtend() {
		collecterSolenoid.set(ClimberConstants.ClimberExtend);
	}

	public void collecterRetract() {
		collecterSolenoid.set(ClimberConstants.ClimberRetract);
	}
}
