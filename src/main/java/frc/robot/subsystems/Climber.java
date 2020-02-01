/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.d
//import edu.wpi.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

/**
 * Add your docs here.
 */

public class Climber extends SubsystemBase {

	DoubleSolenoid climberSolenoid = new DoubleSolenoid(ClimberConstants.kClimberExtendSolenoid,
			ClimberConstants.kClimberRetractSolenoid);

	// climberSolenoid.set(kOff);
	// climberSolenoid.set(kForward);
	// climberSolenoid.set(kReverse);

	public Climber() {
		System.out.println("+++++ Climber Constructor starting ...");

		SmartDashboard.putData("Climber Solenoid", climberSolenoid);

		System.out.println("----- Climber Constructor finished ...");
	}

	public void climberExtend() {
		climberSolenoid.set(ClimberConstants.ClimberExtend);
	}

	public void climberRetract() {
		climberSolenoid.set(ClimberConstants.ClimberRetract);
	}
}
