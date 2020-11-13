/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

import frc.robot.Constants.YawConstants;
import frc.robot.Library;
import frc.robot.Constants.TiltConstants;

public class TurretAimJoystick extends CommandBase {

	DoubleSupplier tiltJoystick;
	DoubleSupplier yawJoystick;

	Turret turret = null;

	double tiltCmd = 0;
	double yawCmd = 0;
	double tiltNew = 0;
	double yawNew = 0;

	Library lib = new Library();

	/**
	 * Creates a new AimShooter.
	 */
	public TurretAimJoystick(DoubleSupplier tiltJoystick, DoubleSupplier yawJoystick, Turret turret) {
		this.turret = turret;
		this.tiltJoystick = tiltJoystick;
		this.yawJoystick = yawJoystick;
		addRequirements(turret);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// Read joystick position
		tiltCmd = -tiltJoystick.getAsDouble();
		System.out.println("tiltCmd " + tiltCmd);
		// Disable until Yaw PID is adjusted
		yawCmd = yawJoystick.getAsDouble();	// -yawJoystick.getAsDouble();

		// Zero out deadZone to avoid creep
		tiltCmd = (Math.abs(tiltCmd) < YawConstants.kJoystickDeadZone ? 0.0 : tiltCmd);
		yawCmd = (Math.abs(yawCmd) < YawConstants.kJoystickDeadZone ? 0.0 : yawCmd);
	
		// Calculate new target as Curr Target + Increment 
		tiltNew = turret.getTiltTarget() + (tiltCmd * TiltConstants.kRateDpS);
		yawNew = turret.getAngleTarget() + (yawCmd * YawConstants.kRateDpS);

		System.out.println("tiltNew " + tiltNew);
//		System.out.println("yawNew " + yawNew);
		// Clamp new targets to min and max values
		tiltNew = lib.Clip(tiltNew, TiltConstants.kMaxDeg, TiltConstants.kMinDeg);
		yawNew = lib.Clip(yawNew, YawConstants.kMaxDeg, YawConstants.kMinDeg);

		// Update motor controllers with new targets
		turret.setTiltTarget(tiltNew);
		turret.setAngleTarget(yawNew);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
