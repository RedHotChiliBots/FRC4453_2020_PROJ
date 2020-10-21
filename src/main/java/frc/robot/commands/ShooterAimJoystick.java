/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

import frc.robot.Constants.YawConstants;
import frc.robot.Constants.TiltConstants;

public class ShooterAimJoystick extends CommandBase {

	DoubleSupplier tiltJoystick;
	DoubleSupplier yawJoystick;

	Shooter shooter = null;

	double tiltCmd = 0;
	double yawCmd = 0;
	double tiltNew = 0;
	double yawNew = 0;

	/**
	 * Creates a new AimShooter.
	 */
	public ShooterAimJoystick(DoubleSupplier tiltJoystick, DoubleSupplier yawJoystick, Shooter shooter) {
		this.shooter = shooter;
		this.tiltJoystick = tiltJoystick;
		this.yawJoystick = yawJoystick;
		addRequirements(shooter);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// Read joystick position
		tiltCmd = tiltJoystick.getAsDouble();
		yawCmd = yawJoystick.getAsDouble();

		// Calculate new target as Curr Target + Increment 
		tiltNew = shooter.getTiltPosition() + (tiltCmd * TiltConstants.kRateDpS);
		yawNew = shooter.getAnglePosition() + (yawCmd * YawConstants.kRateDpS);

		// Clamp new targets to min and max values
		tiltNew = Math.max(TiltConstants.kMinPos, Math.min(TiltConstants.kMaxPos, tiltNew));
		yawNew = Math.max(YawConstants.kMinPos, Math.min(YawConstants.kMaxPos, yawNew));

		// Update controllers with new targets
		shooter.setTiltPosition(tiltNew);
		shooter.setAnglePosition(yawNew);
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