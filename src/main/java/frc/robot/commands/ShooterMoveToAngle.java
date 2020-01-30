/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShooterMoveToAngle extends CommandBase {

	private Shooter shooter;
	private double pos;

	public ShooterMoveToAngle(Shooter shooter, double pos) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		this.shooter = shooter;
		this.pos = pos;

	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
		shooter.setAnglePosition(pos);

	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	public boolean isFinished() {
		return shooter.getAnglePosition() == pos;
	}

	// Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	// @Override
	// public void interrupted() {
	// }
}
