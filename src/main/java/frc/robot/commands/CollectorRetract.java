/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;

/**
 * Add your docs here.
 */
public class CollectorRetract extends CommandBase {
	private final Collector collector;

	public CollectorRetract(Collector collector) {
		this.collector = collector;
		addRequirements(collector);
		// super(timeout);
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		collector.collectorRetract();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	public boolean isFinished() {
		return true;
	}

	// Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {
	}

}