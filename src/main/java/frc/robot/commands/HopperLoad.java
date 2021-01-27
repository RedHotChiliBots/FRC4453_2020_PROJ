/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.HopperConstants;
import frc.robot.subsystems.Hopper;
import edu.wpi.first.wpilibj.Timer;

public class HopperLoad extends CommandBase {
	/**
	 * Creates a new HopperLoad.
	 */
	private final Hopper hopper;
	private boolean force;
	private Timer timer = new Timer();

	public HopperLoad(Hopper hopper, boolean force) {
		// Use addRequirements() here to declare subsystem dependencies.
		this.hopper = hopper;
		this.force = force;
		addRequirements(hopper);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		timer.reset();
		timer.start();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (force || hopper.getLowerSensor()) {
			hopper.setHopperVelocity(HopperConstants.kLoadRPMs);
		} else {
			hopper.stopHopper();
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		hopper.stopHopper();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return timer.get() < 0.25 ? false :  hopper.getUpperSensor();
	}
}
