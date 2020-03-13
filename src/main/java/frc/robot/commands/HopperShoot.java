/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.HopperConstants;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;

public class HopperShoot extends CommandBase {
	/**
	 * Creates a new HopperLoad.
	 */
	private final Hopper hopper;
	private Timer timer;
	private boolean timing;

	public HopperShoot(Hopper hopper, Shooter shooter) {
		// Use addRequirements() here to declare subsystem dependencies.
		this.hopper = hopper;
		addRequirements(hopper, shooter);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		timing = false;
		hopper.setHopperVelocity(HopperConstants.kShootRPMs);

	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (!hopper.getUpperSensor()) {
			if (!timing) {
				timer.reset();
			}
			timing = true;
		} else {
			timing = false;
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return timing && timer.get() > 3.0;
	}
}
