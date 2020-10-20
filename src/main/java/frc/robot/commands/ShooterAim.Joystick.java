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

public class ShooterAimJoystick extends CommandBase {

	DoubleSupplier tiltCmd;
	DoubleSupplier yawCmd;

	Shooter shooter = null;
	
	double tiltMaxDeg = 0;
	double tiltMinDeg = 0;
	double tiltMaxSec = 0;

	double tiltRangeDeg = 0;
	double tiltRateDpS = 0;

	double yawMaxDeg = 0;
	double yawMinDeg = 0;
	double yawMaxSec = 0;

	double yawRangeDeg = 0;
	double yawRateDpS = 0;

	double tiltCurr = 0;
	double yawCurr = 0;
	double tiltNew = 0;
	double yawNew = 0;
	double currTgt;
	double inc;
	double target;

  /**
   * Creates a new AimShooter.
   */
  public ShooterAimJoystick(DoubleSupplier tiltCmd, DoubleSupplier yawCmd, Shooter shooter) {
		this.shooter = shooter;
		this.tiltCmd = tiltCmd;
		this.yawCmd = yawCmd;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
		tiltRangeDeg = tiltMaxDeg - tiltMinDeg;
		tiltRateDpS = tiltRangeDeg / tiltMaxSec / 50.0;
		
		yawRangeDeg = yawMaxDeg - yawMinDeg;
		yawRateDpS = yawRangeDeg / yawMaxSec / 50.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
	public void execute() {
		tiltCurr = tiltCmd.getAsDouble();
		yawCurr = yawCmd.getAsDouble();

		tiltNew = shooter.getTiltPosition() + (tiltCurr * tiltRateDpS);
		yawNew = shooter.getAnglePosition() + (yawCurr * yawRateDpS);

		tiltNew = Math.max(tiltMinDeg, Math.min(tiltMaxDeg, tiltNew));
		yawNew = Math.max(yawMinDeg, Math.min(yawMaxDeg, yawNew));

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
