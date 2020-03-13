/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TiltConstants;
import frc.robot.subsystems.Shooter;

public class ShooterTiltInit extends CommandBase {

  private final Shooter shooter;

  public ShooterTiltInit(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
		shooter.moveTiltDown(TiltConstants.kATiltFindSpeed);
  }

  @Override
  public boolean isFinished() {
    return shooter.getTiltAmps() > TiltConstants.kTiltAmps;
  }

  // Called once after isFinished returns true
  @Override
	public void end(boolean interrupted) {
		shooter.stopTilt();
		shooter.setTiltZeroPos();
  }
}
