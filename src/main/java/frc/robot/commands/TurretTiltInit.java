/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TiltConstants;
import frc.robot.subsystems.Turret;

public class TurretTiltInit extends CommandBase {

  private final Turret turret;

  public TurretTiltInit(Turret turret) {
    this.turret = turret;
    addRequirements(turret);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
		turret.moveTiltDown(TiltConstants.kATiltFindSpeed);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(turret.getTiltAmps()) > TiltConstants.kTiltAmps;
  }

  // Called once after isFinished returns true
  @Override
	public void end(boolean interrupted) {
		turret.stopTilt();
		turret.setTiltZeroPos();
  }
}
