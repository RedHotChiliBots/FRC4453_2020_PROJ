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
  private boolean finished = false;

  public ShooterTiltInit(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    shooter.moveTiltDown(TiltConstants.kATiltFindSpeed);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    if (shooter.getTiltAmps() > TiltConstants.kTiltAmps) {
      shooter.setTiltZeroPos();
      finished = true;
    }
  }

  @Override
  public boolean isFinished() {
    return finished;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }
}
