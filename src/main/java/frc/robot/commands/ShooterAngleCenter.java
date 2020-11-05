/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.YawConstants;
import frc.robot.subsystems.Shooter;

public class ShooterAngleCenter extends CommandBase {

  private final Shooter shooter;
  private boolean movingLeft = true;
  private boolean thisFound = false;
  private boolean lastFound = false;
  private double leftPos = 0;
  private double rightPos = 0;

  public ShooterAngleCenter(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    shooter.moveAngleLeft(YawConstants.kAngleCenterSpeed);
    movingLeft = true;
    thisFound = shooter.getAngleCenterPos();
    lastFound = shooter.getAngleCenterPos();
    leftPos = 0;
    rightPos = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    // capture current Center position
    thisFound = shooter.getAngleCenterPos();

    // if transition from not seeing Center to seeing Center, capture angle
    if (thisFound && (thisFound != lastFound)) {
      if (movingLeft) {
        leftPos = shooter.getAnglePosition();
      } else {
        rightPos = shooter.getAnglePosition();
      }
    }

    // if transition off Center position, switch direction
    if (!thisFound && (thisFound != lastFound)) {
      if (movingLeft) {
        movingLeft = false;
        shooter.moveAngleRight(YawConstants.kAngleCenterSpeed);
      } else if (!movingLeft) {
        movingLeft = true;
        shooter.moveAngleLeft(YawConstants.kAngleCenterSpeed);
      }
    }

    // reset last found
    lastFound = thisFound;

    shooter.sbLeftPos.setDouble(leftPos);
    shooter.sbRightPos.setDouble(rightPos);
  }

  @Override
  public boolean isFinished() {
    if (leftPos != 0 && rightPos != 0) {
      return true;
    } else {
      return false;
    }
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    double centerPos = (leftPos + rightPos) / 2.0;
    shooter.sbCenterPos.setDouble(centerPos);
    shooter.setAngleTarget(centerPos);
    shooter.setAngleZeroPos();
  }
}
