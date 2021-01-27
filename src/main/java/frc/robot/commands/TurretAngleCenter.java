/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.YawConstants;
import frc.robot.subsystems.Turret;

public class TurretAngleCenter extends CommandBase {

  private final Turret turret;
  private boolean movingLeft = true;
  private boolean thisFound = false;
  private boolean lastFound = false;
  private double leftPos = 0;
  private double rightPos = 0;

  public TurretAngleCenter(Turret turret) {
    this.turret = turret;
    addRequirements(turret);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    turret.moveYawLeft(YawConstants.kYawCenterSpeed);
    movingLeft = true;
    thisFound = turret.getYawCenterPos();
    lastFound = turret.getYawCenterPos();
    leftPos = 0;
    rightPos = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    // capture current Center position
    thisFound = turret.getYawCenterPos();

    // if transition from not seeing Center to seeing Center, capture angle
    if (thisFound && (thisFound != lastFound)) {
      if (movingLeft) {
        leftPos = turret.getYawPosition();
      } else {
        rightPos = turret.getYawPosition();
      }
    }

    // if transition off Center position, switch direction
    if (!thisFound && (thisFound != lastFound)) {
      if (movingLeft) {
        movingLeft = false;
        turret.moveYawRight(YawConstants.kYawCenterSpeed);
      } else if (!movingLeft) {
        movingLeft = true;
        turret.moveYawLeft(YawConstants.kYawCenterSpeed);
      }
    }

    // reset last found
    lastFound = thisFound;

    turret.sbLeftPos.setDouble(leftPos);
    turret.sbRightPos.setDouble(rightPos);
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
    turret.sbCenterPos.setDouble(centerPos);
    turret.setYawTarget(centerPos);
    turret.setYawZeroPos();
  }
}
