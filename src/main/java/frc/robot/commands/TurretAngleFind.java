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

public class TurretAngleFind extends CommandBase {

  private final Turret turret;
  private boolean movingLeft = true;

  public TurretAngleFind(Turret turret) {
    this.turret = turret;
    addRequirements(turret);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    turret.moveAngleLeft(YawConstants.kAngleFindSpeed);
    movingLeft = true;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    if (!turret.getAngleCenterPos()) {
      if (movingLeft && turret.getAngleLeftLimit()) {
        movingLeft = false;
        turret.moveAngleRight(YawConstants.kAngleFindSpeed);
      } else if (!movingLeft && turret.getAngleRightLimit()) {
        movingLeft = true;
        turret.moveAngleLeft(YawConstants.kAngleFindSpeed);
      }
    }
  }

  @Override
  public boolean isFinished() {
    return turret.getAngleCenterPos();
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }
}
