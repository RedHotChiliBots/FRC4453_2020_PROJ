/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AngleConstants;
import frc.robot.subsystems.Shooter;

public class ShooterAngleFind extends CommandBase {

  private final Shooter shooter;
  private boolean movingLeft = true;

  public ShooterAngleFind(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    shooter.moveAngleLeft(AngleConstants.kAngleFindSpeed);
    movingLeft = true;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    if (!shooter.getAngleCenterPos()) {
      if (movingLeft && shooter.getAngleLeftLimit()) {
        movingLeft = false;
        shooter.moveAngleRight(AngleConstants.kAngleFindSpeed);
      } else if (!movingLeft && shooter.getAngleRightLimit()) {
        movingLeft = true;
        shooter.moveAngleLeft(AngleConstants.kAngleFindSpeed);
      }
    }
  }

  @Override
  public boolean isFinished() {
    return shooter.getAngleCenterPos();
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }
}
