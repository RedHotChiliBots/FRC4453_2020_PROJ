/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.Constants.YawConstants;
import frc.robot.subsystems.Shooter;

public class ShooterAngleCenter extends CommandBase {

  private final Shooter shooter;
  private boolean movingLeft = true;
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
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    if (!shooter.getAngleCenterPos()) {
      if (movingLeft) {
        movingLeft = false;
        leftPos = shooter.getAnglePosition();
        shooter.moveAngleRight(YawConstants.kAngleCenterSpeed);
      } else if (!movingLeft) {
        movingLeft = true;
        rightPos = shooter.getAnglePosition();
        shooter.moveAngleLeft(YawConstants.kAngleCenterSpeed);
      }
    }
    shooter.sbLeftPos.setDouble(leftPos);
    shooter.sbRightPos.setDouble(rightPos);
  }

  @Override
  public boolean isFinished() {
    if (leftPos != 0 && rightPos != 0) {
      shooter.setAnglePosition((leftPos + rightPos) / 2.0);
      shooter.setAngleZeroPos();
      return true;
    } else {
      return false;
    }
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }
}
