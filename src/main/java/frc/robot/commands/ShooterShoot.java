/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.*;

public class ShooterShoot extends CommandBase {
  private final Shooter shooter;

  public ShooterShoot(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
    // super(timeout);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    shooter.setShootSetPoint(ShooterConstants.shootRPMs);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    // shooter.shoot(ShooterConstants.shootSpeed);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }

  // // Called when another command which requires one or more of the same
  // // subsystems is scheduled to run
  // @Override
  // public void interrupted() {
  // }
}
