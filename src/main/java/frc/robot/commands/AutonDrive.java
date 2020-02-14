/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class AutonDrive extends CommandBase {

  private final Chassis chassis;
  private final double dist;

  public AutonDrive(Chassis chassis, double dist) {
    this.chassis = chassis;
    this.dist = dist;
    addRequirements(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    // System.out.println("AutonDrive running");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    // chassis.autonDrive(dist);
    chassis.driveDistance(dist);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return chassis.distanceOnTarget();
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }
}
