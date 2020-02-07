/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ChassisConstants;
import frc.robot.subsystems.Chassis;

public class AutonDriveDistance extends CommandBase {

  private DoubleSupplier spd;
  private DoubleSupplier rot;
  private final Chassis chassis;

  public AutonDriveDistance(DoubleSupplier spd, DoubleSupplier rot, Chassis chassis) {
    this.spd = spd;
    this.rot = rot;
    this.chassis = chassis;
    addRequirements(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    double xSpeed = spd.getAsDouble() * ChassisConstants.kMaxSpeedMPS;
    double xRot = rot.getAsDouble() * ChassisConstants.kMaxAngularSpeed;
    chassis.drive(xSpeed, xRot);

    chassis.updateOdometry();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }
}