/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
<<<<<<< HEAD
import frc.robot.subsystems.Spinner;

public class SpinnerStow extends CommandBase {

  private final Spinner m_spinner;

  public SpinnerStow(Spinner subsystem) {
    m_spinner = subsystem;
    addRequirements(subsystem);
=======

public class SpinnerStow extends CommandBase {
  public SpinnerStow() {
>>>>>>> 62837b11f543b1a137b4e99fc93c6fe2036a214f
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
<<<<<<< HEAD
  public void initialize() {
=======
  protected void initialize() {
>>>>>>> 62837b11f543b1a137b4e99fc93c6fe2036a214f
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
<<<<<<< HEAD
  public void execute() {
=======
  protected void execute() {
>>>>>>> 62837b11f543b1a137b4e99fc93c6fe2036a214f
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
<<<<<<< HEAD
  public boolean isFinished() {
=======
  protected boolean isFinished() {
>>>>>>> 62837b11f543b1a137b4e99fc93c6fe2036a214f
    return false;
  }

  // Called once after isFinished returns true
  @Override
<<<<<<< HEAD
  public void end(boolean interrupted) {
=======
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
>>>>>>> 62837b11f543b1a137b4e99fc93c6fe2036a214f
  }
}
