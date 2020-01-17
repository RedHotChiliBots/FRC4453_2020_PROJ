/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SpinnerConstants;
import frc.robot.subsystems.Spinner;

public class SpinnerCountRevs extends CommandBase {

  private final Spinner m_subsystem;

  public SpinnerCountRevs(Spinner subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    m_subsystem.countColor(true);
    m_subsystem.setSetPoint(SpinnerConstants.kCountRevRPMs);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    m_subsystem.countColor(false);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return m_subsystem.sumColor() >= 25;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setSetPoint(SpinnerConstants.kStopRPMs);
  }
}
