/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SpinnerConstants;
import frc.robot.Constants.SpinnerConstants.COLOR;
import frc.robot.subsystems.Spinner;

public class SpinnerStopOnColor extends CommandBase {

  private Spinner m_spinner;
  private COLOR color;
  char gameColor;

  public SpinnerStopOnColor(Spinner subsystem) {
    m_spinner = subsystem;
    addRequirements(subsystem);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    gameColor = DriverStation.getInstance().getGameSpecificMessage().charAt(0);
    m_spinner.setSetPoint(SpinnerConstants.kStopOnColorRPMs);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    color = m_spinner.getColor();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return m_spinner.getStopOnColor(gameColor) == color;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    m_spinner.setSetPoint(SpinnerConstants.kStopRPMs);

  }
}
