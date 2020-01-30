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

  private final Spinner spinner;
  // private final double setPoint = SpinnerConstants.kCountRevRPMs;

  public SpinnerCountRevs(Spinner spinner) {
    this.spinner = spinner;
    addRequirements(spinner);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    spinner.countColor(true);

    spinner.setRPMs(SpinnerConstants.kCountRevRPMs);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    spinner.countColor(false);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return spinner.sumColor() >= 25;
  }

  // Called once after isFinished returns true
  @Override
  public void end(final boolean interrupted) {
    // spinner.setSetPoint(SpinnerConstants.kStopRPMs);
    spinner.stopSpin();
  }
}
