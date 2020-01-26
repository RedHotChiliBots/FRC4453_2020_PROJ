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

  private Spinner spinner;
  private COLOR color;
  String gameData;
  char gotoColor;

  public SpinnerStopOnColor(Spinner spinner) {
    this.spinner = spinner;
    addRequirements(spinner);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    gotoColor = 'X';
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    gameData = "Y";
    if (gameData.length() > 0) {
      gotoColor = gameData.charAt(0);
      spinner.setRPMs(SpinnerConstants.kStopOnColorRPMs);
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    color = spinner.getColor();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return ((spinner.getStopOnColor(gotoColor) == color) || (gotoColor == 'X'));
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    spinner.setRPMs(SpinnerConstants.kStopRPMs);
  }
}
