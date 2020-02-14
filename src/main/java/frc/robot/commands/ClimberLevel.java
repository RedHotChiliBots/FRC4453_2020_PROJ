/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;

public class ClimberLevel extends CommandBase {

  private Climber climber;
  private DoubleSupplier dsspeed = null;
  private double speed = 0.0;

  public ClimberLevel(Climber climber, DoubleSupplier dsspeed) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.climber = climber;
    this.dsspeed = dsspeed;
    addRequirements(climber);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    speed = dsspeed.getAsDouble();
    climber.levelTeleop(speed);
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
