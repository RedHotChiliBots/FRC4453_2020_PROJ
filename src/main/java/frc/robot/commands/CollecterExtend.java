/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collecter;

public class CollecterExtend extends CommandBase {
  private final Collecter m_collecter;

  public CollecterExtend(Collecter collecter) {
    m_collecter = collecter;
    addRequirements(collecter);
    // super(timeout);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    m_collecter.collecterExtend();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
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

}
