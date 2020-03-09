/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.CollectArmConstants;
import frc.robot.subsystems.Collector;

public class CollectorArmInit extends CommandBase {

  private final Collector collector;
  private boolean finished = false;

  public CollectorArmInit(Collector collector) {
    this.collector = collector;
    addRequirements(collector);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    collector.moveArmUp(CollectArmConstants.kArmFindSpeed);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    if (collector.getArmAmps() > CollectArmConstants.kCollectArmAmps) {
      collector.setArmZeroPos();
      finished = true;
    }
  }

  @Override
  public boolean isFinished() {
    return finished;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }
}
