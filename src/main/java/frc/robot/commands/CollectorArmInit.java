/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.CollectArmConstants;
import frc.robot.subsystems.Collector;

public class CollectorArmInit extends CommandBase {

	private final Collector collector;
	private Timer timer = new Timer();

  public CollectorArmInit(Collector collector) {
    this.collector = collector;
    addRequirements(collector);
  }

  // Called just before this Command runs the first time
  @Override
	public void initialize() {
		timer.reset();
		timer.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
	public void execute() {
		collector.moveArmUp(CollectArmConstants.kArmFindSpeed);
  }

  @Override
  public boolean isFinished() {
    return /*timer.get() < 0.25 ? false : Math.abs(collector.getArmAmps()) > CollectArmConstants.kCollectArmAmps;*/ Math.abs(collector.getArmAmps()) > Constants.CollectArmConstants.kCollectArmAmps;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
      collector.setArmPostition(CollectArmConstants.kMinPos);
      collector.setArmTarget(CollectArmConstants.kStowPos);
  }
}
