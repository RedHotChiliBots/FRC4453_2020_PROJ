/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class DriveTeleop extends CommandBase {

  private final Chassis m_chassis;
  private DoubleSupplier left;
  private DoubleSupplier right;
  private ShuffleboardTab chassisTab;

  public DriveTeleop(Chassis chassis, DoubleSupplier left, DoubleSupplier right) {
    this.m_chassis = chassis;
    this.left = left;
    this.right = right;
    addRequirements(chassis);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    chassisTab = Shuffleboard.getTab("Chassis");
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    chassisTab.add("Left Y", left.getAsDouble());
    chassisTab.add("Right Y", right.getAsDouble());

    m_chassis.driveTeleop(left.getAsDouble(), right.getAsDouble());
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
