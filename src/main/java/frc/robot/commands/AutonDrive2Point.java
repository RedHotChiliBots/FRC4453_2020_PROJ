/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Chassis;

public class AutonDrive2Point extends CommandBase {

  private Pose2d tgtPose;
  private double dist;
  private final Chassis chassis;

  public AutonDrive2Point(Pose2d tgtPose, Chassis chassis) {
    this.tgtPose = tgtPose;
    this.chassis = chassis;
    addRequirements(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    Pose2d robotPose = chassis.getOdometry().getPoseMeters();

    dist = robotPose.getTranslation().getDistance(tgtPose.getTranslation());
    double dX = tgtPose.getTranslation().getX() - robotPose.getTranslation().getX();
    double dY = tgtPose.getTranslation().getY() - robotPose.getTranslation().getY();
    double angle = new Rotation2d(dX, dY).getDegrees();

    double xSpeed = chassis.getDistPID().calculate(dist, 0.0);
    double xRot = chassis.getRotPID().calculate(angle, 0.0);

    SmartDashboard.putNumber("Pose Dist", dist);
    SmartDashboard.putNumber("Pose Angle", angle);

    SmartDashboard.putNumber("Lime Spd", xSpeed);
    SmartDashboard.putNumber("Lime Rot", xRot);

    chassis.drive(xSpeed, xRot);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return dist < 1.0;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }
}
