/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Chassis;

public class AutonDrive2Point extends CommandBase {

  private Pose2d tgtPose;
  private double dist;
  private final Chassis chassis;

  private final ShuffleboardTab visionTab = Shuffleboard.getTab("Vision");
  private NetworkTableEntry sbLimeSpd1 = visionTab.addPersistent("Lime Spd1", 0).getEntry();
  private NetworkTableEntry sbLimeRot1 = visionTab.addPersistent("Lime Rot1", 0).getEntry();
  private NetworkTableEntry sbLimeDist = visionTab.addPersistent("Lime Dist", 0).getEntry();
  private NetworkTableEntry sbLimeAngle = visionTab.addPersistent("Lime Angle", 0).getEntry();

  private NetworkTableEntry sbRobotX = visionTab.addPersistent("Robot X", 0).getEntry();
  private NetworkTableEntry sbRobotY = visionTab.addPersistent("Robot Y", 0).getEntry();
  private NetworkTableEntry sbRobotDeg = visionTab.addPersistent("Robot Deg", 0).getEntry();

  private NetworkTableEntry sbTgtX = visionTab.addPersistent("Target X", 0).getEntry();
  private NetworkTableEntry sbTgtY = visionTab.addPersistent("Target Y", 0).getEntry();
  private NetworkTableEntry sbTgtDeg = visionTab.addPersistent("Target Deg", 0).getEntry();

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
    double robotX = robotPose.getTranslation().getX();
    double robotY = robotPose.getTranslation().getY();
    double robotDeg = robotPose.getRotation().getDegrees();

    sbRobotX.setDouble(robotX);
    sbRobotY.setDouble(robotY);
    sbRobotDeg.setDouble(robotDeg);

    double tgtX = tgtPose.getTranslation().getX();
    double tgtY = tgtPose.getTranslation().getY();
    double tgtDeg = tgtPose.getRotation().getDegrees();

    sbTgtX.setDouble(tgtX);
    sbTgtY.setDouble(tgtY);
    sbTgtDeg.setDouble(tgtDeg);

    dist = robotPose.getTranslation().getDistance(tgtPose.getTranslation());
    double dX = tgtPose.getTranslation().getX() - robotPose.getTranslation().getX();
    double dY = tgtPose.getTranslation().getY() - robotPose.getTranslation().getY();
    double angle = new Rotation2d(dX, dY).getDegrees();

    chassis.getOdometry().resetPosition(new Pose2d(robotX, robotY, new Rotation2d(angle)), new Rotation2d(angle));

    double xSpeed = chassis.getDistPID().calculate(dist, 0.0);
    double xRot = chassis.getRotPID().calculate(angle, 0.0);

    sbLimeSpd1.setDouble(-xSpeed);
    sbLimeRot1.setDouble(xRot);
    sbLimeDist.setDouble(dist);
    sbLimeAngle.setDouble(angle);

    chassis.drive(-xSpeed, xRot);
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
