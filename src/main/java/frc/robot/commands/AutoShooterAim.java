/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;

public class AutoShooterAim extends CommandBase {

  Shooter shooter = null;
  double x = 0.0;
  double y = 0.0;
  DoubleSupplier dsx = null;
  DoubleSupplier dsy = null;

  private double distance = 0.0;

  private final ShuffleboardTab visionTab = Shuffleboard.getTab("Vision");
  private NetworkTableEntry sbVisionX = visionTab.addPersistent("Vision X", 0).getEntry();
  private NetworkTableEntry sbVisionY = visionTab.addPersistent("Vision Y", 0).getEntry();
  private NetworkTableEntry sbVisionDist = visionTab.addPersistent("Vision Dist", 0).getEntry();

  /**
   * Creates a new AimShooter.
   */
  public AutoShooterAim(Shooter shooter, DoubleSupplier dsx, DoubleSupplier dsy) {
    this.shooter = shooter;
    this.dsx = dsx;
    this.dsy = dsy;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setTiltZeroPos();
    shooter.setAngleZeroPos();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    x = dsx.getAsDouble();
    y = dsy.getAsDouble();
    distance = (ShooterConstants.kTargetHeight - ShooterConstants.kCameraHeight)
        / Math.tan(ShooterConstants.kCameraAngle + y);
    shooter.setTiltPosition(y);
    shooter.setAnglePosition(x);
    // shooter.setAnglePosition(shooter.getAnglePosition() + x);

    sbVisionX.setDouble(x);
    sbVisionY.setDouble(y);
    sbVisionDist.setDouble(distance);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
