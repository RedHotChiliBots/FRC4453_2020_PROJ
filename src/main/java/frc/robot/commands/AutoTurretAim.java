/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class AutoTurretAim extends CommandBase {

  Turret turret = null;
  double tilt = 0.0;
  double yaw = 0.0;
  DoubleSupplier dsTilt = null;
  DoubleSupplier dsYaw = null;

  /**
   * Creates a new Aimturret.
   */
  public AutoTurretAim(DoubleSupplier dsTilt, DoubleSupplier dsYaw, Turret turret) {
    this.turret = turret;
    this.dsTilt = dsTilt;
    this.dsYaw = dsYaw;
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tilt = dsTilt.getAsDouble();
    yaw = dsYaw.getAsDouble();

    turret.setTiltTarget(tilt);
    turret.setAngleTarget(yaw);
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
