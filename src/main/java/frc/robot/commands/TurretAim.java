/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class TurretAim extends CommandBase {

  Turret turret = null;
  int deg = 0;
  int counter = 0;
  double tiltInc = 2.0;
  double angleInc = 5.0;

  /**
   * Creates a new Aimturret.
   */
  public TurretAim(Turret turret, int deg) {
    this.turret = turret;
    this.deg = deg;
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((counter % 25) == 0) {
      switch (deg) {
      case 0:
        turret.setTiltTarget(turret.getTiltPosition() + tiltInc);
        break;
      case 1:
        turret.setTiltTarget(turret.getTiltPosition() + tiltInc);
        turret.setAngleTarget(turret.getAnglePosition() + angleInc);
        break;
      case 2:
        turret.setAngleTarget(turret.getAnglePosition() + angleInc);
        break;
      case 3:
        turret.setTiltTarget(turret.getTiltPosition() - tiltInc);
        turret.setAngleTarget(turret.getAnglePosition() + angleInc);
        break;
      case 4:
        turret.setTiltTarget(turret.getTiltPosition() - tiltInc);
        break;
      case 5:
        turret.setTiltTarget(turret.getTiltPosition() - tiltInc);
        turret.setAngleTarget(turret.getAnglePosition() - angleInc);
        break;
      case 6:
        turret.setAngleTarget(turret.getAnglePosition() - angleInc);
        break;
      case 7:
        turret.setTiltTarget(turret.getTiltPosition() + tiltInc);
        turret.setAngleTarget(turret.getAnglePosition() - angleInc);
        break;
      }
    }
    counter++;
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
