/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShooterAim extends CommandBase {

  Shooter shooter = null;
  int deg = 0;
  int counter = 0;
  double tiltInc = 35 / 20;
  double angleInc = 90 / 20;

  /**
   * Creates a new AimShooter.
   */
  public ShooterAim(Shooter shooter, int deg) {
    this.shooter = shooter;
    this.deg = deg;
    addRequirements(shooter);
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
        shooter.setTiltPosition(/* shooter.getTiltPosition() */shooter.getTiltSetPoint() + tiltInc);
        break;
      case 1:
        shooter.setTiltPosition(shooter.getTiltPosition() + tiltInc);
        shooter.setAnglePosition(shooter.getAnglePosition() + angleInc);
        break;
      case 2:
        shooter.setAnglePosition(shooter.getAnglePosition() + angleInc);
        break;
      case 3:
        shooter.setTiltPosition(shooter.getTiltPosition() - tiltInc);
        shooter.setAnglePosition(shooter.getAnglePosition() + angleInc);
        break;
      case 4:
        shooter.setTiltPosition(shooter.getTiltPosition() - tiltInc);
        break;
      case 5:
        shooter.setTiltPosition(shooter.getTiltPosition() - tiltInc);
        shooter.setAnglePosition(shooter.getAnglePosition() - angleInc);
        break;
      case 6:
        shooter.setAnglePosition(shooter.getAnglePosition() - angleInc);
        break;
      case 7:
        shooter.setTiltPosition(shooter.getTiltPosition() + tiltInc);
        shooter.setAnglePosition(shooter.getAnglePosition() - angleInc);
        break;
      }
    }
    counter++;
    SmartDashboard.putNumber("Counter", counter);
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
