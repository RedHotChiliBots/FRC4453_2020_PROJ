/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.EjectorConstants;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;

public class HopperShoot extends CommandBase {
  /**
   * Creates a new HopperLoad.
   */
  private final Hopper hopper;
  private final Shooter shooter;

  public HopperShoot(Hopper hopper, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.hopper = hopper;
    this.shooter = shooter;
    addRequirements(hopper, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hopper.setEjectorVelocity(EjectorConstants.kEjectorShootRPMs);
    shooter.setShootVelocity(ShooterConstants.kShooterShootRPMs);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((shooter.getShootVelocity() > ShooterConstants.kShooterShootRPMs * 0.9)
        && (hopper.getEjectorVelocity() > EjectorConstants.kEjectorShootRPMs * 0.9)) {
      hopper.setHopperVelocity(HopperConstants.kHopperShootRPMs);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hopper.stopHopper();
    hopper.stopEjector();
    shooter.stopShoot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
