/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Turret;

/**
 * A complex auto command that drives forward, releases a hatch, and then drives
 * backward.
 */
public class TurretAngleInit extends SequentialCommandGroup {
  /**
   * Creates a new ComplexAuto.
   *
   * @param drive The drive subsystem this command will run on
   * @param hatch The hatch subsystem this command will run on
   */
  public TurretAngleInit(Turret turret) {
    addCommands(
        // Find center magnet
        new TurretAngleFind(turret),

        // Center on magnetic field
        new TurretAngleCenter(turret));
  }
}