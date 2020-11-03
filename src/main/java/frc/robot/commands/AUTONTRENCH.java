/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;

public class AUTONTRENCH extends SequentialCommandGroup {
  /**
   * Add your docs here.
   */
  public AUTONTRENCH(Shooter shooter, Collector collector, Hopper hopper, Chassis chassis) {
		new SequentialCommandGroup(
			new ShooterInit(shooter), 
			new AutoShooterAim(shooter, () -> shooter.getTgtX(), () -> shooter.getTgtY()),
			new SHOOT(collector, hopper, shooter),
		// move to Trench and collect and shoot there
			new AutonDriveTrajectory(chassis.lineToTrenchTrajectory, chassis),
			new ParallelRaceGroup(
				new AutonDriveTrajectory(chassis.trenchPickUpTrajectory, chassis), 
				new COLLECT(collector, hopper, true)),
			new AutoShooterAim(shooter, () -> shooter.getTgtX(), () -> shooter.getTgtY()),
			new SHOOT(collector, hopper, shooter));
  }
}
