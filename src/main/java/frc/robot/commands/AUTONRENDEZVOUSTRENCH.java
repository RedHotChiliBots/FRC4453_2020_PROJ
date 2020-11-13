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

public class AUTONRENDEZVOUSTRENCH extends SequentialCommandGroup {
  /**
   * Add your docs here.
   */
  public AUTONRENDEZVOUSTRENCH(Shooter shooter, Collector collector, Hopper hopper, Chassis chassis) {
   addCommands(new ShooterInit(shooter), 
			//new AutoShooterAim(shooter, () -> shooter.getTgtX(), () -> shooter.getTgtY()),
			new SHOOT(collector, hopper, shooter),
		// move to Rendevous Zone and collect and shoot there
			new AutonDriveTrajectory(chassis.lineToRendezvousTrajectory, chassis),
			new ParallelRaceGroup(new AutonDriveTrajectory(chassis.rendezvousPickUpTrajectory, chassis),
				new COLLECT(collector, hopper, false)),
			//new AutoShooterAim(shooter, () -> shooter.getTgtX(), () -> shooter.getTgtY()), 
			new SHOOT(collector, hopper, shooter),
		// Move from Rendezvous to trench, collect and shoot there
			new AutonDriveTrajectory(chassis.rendezvousToTrenchTrajectory, chassis),
			new ParallelRaceGroup(new
				AutonDriveTrajectory(chassis.trenchPickUpTrajectory, chassis),
				new COLLECT(collector, hopper, false)),
			//new AutoShooterAim(shooter, () -> shooter.getTgtX(), () -> shooter.getTgtY()),
				new SHOOT(collector, hopper, shooter));

  }
}
