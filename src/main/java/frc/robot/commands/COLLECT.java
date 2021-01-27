/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Hopper;

public class COLLECT extends SequentialCommandGroup {
	/**
	 * Add your docs here.
	 */

	public COLLECT(Collector collector, Hopper hopper, boolean override) {
		// new SequentialCommandGroup(new CollectorExtend(collector),
		addCommands(new CollectorExtend(collector), 
				new ParallelRaceGroup(new HopperLoad(hopper, override), new CollectorIntake(collector)),
				new ParallelRaceGroup(new CollectorReject(collector), new WaitCommand(2.0)), new CollectorStop(collector),
				new CollectorRetract(collector));

		// Add Commands here:
		// e.g. addSequential(new Command1());
		// addSequential(new Command2());
		// these will run in order.

		// To run multiple commands at the same time,
		// use addParallel()
		// e.g. addParallel(new Command1());
		// addSequential(new Command2());
		// Command1 and Command2 will run in parallel.

		// A command group will require all of the subsystems that each member
		// would require.
		// e.g. if Command1 requires chassis, and Command2 requires arm,
		// a CommandGroup containing them would require both the chassis and the
		// arm.
	}
}
