/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.ChassisConstants;
import frc.robot.subsystems.Chassis;

public class AutonDriveTrajectory extends RamseteCommand {

	Chassis chassis;

	/**
	 * Constructs a new RamseteCommand that, when executed, will follow the provided
	 * trajectory. PID control and feedforward are handled internally, and outputs
	 * are scaled -12 to 12 representing units of volts.
	 *
	 * <p>
	 * Note: The controller will *not* set the outputVolts to zero upon completion
	 * of the path - this is left to the user, since it is not appropriate for paths
	 * with nonstationary endstates.
	 *
	 * @param trajectory      The trajectory to follow.
	 * @param pose            A function that supplies the robot pose - use one of
	 *                        the odometry classes to provide this.
	 * @param controller      The RAMSETE controller used to follow the trajectory.
	 * @param feedforward     The feedforward to use for the drive.
	 * @param kinematics      The kinematics for the robot drivetrain.
	 * @param wheelSpeeds     A function that supplies the speeds of the left and
	 *                        right sides of the robot drive.
	 * @param leftController  The PIDController for the left side of the robot
	 *                        drive.
	 * @param rightController The PIDController for the right side of the robot
	 *                        drive.
	 * @param outputVolts     A function that consumes the computed left and right
	 *                        outputs (in volts) for the robot drive.
	 * @param requirements    The subsystems to require.
	 */
	public AutonDriveTrajectory(Trajectory trajectory, Chassis chassis) {

		// public AutonDriveTrajectory(Trajectory trajectory, Supplier<Pose2d> pose,
		// RamseteController controller,
		// SimpleMotorFeedforward feedforward, DifferentialDriveKinematics kinematics,
		// Supplier<DifferentialDriveWheelSpeeds> wheelSpeeds, PIDController
		// leftController, PIDController rightController,
		// BiConsumer<Double, Double> outputVolts, Chassis chassis) {
		// super(trajectory, pose, controller, feedforward, kinematics, wheelSpeeds,
		// leftController, rightController,
		// outputVolts, chassis);

		super(trajectory, chassis::getPose,
				new RamseteController(ChassisConstants.kRamseteB, ChassisConstants.kRamseteZeta),
				new SimpleMotorFeedforward(ChassisConstants.ksVolts, ChassisConstants.kvVoltSecondsPerMeter,
						ChassisConstants.kaVoltSecondsSquaredPerMeter),
				chassis.m_kinematics, chassis::getWheelSpeeds,
				new PIDController(ChassisConstants.kP, ChassisConstants.kI, ChassisConstants.kD),
				new PIDController(ChassisConstants.kP, ChassisConstants.kI, ChassisConstants.kD), // RamseteCommand passes volts
																																													// to the callback
				chassis::driveTankVolts, chassis);

		this.chassis = chassis;
		addRequirements(chassis);
	}

}
