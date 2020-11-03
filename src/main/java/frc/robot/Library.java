/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import frc.robot.Constants.ShooterConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Library {

	public double[] tgtCmd(double x, double y, double skew) {
		double[] cmd = { 0.0, 0.0 };
		double skewAngle = 0.0;

		skewAngle = calcSkewAngle(skew);

		cmd[0] = -x;
		double distHorz = ShooterConstants.kDistToTarget / Math.tan(Math.toRadians(y));
		double distOffset = distHorz * Math.tan(Math.toRadians(skewAngle));
		double distTarget = Math.sqrt(Math.pow(ShooterConstants.kDistToTarget, 2) + Math.pow(distHorz, 2) +
				Math.pow(distOffset, 2));
		cmd[1] = calcTiltAngle(distTarget);

		return cmd;
	}

	private Map<Integer, Double> conv = new HashMap<Integer, Double>()
		{

		private static final long serialVersionUID = 1L;

		{
			put(3, 10.0);
			put(4, 12.5);
			put(5, 15.0);
			put(6, 16.0);
			put(7, 17.0);
			put(8, 18.0);
			put(9, 19.0);
			put(10, 20.0);
			put(11, 21.0);
			put(12, 22.0);
			put(13, 23.0);
			put(14, 24.0);
			put(15, 25.0);
			put(16, 26.0);
			put(17, 27.0);
			put(18, 28.0);
			put(19, 29.0);
			put(20, 30.0);
			put(21, 31.0);
			put(22, 32.0);
			put(23, 33.0);
			put(24, 34.0);
			put(25, 35.0);
			put(26, 36.0);
			put(27, 37.0);
			put(28, 38.0);
			put(29, 39.0);
			put(30, 40.0);
			put(31, 41.0);
			put(32, 42.0);
			put(33, 43.0);
			put(34, 44.0);
			put(35, 45.0); 
			put(36, 45.0); 
			put(37, 45.0); 
			put(38, 45.0); 
			put(39, 45.0);
			put(40, 45.0);
		}};

	public double calcTiltAngle(double dist) {
		dist = Clip(dist, 3, 40);
		return conv.get((int)Math.round(dist));
	}

	public double calcSkewAngle(double skew) {
		return Math.abs(skew+45.0);
	}

	public double Clip(double value, double max, double min) {
		return Math.min(Math.max(value, min), max);
	}

	public int Clip(int value, int max, int min) {
		return Math.min(Math.max(value, min), max);
	}
}
