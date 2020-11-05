package frc.robot;

import static org.junit.Assert.*;
import org.junit.Test;

public class LibraryTest {

	// Testing Clip(num, max, min)
	Library lib = new Library();
	final double DELTA = 0.000001;
	int intResult = 0;
	double dblResult = 0.0;
	double[] dblArray = {0.0, 0.0};

	@Test
	public void testClipInt() throws Exception {
		intResult = lib.Clip(0, 0, 0);
		assertEquals(0, intResult);

		intResult = lib.Clip(15, 20, 10);
		assertEquals(15, intResult);

		intResult = lib.Clip(5, 20, 10);
		assertEquals(10, intResult);

		intResult = lib.Clip(25, 20, 10);
		assertEquals(20, intResult);
	}

	@Test
	public void testClipDbl() throws Exception {
		dblResult = lib.Clip(0.0, 0.0, 0.0);
		assertEquals(0.0, dblResult, DELTA);

		dblResult = lib.Clip(15.0, 20.0, 10.0);
		assertEquals(15.0, dblResult, DELTA);

		dblResult = lib.Clip(5.0, 20.0, 10.0);
		assertEquals(10.0, dblResult, DELTA);

		dblResult = lib.Clip(25.0, 20.0, 10.0);
		assertEquals(20.0, dblResult, DELTA);
	}

	@Test
	public void testCalcSkewAngle() throws Exception {
		assertEquals(45.0, lib.calcSkewAngle(-90.0), DELTA);
		assertEquals(0.0, lib.calcSkewAngle(-45.0), DELTA);
		assertEquals(45.0, lib.calcSkewAngle(0.0), DELTA);
	}

	@Test
	public void testCalcTiltAngle() throws Exception {
		assertEquals(10.0, lib.calcTiltAngle(3.0), DELTA);
		assertEquals(20.0, lib.calcTiltAngle(10.0), DELTA);
		assertEquals(45.0, lib.calcTiltAngle(40.0), DELTA);
	}

	@Test
	public void testTgtCmd() throws Exception {
		dblArray = lib.tgtCmd(10.0, 20.0, -75.0);
		System.out.println("X " + dblArray[0] + "  Y " + dblArray[1]);
		assertEquals(-10.0, dblArray[0], DELTA);
		assertEquals(20.0, dblArray[1], DELTA);
	}
}
