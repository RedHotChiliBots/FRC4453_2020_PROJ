package frc.robot;

import static org.junit.Assert.*;
import org.junit.Test;

public class VisionTest {

	// Testing Clip(num, max, min)
	Library lib = new Library();
	final double DELTA = 0.000001;

	@Test
	public void testCalcSkewAngle() throws Exception {
		assertEquals(0.0, lib.calcSkewAngle(-90.0), DELTA);
		assertEquals(20.0, lib.calcSkewAngle(-70.0), DELTA);
		assertEquals(40.0, lib.calcSkewAngle(-50.0), DELTA);
		assertEquals(40.0, lib.calcSkewAngle(-40.0), DELTA);
		assertEquals(20.0, lib.calcSkewAngle(-20.0), DELTA);
		assertEquals(0.0, lib.calcSkewAngle(0.0), DELTA);
	}

	@Test
	public void testCalcTiltAngle() throws Exception {
		assertEquals(10.0, lib.calcTiltAngle(2.9), DELTA);
		assertEquals(10.0, lib.calcTiltAngle(3.1), DELTA);
		assertEquals(12.5, lib.calcTiltAngle(3.9), DELTA);
		assertEquals(20.0, lib.calcTiltAngle(10.0), DELTA);
		assertEquals(45.0, lib.calcTiltAngle(39.9), DELTA);
		assertEquals(45.0, lib.calcTiltAngle(40.1), DELTA);
		assertEquals(45.0, lib.calcTiltAngle(40.9), DELTA);
	}

	@Test
	public void testTgtCmd() throws Exception {
		assertEquals(-10.0, lib.calcTgtCmd(10.0, 20.0, -75.0)[0], DELTA);
		assertEquals(19.0, lib.calcTgtCmd(10.0, 20.0, -75.0)[1], DELTA);

		assertEquals(0.0, lib.calcTgtCmd(0.0, 50.0, -45.0)[0], DELTA);
		assertEquals(15.0, lib.calcTgtCmd(0.0, 50.0, -45.0)[1], DELTA);

		assertEquals(0.0, lib.calcTgtCmd(0.0, 1.0, -45.0)[0], DELTA);
		assertEquals(45.0, lib.calcTgtCmd(0.0, 1.0, -45.0)[1], DELTA);

		assertEquals(0.0, lib.calcTgtCmd(0.0, 10.0, -40.0)[0], DELTA);
		assertEquals(33.0, lib.calcTgtCmd(0.0, 10.0, -40.0)[1], DELTA);
	}
}
