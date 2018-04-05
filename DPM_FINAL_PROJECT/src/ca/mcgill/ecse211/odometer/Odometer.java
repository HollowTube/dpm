/**
 * This class is meant as a skeleton for the odometer class to be used.
 * 
 * @author Rodrigo Silva
 * @author Dirk Dubois
 * @author Derek Yu
 * @author Karim El-Baba
 * @author Michael Smith
 */

package ca.mcgill.ecse211.odometer;

import lejos.hardware.motor.EV3LargeRegulatedMotor;


/**
 * This class handles the odometer working as a thread during the run.
 * The Odometer is used by the robot to know its position on the field at all times.
 * It uses its wheel radius and converts it to a distance by the number of rotation it does.
 * It uses the TachoCount already in place inside the motors to know how far the it has
 * traveled in the field.
 * <p>
 * It also calculates change in orientation with calculations from the wheel radius
 * as well as the robot track.
 * This class handles all positioning and orientation that is useful for navigation,
 * localization and search algorithm.
 * 
 * @author Tritin
 *
 */
public class Odometer extends OdometerData implements Runnable {

	private static Odometer odo = null; // Returned as singleton

	// Motors and related variables
	private int leftMotorTachoCount;
	private int rightMotorTachoCount;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;

	private final double TRACK;
	private final double WHEEL_RAD;

	private double[] position = { 0, 0, 0 }; // initialize initial position here

	private static final long ODOMETER_PERIOD = 25; // odometer update period in ms

	/**
	 * This is the default constructor of this class. It initiates all motors and
	 * variables once. It cannot be accessed externally.
	 * 
	 * @param leftMotor Left Motor
	 * @param rightMotor Right Motor
	 * @throws OdometerExceptions
	 */
	private Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, final double TRACK,
			final double WHEEL_RAD) throws OdometerExceptions {
													// manipulation methods
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;

		// Reset the values of x, y and z to 0

		this.leftMotorTachoCount = 0;
		this.rightMotorTachoCount = 0;

		this.TRACK = TRACK;
		this.WHEEL_RAD = WHEEL_RAD;

	}

	/**
	 * This method is meant to ensure only one instance of the odometer is used
	 * throughout the code.
	 * 
	 * @param leftMotor Left Motor
	 * @param rightMotor Right Motor
	 * @return new or existing Odometer Object
	 * @throws OdometerExceptions
	 */
	public synchronized static Odometer getOdometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			final double TRACK, final double WHEEL_RAD) throws OdometerExceptions {
		if (odo != null) { // Return existing object
			return odo;
		} else { // create object and return it
			odo = new Odometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
			return odo;
		}
	}

	/**
	 * This class is meant to return the existing Odometer Object. It is meant to be
	 * used only if an odometer object has been created
	 * 
	 * @return Error if no previous odometer exists
	 */
	public synchronized static Odometer getOdometer() throws OdometerExceptions {

		if (odo == null) {
			throw new OdometerExceptions("No previous Odometer exits.");

		}
		return odo;
	}

	/**
	 * This method is where the logic for the odometer will run as a thread. It uses the methods
	 * provided from the OdometerData class to implement the odometer.
	 */
	// run method (required for Thread)
	public static double deltaL;
	public static double deltaR;

	public void run() {
		long updateStart, updateEnd;
		int prevLeftCount = 0;
		int prevRightCount = 0;
		double deltaD, deltaT = 0;
		double dx, dy;
		double theta = 0;

		while (true) {
			updateStart = System.currentTimeMillis();

			leftMotorTachoCount = leftMotor.getTachoCount();
			rightMotorTachoCount = rightMotor.getTachoCount();
			// gets current tachometer count
			//System.out.println(odo.getXYT()[2]);
			theta = odo.getXYT()[2]*Math.PI/180;
//			System.out.println(theta*180/Math.PI);

			deltaL = Math.PI * WHEEL_RAD * (leftMotorTachoCount - prevLeftCount) / 180; // calculates the distance each
																						// wheel has travelled
			deltaR = Math.PI * WHEEL_RAD * (rightMotorTachoCount - prevRightCount) / 180;
			prevLeftCount = leftMotorTachoCount; // updates the prev tacho count
			prevRightCount = rightMotorTachoCount;

			deltaD = 0.5 * (deltaL + deltaR);
			deltaT = (deltaL - deltaR) / TRACK;
			

			dx = deltaD * Math.sin(theta);
			dy = deltaD * Math.cos(theta);
			theta+= deltaT;

			odo.update(dx, dy, deltaT * 180 / Math.PI);

			// this ensures that the odometer only runs once every period
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < ODOMETER_PERIOD) {
				try {
					Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					// there is nothing to be done
				}
			}
		}
	}

}
