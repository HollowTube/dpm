
package ca.mcgill.ecse211.main_package;

import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.Localization.*;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.*;
import lejos.hardware.port.Port;
import lejos.robotics.SampleProvider;
import lejos.robotics.navigation.Navigator;
import ca.mcgill.ecse211.Localization.*;

public class Lab5 {

	// Motor Objects, and Robot related parameters
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	private static final Port sensorPort = LocalEV3.get().getPort("S1");
	private static final Port sensorPortColor = LocalEV3.get().getPort("S3");
	public static final double WHEEL_RAD = 2.2;
	public static final double TRACK = 17.0;

	static Port portUS = LocalEV3.get().getPort("S2");
	static SensorModes myUS = new EV3UltrasonicSensor(portUS);
	static SampleProvider myDistance = myUS.getMode("Distance");
	static float[] sampleUS = new float[myDistance.sampleSize()];

	static EV3ColorSensor colorSensorReflected = new EV3ColorSensor(sensorPort);
	static SampleProvider colorRGBSensorReflected = colorSensorReflected.getRedMode();
	static int sampleSizeReflected = colorRGBSensorReflected.sampleSize();
	static float[] sampleReflected = new float[sampleSizeReflected];

	static EV3ColorSensor colorSensor = new EV3ColorSensor(sensorPortColor);
	static SampleProvider colorRGBSensor = colorSensor.getRedMode();
	static int sampleSize = colorRGBSensor.sampleSize();
	static float[] sample = new float[sampleSize];

	final static myUSPoller usPoller = new myUSPoller(myDistance, sampleUS);

	// final static LightPollerColor lightPoller = new
	// LightPollerColor(colorRGBSensor, sample);
	final static LightPoller lightPollerleft = new LightPoller(colorRGBSensorReflected, sampleReflected);
	final static LightPoller lightPollerright = new LightPoller(colorRGBSensor, sample);

	final static String target_color = "red";

	// TODO breakdown waypoints into x and y coordinates

	// TODO heading correction to be done before every turn
	// TODO convert parameters of course into workable coordinates
	// TODO extra localization on bridge and tunnel
	// TODO
	public enum List_of_states {
		IDLE, SEARCHING, IDENTIFYING, INITIALIZE, TURNING, AVOIDANCE, TRAVEL_TO_TARGET, LOCALIZE_WITH_PATH, COLOR_DEMO, RETURN_TO_PATH, TEST, ANGLE_LOCALIZATION
	}

	static List_of_states state;

	public static void main(String[] args) throws OdometerExceptions {

		int buttonChoice;

		// Odometer related objects
		final MotorControl motorControl = MotorControl.getMotor(leftMotor, rightMotor);
		final Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		OdometryCorrection odometryCorrection = new OdometryCorrection(colorRGBSensorReflected, sampleReflected);

		Display odometryDisplay = new Display(lcd); // No need to change

		final Navigation navigator = new Navigation();
		final Angle_Localization A_loc = new Angle_Localization(lightPollerleft, lightPollerright);
		// final Nav nav = new Nav(leftMotor, rightMotor,WHEEL_RAD, TRACK, odometer);
		// final UltrasonicLocalizer USLoc = new UltrasonicLocalizer(odometer, nav,
		// (EV3UltrasonicSensor) myDistance, 1);
		// final LightLocalizer lightLoc = new LightLocalizer(odometer, nav,
		// colorSensorReflected, leftMotor, rightMotor);

		// clear the display
		lcd.clear();

		// ask the user whether odometery correction should be run or not
		lcd.drawString("< Left | Right >", 0, 0);
		lcd.drawString("  No   | with   ", 0, 1);
		lcd.drawString(" corr- | corr-  ", 0, 2);
		lcd.drawString(" ection| ection ", 0, 3);
		lcd.drawString("       |        ", 0, 4);

		buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
		lcd.clear();
		// Start odometer and display threads

		Thread odoDisplayThread = new Thread(odometryDisplay);
		odoDisplayThread.start();
		Thread odoThread = new Thread(odometer);
		odoThread.start();

		// TODO make sure odometry correction works properly, adjust values as necessary
		// Start correction if right button was pressed
		if (buttonChoice == Button.ID_RIGHT) {
			Thread odoCorrectionThread = new Thread(odometryCorrection);
			odoCorrectionThread.start();
		}

		// spawn a new Thread to avoid SquareDriver.drive() from blocking
		(new Thread() {
			public void run() {

				// TODO algorithm to determine the necessary waypoints with given Lower left
				// corner and upper right corner

				// simply imput waypoints here, will only update after it reaches the
				// destination
				double[][] waypoints = { { 30, 0 }, { 30, 30 }, { 0, 30 }, { 0, 0 } };
				int i = 0;
				double xf = 0;
				double yf = 0;


				// state machine implementation, if you add any states makes sure that it does
				// not get stuck in a loop

				// set initial state
				state = List_of_states.INITIALIZE;
				while (true) {
					switch (state) {

					// TODO implement localization, set odometer to (30,30,0)
					// intial state of the robot, localization should be implemented here
					case INITIALIZE:
						odometer.setXYT(0.01, 0.01, 0.01);
						state = List_of_states.IDLE;
						break;

					// do nothing until button is pressed up
					case IDLE:
						while (Button.waitForAnyPress() != Button.ID_UP)
							sleeptime(50); // waits until the up button is pressed
						state = List_of_states.TURNING;
						break;
					// dime turn towards necessary destination
					case TURNING:

						Sound.beep();
						xf = waypoints[i][0];
						yf = waypoints[i][1];
						navigator.turn_to_heading(xf, yf);
						state = List_of_states.SEARCHING;
						break;

					// travels to waypoint while scanning for objects
					case SEARCHING:

						// TODO implement simple control feedback while the robot is travelling so that
						// it stays on course
						navigator.travelTo(xf, yf);
						A_loc.fix_angle();

						// if (usPoller2.obstacleDetected(50)) {
						// motorControl.stop();
						// state = List_of_states.TRAVEL_TO_TARGET;
						// break;
						// }

						// triggers when the destination is reached
						if (navigator.destination_reached(xf, yf)) {
							motorControl.stop();
							sleeptime(2000);
							i++;
							Sound.beep();

							// resets the machine to its initial state
							if (i > waypoints.length) {
								i = 0;
								state = List_of_states.IDLE;

							} else {
								state = List_of_states.TURNING;
							}
							break;
						}
						break;

					// TODO after the sensor pick up an object to the side, rotates 90 degrees and
					// moves
					// until the color sensor is in position
					// be sure to test this
					case TRAVEL_TO_TARGET:

						motorControl.dime_turn(-90);
						motorControl.forward(100, 100);
						while (usPoller.obstacleDetected(10)) {
						}
						motorControl.stop();
						state = List_of_states.IDENTIFYING;

						// TODO subroutine to get back on the travel path should be done here
						// suggest to store the position when the object is detected and return to that
						// after
					case RETURN_TO_PATH:

						state = List_of_states.TURNING;

						// identifies the color on screen
					case IDENTIFYING:

						// lightPoller.target_found(target_color);
						state = List_of_states.IDLE;
						break;

					case COLOR_DEMO:
						lcd.clear();
						while (usPoller.obstacleDetected(10)) {
							lcd.drawString("Oject detected", 0, 0);
							// lightPoller.detectColor();
							while (Button.waitForAnyPress() != Button.ID_UP)
								sleeptime(50); // waits until the up button is pressed

						}
						break;
					case ANGLE_LOCALIZATION:
						motorControl.forward();
						A_loc.fix_angle();
						//state = List_of_states.IDLE;
						break;

					case TEST:
						motorControl.leftRot(100, true);
						motorControl.rightRot(100, false);
						state = List_of_states.IDLE;
						break;
					}

					sleeptime(50);
				}
			}
		}).start();

		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);
	}

	public static void sleeptime(int time) {
		try {
			Thread.sleep(time);
		} catch (InterruptedException e) {
			// There is nothing to be done here
		}
	}

}
