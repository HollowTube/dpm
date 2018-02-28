package ca.mcgill.ecse211.Searching;

import ca.mcgill.ecse211.Odometry.*;
import ca.mcgill.ecse211.Searching.*;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.hardware.Audio;

public class SearchLab {
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	private static final EV3ColorSensor colorSensor = new EV3ColorSensor(LocalEV3.get().getPort("S1"));
	public static final double WHEEL_RAD = 2.05;
	public static final double TRACK = 14.45;
	public static final double TILE_SIZE = 30.48;
	public static Odometer odometer;
	private static EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S2"));
	private static EV3UltrasonicSensor usSensorBlock = new EV3UltrasonicSensor(LocalEV3.get().getPort("S4"));
	public static Audio noisemaker = LocalEV3.get().getAudio();
	private static UltrasonicLocalizer usLocalizer;
	private static Navigation nav;
	private static LightLocalizer liLocalizer;
	private static int[] LL = { 2, 3 };
	private static int[] UR = { 6, 7 };
	public static String targetBlock = "blue";
	public static double xyt[] = new double[3];
	public static double currentX; // current values represent robot's X,Y,theta
	public static double currentY;
	public static double currentTheta;
	public static double deltaTheta;
	public static int[][] waypoints = { { LL[0], LL[1] }, { UR[0], LL[1] }, { UR[0], UR[1] }, { LL[0], UR[1] },
			{ LL[0], LL[1] } };
	public static double path_angle; // angle that is calculated from the normal
	public static int wpCtr = 0; // counter for order of the waypoints
	public static boolean foundBlock;
	public static Searching search;
	public static float[] sample = new float[3];
	public static boolean wasStarted;
	public static boolean COMPLETE;
	private static LightPollerColor liPol;

	static EV3ColorSensor colorSensorBlock = new EV3ColorSensor(LocalEV3.get().getPort("S3"));
	static SampleProvider colorRGBSensor = colorSensor.getRGBMode();
	static int sampleSize = colorRGBSensor.sampleSize();
	static float[] samplecolor = new float[sampleSize];

	public static void main(String[] args) throws OdometerExceptions {
		int buttonChoice;
		nav = new Navigation(leftMotor, rightMotor, WHEEL_RAD, TRACK, odometer);
		odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		search = new Searching(leftMotor, rightMotor, odometer, nav, usSensorBlock, usSensor, colorSensorBlock);
		Display odometryDisplay = new Display(lcd);

		SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance
		// provides
		// samples
		// from this
		// instance
		do {
			// clear the display
			lcd.clear();
			// ask the user for rising edge or falling edge
			lcd.drawString("< Left | Right >", 0, 0);
			lcd.drawString("       |        ", 0, 1);
			lcd.drawString("No     | Localize", 0, 2);
			lcd.drawString("Localize|         ", 0, 3);
			lcd.drawString("       |        ", 0, 4);

			buttonChoice = Button.waitForAnyPress(); // Record choice (left or
			// right press)
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT && buttonChoice != Button.ID_DOWN
				&& buttonChoice != Button.ID_UP);

		if (buttonChoice == Button.ID_UP) {

		} else if (buttonChoice == Button.ID_LEFT) {
			// ---------------------------------------------------LEFT BUTTON-----------------------

			// -------------------------------------------------------------------------------------
		} else if (buttonChoice == Button.ID_RIGHT) {
			// ---------------------------------------------------RIGHT BUTTON-----------------------
			// FULL DEMO OPTION
			Thread odoThread = new Thread(odometer);
			odoThread.start();
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();

			usLocalizer = new UltrasonicLocalizer(odometer, nav, usSensor, 2, leftMotor, rightMotor);
			try {
				Thread.sleep(1000); // sleep thread to give ultrasonic localizer
				// time to instantiate
			} catch (Exception e) {
				noisemaker.systemSound(4);
			}
			usLocalizer.Localize();

			// begin light sensor localization
			liLocalizer = new LightLocalizer(odometer, nav, colorSensor, leftMotor, rightMotor);
			liLocalizer.Localize();

			Thread searchThread = new Thread(search);

			//iterate through the waypoints
			while (wpCtr < waypoints.length) { 
				if (wpCtr == 1 && !wasStarted) { //avoid early searching and double starting
					searchThread.start();
					wasStarted = true;
				}
				while (foundBlock) { //hold process if search thread has thrown a flag
				}
				goToWaypoint(wpCtr);
				while (foundBlock) { //hold process if search thread has thrown a flag
				}
				wpCtr++;
				if (COMPLETE) { //flag for found target block
					goToWaypoint(2);
					break;
				}
			}
			// -------------------------------------------------------------------------------------
		} else if (buttonChoice == Button.ID_DOWN) {
			// ----------------------------------------DOWN BUTTON------------------------------

			// -------------------------------------------------------------------------------------
		}
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}

	public static void goToWaypoint(int wpNum) {
		xyt = odometer.getXYT(); // get the robots position on the grid
		currentX = xyt[0]; // place data in its respective variables
		currentY = xyt[1];
		currentTheta = xyt[2];
		/*
		 * The following calculations figure out the desired angle that the bot
		 * needs to turn. It uses arctan2 to get the angle in polar coordinates
		 * from (0,0) so for every coordinate it will calculate the angle for
		 * the implicit coordinate in relation to (0,0). For example: if the bot
		 * is at (2,2,45) and it needs to turn to (2,0), the logic below will
		 * figure out the angle needed to turn to (0,-2) and then subtract the
		 * current angle of the bot.
		 */
		if (wpNum > 0) {// need to make sure this is not performed for the first
			// coordinate as we assume the bot starts at 0,0
			if (waypoints[wpNum][0] >= 0 && waypoints[wpNum][1] >= 0) { // angles
				// are
				// from
				// the
				// y-axis
				// instead
				// of
				// x-axis
				path_angle = (180 / Math.PI) * Math.atan2(waypoints[wpNum][0] - (Math.round(currentX / TILE_SIZE)),
						waypoints[wpNum][1] - (Math.round(currentY / TILE_SIZE))); // angles
				// in
				// positive
				// xy-plane
			} else if (waypoints[wpNum][1] < 0) {
				path_angle = 180
						+ (180 / Math.PI) * Math.atan2(waypoints[wpNum][0] - (Math.round(currentX / TILE_SIZE)),
								waypoints[wpNum][1] - (Math.round(currentY / TILE_SIZE))); // angles
				// in
				// the
				// negative
				// y-plane
			} else {
				path_angle = 360
						+ (180 / Math.PI) * Math.atan2(waypoints[wpNum][0] - (Math.round(currentX / TILE_SIZE)),
								waypoints[wpNum][1] - (Math.round(currentY / TILE_SIZE))); // angles
				// in
				// negative
				// x-plane
				// and
				// positive
				// y-plane
			}
		} else {
			if (waypoints[wpNum][0] >= 0 && waypoints[wpNum][1] >= 0) { // angles
				// are
				// from
				// the
				// y-axis
				// instead
				// of
				// x-axis
				path_angle = (180 / Math.PI) * Math.atan2(waypoints[wpNum][0], waypoints[wpNum][1]); // angles
				// in
				// positive
				// xy-plane
			} else if (waypoints[wpNum][1] < 0) {
				path_angle = 180 + (180 / Math.PI) * Math.atan2(waypoints[wpNum][0], waypoints[wpNum][1]); // angles
				// in
				// the
				// negative
				// y-plane
			} else {
				path_angle = 360 + (180 / Math.PI) * Math.atan2(waypoints[wpNum][0], waypoints[wpNum][1]); // angles
				// in
				// negative
				// x-plane
				// and
				// positive
				// y-plane
			}
		}
		deltaTheta = path_angle - currentTheta;
		nav.turnTo(deltaTheta);
		nav.travelTo(waypoints[wpNum][0] * TILE_SIZE, waypoints[wpNum][1] * TILE_SIZE);
	}
}
