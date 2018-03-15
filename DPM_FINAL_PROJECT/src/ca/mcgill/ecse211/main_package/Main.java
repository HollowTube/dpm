
package ca.mcgill.ecse211.main_package;

import ca.mcgill.ecse211.odometer.*;

import java.util.ArrayList;

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

public class Main {

	// Motor Objects, and Robot related parameters
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	private static final Port sensorPort = LocalEV3.get().getPort("S1");
	private static final Port sensorPortColor = LocalEV3.get().getPort("S3");
	public static final double WHEEL_RAD = 2.2;
	public static final double TRACK = 15.28;
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
	public enum List_of_states {
		IDLE, SEARCHING, IDENTIFYING, INITIALIZE, TURNING, AVOIDANCE, TRAVEL_TO_TARGET, LOCALIZE_WITH_PATH, COLOR_DEMO, RETURN_TO_PATH, TEST, ANGLE_LOCALIZATION, BRIDGE_CROSSING, TRAVELLING
	}

	static List_of_states state;

	public static void main(String[] args) throws OdometerExceptions {

		int buttonChoice;
		int current_waypoint = 0;
		
		
		// Odometer related objects
		final Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		OdometryCorrection odometryCorrection = new OdometryCorrection(colorRGBSensorReflected, sampleReflected);
		Display odometryDisplay = new Display(lcd); // No need to change

		
		//Various class initialization
		final MotorControl motorControl = MotorControl.getMotor(leftMotor, rightMotor);
		final Navigation navigator = new Navigation();
		final Angle_Localization A_loc = new Angle_Localization(lightPollerleft, lightPollerright);
		final Full_Localization Localize = new Full_Localization(myDistance, motorControl, lightPollerleft, lightPollerright);
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
		
		if (buttonChoice == Button.ID_DOWN) {
			Calibration.radius_calibration();
		}
		else if(buttonChoice == Button.ID_UP) {
			Calibration.track_calibration();
		}
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
		ArrayList<Double[]> list = new ArrayList<Double[]>();

		// spawn a new Thread to avoid SquareDriver.drive() from blocking
		(new Thread() {
			public void run() {

				// simply input waypoints here, will only update after it reaches the
				// destination
				double[][] waypoints = { { 30, 0 }, { 30, 30 }, { 0, 30 }, { 0, 0 } };
				int current_waypoint = 0;
				double xf = 0;
				double yf = 0;

				// state machine implementation, if you add any states makes sure that it does
				// not get stuck in a loop

				// set initial state
				state = List_of_states.INITIALIZE;
				while (true) {
					switch (state) {

					// TODO implement localization, set odometer to (30,30,0)
					// initial state of the robot, localization should be implemented here
					case INITIALIZE:
						try {
							Localize.Corner_Localize(1,1);
						} catch (OdometerExceptions e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						}
						state = List_of_states.IDLE;
						break;

					// do nothing until button is pressed up
					case IDLE:
						while (Button.waitForAnyPress() != Button.ID_UP)
							sleeptime(50); // waits until the up button is pressed
						state = List_of_states.INITIALIZE;
						break;
					// dime turn towards necessary destination
					case TURNING:

						Sound.beep();
						xf = waypoints[current_waypoint][0];
						yf = waypoints[current_waypoint][1];
						navigator.turn_to_destination(xf, yf);
						state = List_of_states.SEARCHING;
						break;

					// travels to waypoints while scanning for objects
					case TRAVELLING:

						navigator.travelTo(xf, yf);
						//A_loc.fix_angle();

						// triggers when the destination is reached
						if (navigator.destination_reached(xf, yf)) {
							motorControl.stop();
							current_waypoint++;
							Sound.beep();

							// resets the machine to its initial state
							if (current_waypoint == waypoints.length) {
								current_waypoint = 0;
								state = List_of_states.IDLE;

							} else {
								state = List_of_states.TURNING;
							}
							break;
						}
						break;

					case TRAVEL_TO_TARGET:

						motorControl.dime_turn(-90);
						motorControl.forward();
						while (usPoller.obstacleDetected(10)) {
						}
						motorControl.stop();
						state = List_of_states.IDENTIFYING;

					case RETURN_TO_PATH:
						// TODO subroutine to get back on the travel path should be done here
						// suggest to store the position when the object is detected and return to that
						// after

						state = List_of_states.TURNING;

						// identifies the color on screen
					case IDENTIFYING:

						// lightPoller.target_found(target_color);
						state = List_of_states.IDLE;
						break;

					case BRIDGE_CROSSING:

						// localize.localize_bridge;
						navigator.turn_to_destination(xf, yf);
						double bridge_length = 0;
						motorControl.leftRot(bridge_length, true);
						motorControl.rightRot(bridge_length, false);

					case ANGLE_LOCALIZATION:
						motorControl.forward();
						A_loc.fix_angle();
						// state = List_of_states.IDLE;
						break;

					case TEST:
						motorControl.leftRot(100, true);
						motorControl.rightRot(100, false);
						// A_loc.fix_angle();
						motorControl.stop();
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
