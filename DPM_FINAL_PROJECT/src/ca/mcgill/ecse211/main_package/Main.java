
package ca.mcgill.ecse211.main_package;

import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.Localization.*;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.sensor.*;
import lejos.hardware.port.Port;
import lejos.robotics.SampleProvider;

/**
 * Main class of the project. It is meant to conduct the entire run and manage
 * all the different threads. All the required tasks will be called upon here
 * for the robot to conduct. This includes the initial localization, navigation,
 * obstacle crossing, field localization and search and identification of flag
 * (eventually).
 * <p>
 * Main class, meant to conduct the entire run and manage all the different
 * threads.
 * <p>
 * >>>>>>> branch 'master' of https://github.com/HollowTube/dpm.git
 * 
 * @author Tritin, Alexandre, Matthew
 */
public class Main {

	// Motor Objects, and Robot related parameters
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final EV3MediumRegulatedMotor usMotor = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("C"));

	// Port initializations and LCD
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	private static final Port leftPort = LocalEV3.get().getPort("S1");
	private static final Port rightPort = LocalEV3.get().getPort("S3");
	private static final Port colorPort = LocalEV3.get().getPort("S4");

	// Physical constants
	public static final double WHEEL_RAD = 3.27;
	public static final double TRACK = 15.9688;
	public static final double TILE_SIZE = 30.48;

	// ultrasonic sensor initialization
	static Port portUS = LocalEV3.get().getPort("S2");
	static SensorModes myUS = new EV3UltrasonicSensor(portUS);
	static SampleProvider myDistance = myUS.getMode("Distance");
	static float[] sampleUS = new float[myDistance.sampleSize()];

	// left light sensor initialization
	static EV3ColorSensor colorSensorReflectedLeft = new EV3ColorSensor(leftPort);
	static SampleProvider colorRGBSensorReflectedLeft = colorSensorReflectedLeft.getRedMode();
	static int sampleSizeReflectedLeft = colorRGBSensorReflectedLeft.sampleSize();
	static float[] sampleReflectedLeft = new float[sampleSizeReflectedLeft];

	// right light sensor intialization
	static EV3ColorSensor colorSensorReflectedRight = new EV3ColorSensor(rightPort);
	static SampleProvider colorRGBSensorReflectedRight = colorSensorReflectedRight.getRedMode();
	static int sampleSizeReflectedRight = colorRGBSensorReflectedRight.sampleSize();
	static float[] sampleReflectedRight = new float[sampleSizeReflectedRight];

	// // color light sensor initialization
	static EV3ColorSensor colorSensor = new EV3ColorSensor(colorPort);
	static SampleProvider colorRGBSensor = colorSensor.getRGBMode();
	static int sampleSize = colorRGBSensor.sampleSize();
	static float[] sample = new float[sampleSize];

	// initialization of poller classes
	static myUSPoller usPoller = new myUSPoller(myDistance, sampleUS);
	static LightPoller lightPollerleft = new LightPoller(colorRGBSensorReflectedLeft, sampleReflectedLeft);
	static LightPoller lightPollerRight = new LightPoller(colorRGBSensorReflectedRight, sampleReflectedRight);
	static LightPoller odoPoller = new LightPoller(colorRGBSensorReflectedLeft, sampleReflectedLeft);
	final static LightPollerColor colorPoller = new LightPollerColor(colorRGBSensor, sample);
	
	private static final long CORRECTION_PERIOD = 30;
	// State declerations
	public enum List_of_states {
		IDLE, SEARCHING, IDENTIFYING, INITIALIZE, TURNING, TEST, BRIDGE_CROSSING, TRAVELLING, TUNNEL_CROSSING, APPROACH
	}

	static List_of_states state;

	/**
	 * Main method that invokes all threads and controls state machine.
	 * 
	 * @param args
	 * @throws OdometerExceptions
	 */
	public static void main(String[] args) throws OdometerExceptions {

		// Odometer related objects
		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		OdometryCorrection odometryCorrection = new OdometryCorrection(colorRGBSensorReflectedLeft,
				sampleReflectedLeft);
		Display odometryDisplay = new Display(lcd); // No need to change

		// Various class initialization
		MotorControl motorControl = MotorControl.getMotor(leftMotor, rightMotor, usMotor, WHEEL_RAD, TRACK);
		Navigation navigator = new Navigation();
		UltrasonicLocalizer ulLoc = new UltrasonicLocalizer(odometer, myDistance, 1, motorControl);
		Angle_Localization A_loc = new Angle_Localization(lightPollerleft, lightPollerRight, ulLoc);
		Full_Localization Localize = new Full_Localization(myDistance, motorControl, lightPollerleft, lightPollerRight);
		Search search = new Search(usPoller, A_loc, navigator, colorPoller);
		Parameter_intake parameters = Parameter_intake.getParameter();

		// instance variables
		int current_waypoint = 0;
		double xf = 0;
		double yf = 0;
		boolean isHunting = false;

		// waits for wifi intake
		parameters.wifiIntake();

		int[][] waypoints = null;
		int[][] Green_waypoints = { { parameters.Green_start_coord_x(), parameters.TN_coord_y() },
				{ parameters.TN_coord_x(), parameters.TN_coord_y() },
				{ parameters.TN_end_x(parameters.TN_coord_x()), parameters.TN_end_y(parameters.TN_coord_y()) },
				{ parameters.SR_LL_x, parameters.TN_end_y(parameters.TN_coord_y()) },
				{ parameters.SR_LL_x, parameters.SR_LL_y }, // Search starts after arriving here
				{ parameters.SR_LL_x, parameters.SR_UR_y }, { parameters.SR_UR_x, parameters.SR_UR_y },
				{ parameters.SR_UR_x, parameters.SR_LL_y }, { parameters.SR_LL_x, parameters.SR_LL_y },
				{ parameters.BR_coord_x(), parameters.SR_LL_y }, { parameters.BR_coord_x(), parameters.BR_coord_y() },
				{ parameters.BR_end_x(parameters.BR_coord_x()), parameters.BR_end_y(parameters.BR_coord_y()) },
				{ parameters.Green_start_coord_x(), parameters.BR_end_y(parameters.BR_coord_y()) },
				{ parameters.Green_start_coord_x(), parameters.Green_start_coord_y() } };

		int[][] Red_waypoints = { { parameters.Red_start_coord_x(), parameters.BR_coord_y() },
				{ parameters.BR_coord_x(), parameters.BR_coord_y() },
				{ parameters.BR_end_x(parameters.BR_coord_x()), parameters.BR_end_y(parameters.BR_coord_y()) },
				{ parameters.SG_LL_x, parameters.BR_end_y(parameters.BR_coord_y()) },
				{ parameters.SG_LL_x, parameters.SG_LL_y }, // Search starts after arriving here
				{ parameters.SG_LL_x, parameters.SG_UR_y }, { parameters.SG_UR_x, parameters.SG_UR_y },
				{ parameters.SG_UR_x, parameters.SG_LL_y }, { parameters.SG_LL_x, parameters.SG_LL_y },
				{ parameters.TN_coord_x(), parameters.SG_LL_y }, { parameters.TN_coord_x(), parameters.TN_coord_y() },
				{ parameters.TN_end_x(parameters.TN_coord_x()), parameters.TN_end_y(parameters.TN_coord_y()) },
				{ parameters.Red_start_coord_x(), parameters.TN_end_y(parameters.TN_coord_y()) },
				{ parameters.Red_start_coord_x(), parameters.Red_start_coord_y() } };
		
		if (parameters.GreenTeam == 13) {
			waypoints = Green_waypoints;
		} else {
			waypoints = Red_waypoints;
		}
		
		lcd.clear();

		// Start odometer and display threads
		Thread odoDisplayThread = new Thread(odometryDisplay);
		odoDisplayThread.start();
		Thread odoThread = new Thread(odometer);
		odoThread.start();

		Thread odoCorrectionThread = new Thread(odometryCorrection);
		odoCorrectionThread.start();
		
		
		
		// set initial state
		long correctionStart, correctionEnd;
		motorControl.setLeftSpeed(150);
		motorControl.setRightSpeed(150);
		state = List_of_states.INITIALIZE;
		while (true) {
			correctionStart = System.currentTimeMillis();
			switch (state) {

			/**
			 * Localization of the robot, places the robot at designated start coordinate
			 * and sets odometer
			 */
			case INITIALIZE:
				try {
					if (parameters.GreenTeam == 13) {
						Localize.Corner_Localize(parameters.Green_start_coord_x(), parameters.Green_start_coord_y(),
								parameters.Green_start_heading());
						odometer.setXYT(TILE_SIZE * parameters.Green_start_coord_x() + 0.01,
								TILE_SIZE * parameters.Green_start_coord_y() + 0.01,
								parameters.Green_start_heading() + 0.01);
					} else if (parameters.RedTeam == 13) {
						Localize.Corner_Localize(parameters.Red_start_coord_x(), parameters.Red_start_coord_y(),
								parameters.Red_start_heading());
						odometer.setXYT(TILE_SIZE * parameters.Red_start_coord_x() + 0.01,
								TILE_SIZE * parameters.Red_start_coord_y() + 0.01,
								parameters.Red_start_heading() + 0.01);
					}
				} catch (OdometerExceptions e) {
					e.printStackTrace();
				}
				try {
					Thread.sleep(500);
				} catch (Exception e) {
				}
				state = List_of_states.TURNING;
				break;

			// do nothing until button is pressed up
			case IDLE:
				odometer.setXYT(1 * TILE_SIZE + 0.01, 1 * TILE_SIZE + 0.01, 90);
				motorControl.setLeftSpeed(190);
				motorControl.setRightSpeed(190);
				while (Button.waitForAnyPress() != Button.ID_UP)
					sleepTime(50); // waits until the up button is pressed

				state = List_of_states.TEST;
				break;
			/**
			 * Turning state, will turn towards the next waypoint and relocalize its angle
			 * Will also check if previous waypoint is same as current waypoint, at which
			 * point it will skip the current waypoint
			 * 
			 */
			case TURNING:
				sleepTime(500);
				motorControl.setLeftSpeed(150);
				motorControl.setRightSpeed(150);
				xf = waypoints[current_waypoint][0] * TILE_SIZE + 0.01;
				yf = waypoints[current_waypoint][1] * TILE_SIZE + 0.01;

				// --------------------
				// Checks if robot already at destination and skips a point if this happens
				// --------------------
				if (current_waypoint != 0 && waypoints[current_waypoint][0] == waypoints[current_waypoint - 1][0]
						&& waypoints[current_waypoint][1] == waypoints[current_waypoint - 1][1]) {
					current_waypoint++;
					if ((current_waypoint == 2 && parameters.GreenTeam == 13)
							|| (current_waypoint == 11 && parameters.RedTeam == 13)) {
						state = List_of_states.TUNNEL_CROSSING;
					} else if ((current_waypoint == 11 && parameters.GreenTeam == 13)
							|| (current_waypoint == 2 && parameters.RedTeam == 13)) {
						state = List_of_states.BRIDGE_CROSSING;
					} else if (current_waypoint == 5) {
						motorControl.turnCW();
						state = List_of_states.TURNING;
					} else if (current_waypoint == 9) {
						motorControl.turnCCW();
						Sound.beep();
						Sound.beep();
						Sound.beep();
						Sound.beep();
						Sound.beep();
						Sound.beep();
						state = List_of_states.TURNING;
					}

					break;
				}
				navigator.turn_to_destination(xf, yf);
				motorControl.backward();
				A_loc.fix_angle();
				motorControl.moveSetDistance(2);
				state = List_of_states.TRAVELLING;
				break;

			/**
			 * Travelling state: Robot moves forward until destination has arrived, will
			 * switch to a different state depending on its current waypoint
			 */
			case TRAVELLING:
				motorControl.forward();
				A_loc.fix_angle_on_path();

				// triggers when the destination is reached
				if (navigator.destination_reached(xf, yf)) {

					motorControl.moveSetDistance(3);
					current_waypoint++;

					// resets the machine to its initial state
					if (current_waypoint == waypoints.length) {
						current_waypoint = 0;
						state = List_of_states.IDLE;

						
//						do tunnel crossing
					} else if ((current_waypoint == 2 && parameters.GreenTeam == 13)
							|| (current_waypoint == 11 && parameters.RedTeam == 13)) {
						state = List_of_states.TUNNEL_CROSSING;
					} else if ((current_waypoint == 11 && parameters.GreenTeam == 13)
							|| (current_waypoint == 2 && parameters.RedTeam == 13)) {
						state = List_of_states.BRIDGE_CROSSING;
					} else if (current_waypoint == 5) {
						motorControl.turnCW();
						state = List_of_states.TURNING;
					} else if (current_waypoint == 9) {
						Sound.beep();
						Sound.beep();
						Sound.beep();
						Sound.beep();
						Sound.beep();
						Sound.beep();
						motorControl.turnCCW();
						state = List_of_states.TURNING;
					} else {
						state = List_of_states.TURNING;
					}
					break;
				} else if (isHunting && usPoller.obstacleDetected(35)) {
					state = List_of_states.APPROACH;
				}
				break;

			case BRIDGE_CROSSING:
				motorControl.stop();

				// set the new waypoints
				xf = waypoints[current_waypoint][0] + 0.01;
				yf = waypoints[current_waypoint][1] + 0.01;

				// --------------------
				// Localize before crossing
				// --------------------

				navigator.offset90(xf * TILE_SIZE, yf * TILE_SIZE);
				motorControl.moveSetDistance(15.5);
				motorControl.dimeTurn(90);
				motorControl.setLeftSpeed(150);
				motorControl.setRightSpeed(150);
				motorControl.backward();
				A_loc.fix_angle();
				motorControl.stop();

				// ---------------------------------
				// Actual crossing
				// ----------------------------------

				motorControl.setLeftSpeed(360);
				motorControl.setRightSpeed(360);
				motorControl.moveSetDistance(49.04+TILE_SIZE*parameters.BR_length());
				// --------------------
				// Localize after crossing
				// --------------------

				try {
					Localize.Tile_Localize();
					odometer.setX(xf * TILE_SIZE);
					odometer.setY(yf * TILE_SIZE);
				} catch (OdometerExceptions e) {

				}
				current_waypoint++;
				state = List_of_states.TURNING;
				break;

			case TUNNEL_CROSSING:
				// sets new waypoints
				xf = waypoints[current_waypoint][0];
				yf = waypoints[current_waypoint][1];

				// --------------------
				// Localize before crossing
				// --------------------
				navigator.offset90(xf * TILE_SIZE + 0.01, yf * TILE_SIZE + 0.01);
				motorControl.moveSetDistance(15.5);
				motorControl.dimeTurn(90);
				motorControl.backward();
				A_loc.fix_angle();
				motorControl.stop();

				// --------------------
				// Actual crossing
				// --------------------
				motorControl.setLeftSpeed(360);
				motorControl.setRightSpeed(360);
				motorControl.moveSetDistance(49.04+TILE_SIZE*parameters.BR_length());

				// --------------------
				// Localize after crossing
				// --------------------
				try {
					Localize.Tile_Localize();
					odometer.setX(xf * TILE_SIZE);
					odometer.setY(yf * TILE_SIZE);
				} catch (OdometerExceptions e) {
				}

				current_waypoint++;
				state = List_of_states.TURNING;
				break;
			case APPROACH:

				if (search.DoSearch()) {
					isHunting = false;
				}

				// --------------------
				// Turns back to original waypoint
				// --------------------
				motorControl.setLeftSpeed(220);
				motorControl.setRightSpeed(200);
				navigator.turn_to_destination(xf, yf);

				state = List_of_states.TRAVELLING;

				break;
			case TEST:
				motorControl.forward();
				A_loc.fix_angle_on_path();
				break;
			default:
				break;
			}
			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
				try {
					Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here
				}
			}
		}
	}

	/**
	 * This method sets a time for thread sleep. Includes a try-catch block.
	 * 
	 * @param time
	 *            sleep time
	 */
	public static void sleepTime(int time) {
		try {
			Thread.sleep(time);
		} catch (InterruptedException e) {
			// There is nothing to be done here
		}
	}

}
