
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

/**
 * Main class of the project. It is meant to conduct the entire run and manage
 * all the different threads. All the required tasks will be called upon here
 * for the robot to conduct.
 * This includes the initial localization, navigation, obstacle crossing, field localization and 
 * search and identification of flag (eventually).
 *<p>
 * Main class, meant to conduct the entire run and manage all the different
 * threads.
 * <p>
>>>>>>> branch 'master' of https://github.com/HollowTube/dpm.git
 * 
 * @author Tritin, Alexandre, Matthew
 */
public class Main {

	// Motor Objects, and Robot related parameters
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	private static final Port leftPort = LocalEV3.get().getPort("S1");
	private static final Port sensorPortColor = LocalEV3.get().getPort("S3");

	public static final double WHEEL_RAD = 2.06;
	public static final double TRACK = 14.46;
	public static final double TILE_SIZE = 30.48;

	// ultrasonic sensor initialization

	static Port portUS = LocalEV3.get().getPort("S2");
	static SensorModes myUS = new EV3UltrasonicSensor(portUS);
	static SampleProvider myDistance = myUS.getMode("Distance");
	static float[] sampleUS = new float[myDistance.sampleSize()];

	// left light sensor initialization
	static EV3ColorSensor colorSensorReflected = new EV3ColorSensor(leftPort);
	static SampleProvider colorRGBSensorReflected = colorSensorReflected.getRedMode();
	static int sampleSizeReflected = colorRGBSensorReflected.sampleSize();
	static float[] sampleReflected = new float[sampleSizeReflected];

	// right light sensor intialization
	static EV3ColorSensor colorSensor = new EV3ColorSensor(sensorPortColor);
	static SampleProvider colorRGBSensor = colorSensor.getRedMode();
	static int sampleSize = colorRGBSensor.sampleSize();
	static float[] sample = new float[sampleSize];

	// final static LightPollerColor lightPoller = new
	// LightPollerColor(colorRGBSensor, sample);

	// initialization of poller classes
	final static myUSPoller usPoller = new myUSPoller(myDistance, sampleUS);
	final static LightPoller lightPollerleft = new LightPoller(colorRGBSensorReflected, sampleReflected);
	final static LightPoller lightPollerright = new LightPoller(colorRGBSensor, sample);

	// TODO heading correction to be done before every turn
	// TODO convert parameters of course into workable coordinates
	public enum List_of_states {
		IDLE, SEARCHING, IDENTIFYING, INITIALIZE, TURNING, AVOIDANCE, COLOR_DEMO, RETURN_TO_PATH, TEST, ANGLE_LOCALIZATION, BRIDGE_CROSSING, TRAVELLING, TILE_LOCALIZATION, TUNNEL_CROSSING
	}

	static List_of_states state;

	/**
	 * Main method to run the program.
	 * 
	 * @param args
	 * @throws OdometerExceptions
	 */
	public static void main(String[] args) throws OdometerExceptions {

		int buttonChoice;
		// Odometer related objects
		final Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		OdometryCorrection odometryCorrection = new OdometryCorrection(colorRGBSensorReflected, sampleReflected);
		Display odometryDisplay = new Display(lcd); // No need to change

		// Various class initialization
		final MotorControl motorControl = MotorControl.getMotor(leftMotor, rightMotor, WHEEL_RAD, TRACK);
		final Navigation navigator = new Navigation();
		final Angle_Localization A_loc = new Angle_Localization(lightPollerleft, lightPollerright);
		final Full_Localization Localize = new Full_Localization(myDistance, motorControl, lightPollerleft,
				lightPollerright);

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
		} else if (buttonChoice == Button.ID_UP) {
			Calibration.track_calibration();
		}
		
		else if(buttonChoice == Button.ID_ENTER) {
			Testing.straightLine();
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

		// simply input waypoints here, will only update after it reaches the
		// destination
		double[][] waypoints = { { TILE_SIZE*2, 0 }, { TILE_SIZE*2, TILE_SIZE*2 }, { 0, TILE_SIZE*2 }, { 0, 0 } };
		int current_waypoint = 0;
		double xf = 0;
		double yf = 0;

		// state machine implementation, if you add any states makes sure that it does
		// not get stuck in a loop

		// set initial state
		state = List_of_states.IDLE;
		while (true) {
			switch (state) {

			case INITIALIZE:
//				 try {
//				 Localize.Corner_Localize(1,1);
//				 } catch (OdometerExceptions e) {
//				 // TODO Auto-generated catch block
//				 e.printStackTrace();
//				 }
				//odometer.setXYT(0.01, 0.01, 0.01);
				state = List_of_states.IDLE;
				break;

			// do nothing until button is pressed up
			case IDLE:
				
				while (Button.waitForAnyPress() != Button.ID_UP)
					sleeptime(50); // waits until the up button is pressed
				odometer.setXYT(0.01, 0.01, 0.01);
				state = List_of_states.BRIDGE_CROSSING;
				break;
			// dime turn towards necessary destination
			case TURNING:
				
				

				Sound.beep();
				xf = waypoints[current_waypoint][0];
				yf = waypoints[current_waypoint][1];
				navigator.turn_to_destination(xf, yf);
				state = List_of_states.TRAVELLING;
				break;

			// travels to waypoints while scanning for objects
			case TRAVELLING:

				//navigator.travelTo(xf, yf);
				motorControl.forward();
				A_loc.fix_angle_on_path();

				// triggers when the destination is reached
				if (navigator.destination_reached(xf, yf)) {
					motorControl.stop();
					current_waypoint++;
					Sound.beep();

					// resets the machine to its initial state
					if (current_waypoint == waypoints.length) {
						current_waypoint = 0;
						state = List_of_states.IDLE;

					}
					else {
						state = List_of_states.TURNING;
					}
					break;
				}
				break;
			
			case TILE_LOCALIZATION:
				try {
					 Localize.Tile_Localize(1,1);
					} catch (OdometerExceptions e) {
					 e.printStackTrace();
					}
				state =List_of_states.IDLE;
				break;
				
			case IDENTIFYING:
				
				state = List_of_states.IDLE;
				break;

			case BRIDGE_CROSSING:
				
				navigator.offset90(0,30);
				motorControl.moveSetDistance(15.8);
				motorControl.dimeTurn(87);
				motorControl.moveSetDistance(100);
				try {
					Localize.Tile_Localize(0, 4);
				} catch (OdometerExceptions e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
					
				}
				state = List_of_states.IDLE;
				break;
			case TUNNEL_CROSSING:
				navigator.offset90(0,30);
				motorControl.moveSetDistance(15);
				motorControl.dimeTurn(87);
//				double bridge_length = 90;
				motorControl.moveSetDistance(100);
				try {
					Localize.Tile_Localize(0, 4);
				} catch (OdometerExceptions e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
					
				}
				state = List_of_states.IDLE;
				break;
				

			case TEST:
				motorControl.forward();
//				while(!lightPollerleft.lessThan(30));
//				motorControl.stop();
			
//				state = List_of_states.IDLE;
				break;
			default:
				break;
			}
			sleeptime(10);
		}

	}

	/**
	 * This method sets a time for thread sleep.
	 * 
	 * @param time sleep time
	 */
	public static void sleeptime(int time) {
		try {
			Thread.sleep(time);
		} catch (InterruptedException e) {
			// There is nothing to be done here
		}
	}

}
