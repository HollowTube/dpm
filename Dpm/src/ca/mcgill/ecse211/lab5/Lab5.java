
package ca.mcgill.ecse211.lab5;

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
import org.apache.commons.math3.distribution.NormalDistribution;
import org.apache.commons.math3.exception.MathArithmeticException;


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
	static SampleProvider colorRGBSensor = colorSensor.getRGBMode();
	static int sampleSize = colorRGBSensor.sampleSize();
	static float[] sample = new float[sampleSize];

	final static myUSPoller usPoller = new myUSPoller(myDistance, sampleUS);
	//final static LightPollerColor lightPoller = new LightPollerColor(colorRGBSensor, sample);
	final static LightPoller lightPollerReflected = new LightPoller(colorRGBSensorReflected, sampleReflected);

	public enum List_of_states {
		IDLE, SEARCHING, IDENTIFYING, INITIALIZE, TURNING, AVOIDANCE
	}

	public static void main(String[] args) throws OdometerExceptions {

		int buttonChoice;

		// Odometer related objects
		final MotorControl motorControl = MotorControl.getMotor(leftMotor, rightMotor);
		final Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		OdometryCorrection odometryCorrection = new OdometryCorrection(colorRGBSensorReflected, sampleReflected);

		Display odometryDisplay = new Display(lcd); // No need to change

		final Localization localizer = new Localization(motorControl);
		final Navigation navigator = new Navigation();
		//final Nav nav = new Nav(leftMotor, rightMotor,WHEEL_RAD, TRACK, odometer);
		//final UltrasonicLocalizer USLoc = new UltrasonicLocalizer(odometer, nav, (EV3UltrasonicSensor) myDistance, 1);
		//final LightLocalizer  lightLoc = new LightLocalizer(odometer, nav, colorSensorReflected, leftMotor, rightMotor);

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

		// Start correction if right button was pressed
		if (buttonChoice == Button.ID_RIGHT) {
			Thread odoCorrectionThread = new Thread(odometryCorrection);
			odoCorrectionThread.start();
		}

		// spawn a new Thread to avoid SquareDriver.drive() from blocking
		(new Thread() {
			public void run() {
				// motorControl.dime_turn(90,100,true);
				//simply imput waypoints here, will only update after it reaches the destination
				double[][] waypoints = { { 0, 212 }, { 212, 212 }, { 212, 0 }, { 0, 0 } };
				int i = 0;
				double xf = waypoints[0][0];
				double yf = waypoints[0][1];
				// motorControl.leftRot(50, true);
				// motorControl.rightRot(50, false);
				// set initial state
				List_of_states state = List_of_states.INITIALIZE;
				while (true) {
					switch (state) {

					case INITIALIZE:
						//USLoc.Localize();
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
						
						navigator.turn_to_heading(xf, yf);
						state = List_of_states.SEARCHING;
						break;

					case SEARCHING:

						navigator.travelTo(xf, yf);
						if (usPoller.obstacleDetected(20)) {
							motorControl.stop();
							state = List_of_states.IDENTIFYING;
						}
						if (navigator.destination_reached(xf, yf)) {
							motorControl.stop();
							sleeptime(2000);
							i++;
							xf = waypoints[i][0];
							yf = waypoints[i][1];
							Sound.beep();
							state = List_of_states.TURNING;
							break;
						}
						break;

					case IDENTIFYING:
						
						motorControl.forward(100, 100);
						while(usPoller.getDist()>10);
						motorControl.stop();
						//lightPoller.detectColor();
						state = List_of_states.AVOIDANCE;
						break;

					case AVOIDANCE:
						motorControl.go_around(100, 20);
						state = List_of_states.TURNING;
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
