// Lab2.java
package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.*;
import lejos.hardware.port.Port;
import lejos.robotics.SampleProvider;
import lejos.robotics.navigation.Navigator;

public class Lab4 {

	// Motor Objects, and Robot related parameters
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	private static final Port sensorPort = LocalEV3.get().getPort("S1");
	public static final double WHEEL_RAD = 2.2;
	public static final double TRACK = 17.0;

	static Port portUS = LocalEV3.get().getPort("S2");
	static SensorModes myUS = new EV3UltrasonicSensor(portUS);
	static SampleProvider myDistance = myUS.getMode("Distance");
	static float[] sampleUS = new float[myDistance.sampleSize()];
	
	
	static EV3ColorSensor colorSensor = new EV3ColorSensor(sensorPort);
	static SampleProvider colorRGBSensor = colorSensor.getRedMode();
	static int sampleSize = colorRGBSensor.sampleSize();
	static float[] sample = new float[sampleSize];
	

	final static myUSPoller usPoller = new myUSPoller(myDistance, sampleUS);
	final static LightPoll lightPoller = new LightPoll(colorRGBSensor, sample);

	static MotorControl motorControl = new MotorControl(leftMotor, rightMotor, WHEEL_RAD, TRACK);
	

	public static void main(String[] args) throws OdometerExceptions {

		int buttonChoice;

		// Odometer related objects

		final Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		OdometryCorrection odometryCorrection = new OdometryCorrection(colorRGBSensor, sample);

		Display odometryDisplay = new Display(lcd); // No need to change

		final Localization localizer = new Localization();
		
		// clear the display
		lcd.clear();

		// ask the user whether odometery correction should be run or not
		lcd.drawString("< Left | Right >", 0, 0);
		lcd.drawString("  No   | with   ", 0, 1);
		lcd.drawString(" corr- | corr-  ", 0, 2);
		lcd.drawString(" ection| ection ", 0, 3);
		lcd.drawString("       |        ", 0, 4);

		buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)

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
				while (true) {
					sleeptime(50);
					while (Button.waitForAnyPress() != Button.ID_UP) sleeptime(50); // waits until the up button is pressed
					odometer.setXYT(0.1, 0.1, 0.001);
					try {
						localizer.localize_falling();
					} catch (OdometerExceptions e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
					motorControl.stop();
					localizer.head_to_origin();
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
