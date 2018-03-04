package ca.mcgill.ecse211.Localization;

import ca.mcgill.ecse211.main_package.*;
import ca.mcgill.ecse211.Odometer.*;
import lejos.hardware.sensor.*;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class LightLocalizer {
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private int ROTATE_SPEED = 100;
	private static int LightDiff = 100;
	private static double Light_Sensor_Position;
	private Odometer odo;
	private Nav nav;
	private EV3ColorSensor LSensor;
	private float[] LSData;
	private float lastLSData;
	private double[] angles;
	private double[] xyt;
	private int lineCount = 0;

	public LightLocalizer(Odometer odo, Nav nav, EV3ColorSensor LSensor, EV3ColorSensor RSensor,
			EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		this.odo = odo;
		this.nav = nav;
		this.LSensor = LSensor;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		LSensor.getRedMode();
	}

	public void Localize() {
		double x, y;
		xyt = odo.getXYT();
		nav.turnTo(360 - xyt[2]);
		nav.turnTo(180);
		while (true) {
			if (lineDetected()) {
				lineCount++;
			}
			if (lineCount % 2 == 1) {
				xyt = odo.getXYT();
				angles[(int) (lineCount / 2)] = xyt[2];
			}
			if (lineCount == 3) {
				x = -Light_Sensor_Position * Math.cos((angles[1] - angles[0]) / 2);
				break;
			}
		}
		lineCount = 0;
		nav.turnTo(90);
		rightMotor.waitComplete();
		nav.turnTo(180);
		while (true) {
			if (lineDetected()) {
				lineCount++;
			}
			if (lineCount % 2 == 1) {
				xyt = odo.getXYT();
				angles[(int) (lineCount / 2)] = xyt[2];
			}
			if (lineCount == 3) {
				y = -Light_Sensor_Position * Math.cos(((360 + angles[1]) - angles[0]) / 2);
				break;
			}
		}
		odo.setX(x);
		odo.setY(y);
	}

	public boolean lineDetected() {
		if (LSData[0] - lastLSData > LightDiff) {
			return true;
		}
		return false;
	}

	public float getLight() {
		lastLSData = LSData[0];
		LSensor.fetchSample(LSData, 0);
		try {
			Thread.sleep(50);
		} catch (InterruptedException e) {
		}
		if (LSData[0] > 255) {
			LSData[0] = 255;
		}
		return LSData[0];
	}
}
