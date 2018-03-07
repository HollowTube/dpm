import ca.mcgill.ecse211.main_package.LightPoller;
import ca.mcgill.ecse211.main_package.MotorControl;
import ca.mcgill.ecse211.main_package.myUSPoller;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.Localization.*;
import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.main_package.*;

public class test {
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
	public static void main(String args)throws OdometerExceptions{
		final MotorControl motorControl = MotorControl.getMotor(leftMotor, rightMotor);
		final Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		Nav nav = new Nav(motorControl, WHEEL_RAD, TRACK, odometer);
		Full_Localization Localize = new Full_Localization(odometer, nav, (EV3UltrasonicSensor)myDistance, motorControl, lightPollerleft, lightPollerright);
		Localize.Corner_Localize();
	}
}
