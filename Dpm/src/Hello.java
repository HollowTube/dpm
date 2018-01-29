import lejos.hardware.sensor.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.robotics.SampleProvider;
import lejos.hardware.Button;
public class Hello {
	public static void main(String[] args) {
		// Print to LCD
		System.out.println("Hello World!");
		// Wait for a button press before exiting
		Button.waitForAnyPress();
		}
}