package ca.mcgill.ecse211.main_package;
import java.text.DecimalFormat;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.lcd.TextLCD;

/**
 * This class is useful for the robot to calibrate its wheel radius and its track.
 * 
 * @author Tritin
 *
 */
public class Calibration {
	static MotorControl motorcontrol = MotorControl.getMotor();

	public static void radius_calibration() {
		double bottom_bound = 1;
		double top_bound = 3;
		double mid;
		int button_choice;
		LCD.clear();
		
		while (true) {
			mid = (top_bound-bottom_bound)/2 + bottom_bound;
			motorcontrol.setRadius(mid);
			motorcontrol.moveSetDistance(30.48);
			
			DecimalFormat numberFormat = new DecimalFormat("######0.00");
			LCD.drawString("Current:" + numberFormat.format(mid), 0, 0);
			LCD.drawString("<-Over || Under->", 0, 1);
			button_choice = Button.waitForAnyPress();
			
			//undershoot
			if(button_choice == Button.ID_LEFT) {
				bottom_bound = mid;
			}
//			overshoot
			else if(button_choice == Button.ID_RIGHT) {
				top_bound = mid;
			}
			LCD.clear();
		}
		
	}	
	
	public static void track_calibration() {
		double bottom_bound = 14;
		double top_bound = 17;
		double mid;
		int button_choice;
		LCD.clear();
		
		while (true) {
			mid = (top_bound-bottom_bound)/2 + bottom_bound;
			motorcontrol.setTrack(mid);
			motorcontrol.dimeTurn(720);
			
			
			DecimalFormat numberFormat = new DecimalFormat("######0.0000");
			LCD.drawString("Current:" + numberFormat.format(mid), 0, 0);
			LCD.drawString("<-Under || Over->", 0, 1);
			
			button_choice = Button.waitForAnyPress();
			
			//undershoot
			if(button_choice == Button.ID_LEFT) {
				bottom_bound = mid;
			}
//			overshoot
			else if(button_choice == Button.ID_RIGHT) {
				top_bound = mid;
			}
//			LCD.clear();
		}
		
	}

}
