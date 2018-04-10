package ca.mcgill.ecse211.main_package;
import java.text.DecimalFormat;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;

/**
 * This class is useful for the robot to calibrate its wheel radius and its track.
 * It uses binary search to find the correct values for wheel radius and wheel track.
 * Each time, the robot is told if it went over or under and uses that to continue its search.
 * 
 * This class was made to quickly find the radius and track
 * during the development process of different hardware configurations.
 * Once the final design is decided upon, this class is not used anymore.
 * 
 * @author Tritin
 *
 */
public class Calibration {
	static MotorControl motorcontrol = MotorControl.getMotor();

	/**
	 * Method to calibrate the radius of the wheels.
	 * It finds the radius with binary search. The first middle value is 2 and then it is either
	 * the top value or lower value depending if the robot goes over or under when it stops turning.
	 * 
	 * This method is repeated until the person calibrating decides it is fit to do so.
	 * 
	 */
	public static void radius_calibration() {
		double bottom_bound = 1;
		double top_bound = 4;
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
	
	/**
	 * Method to calibrate the track of the robot. 
	 * It finds the track of the robot with binary search. The first middle value is 15
	 * and then it is either the top value or lower value depending if the robot goes over
	 * or under when it stops turning.
	 * 
	 * This is repeated until the person calibrating decides it is fit to do so.
	 * 
	 */
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
