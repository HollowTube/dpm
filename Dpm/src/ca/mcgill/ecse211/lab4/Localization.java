package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;

import java.util.*;

public class Localization {
	static boolean measuring = true;

	private final static int EDGE_TRIGGER = 100;
	private static Odometer odometer;

	private MotorControl motorcontrol = Lab4.motorControl;
	double current_dist, prev_dist, change_inDist = 0;

	public boolean rising_edge() {

		current_dist = Lab4.usPoller.getDist();

		if (change_inDist > EDGE_TRIGGER) {
			change_inDist = current_dist - prev_dist;
			prev_dist = current_dist;
			return true;
		} else {
			change_inDist = current_dist - prev_dist;
			prev_dist = current_dist;
			return false;
		}
	}

	public boolean falling_edge() {
		
		current_dist = Lab4.usPoller.getDist();

		if (change_inDist < -EDGE_TRIGGER) {
			change_inDist = current_dist - prev_dist;
			prev_dist = current_dist;
			return true;
		} else {
			change_inDist = current_dist - prev_dist;
			prev_dist = current_dist;
			return false;
		}
	}

	public void go() {
		double position[];
		ArrayList<Double> crit_angles = new ArrayList<Double>();
		motorcontrol.dime_turn(1080, 150, false);
		while (motorcontrol.isMoving()) {
			if (falling_edge()) {
				Sound.beep();
			}
//			else if(falling_edge()){
//				Sound.buzz();
//			}
			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
				// there is nothing to be done here
			}
		}
	}
}
