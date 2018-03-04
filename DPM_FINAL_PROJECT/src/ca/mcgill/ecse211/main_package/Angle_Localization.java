package ca.mcgill.ecse211.main_package;

import ca.mcgill.ecse211.odometer.*;

public class Angle_Localization {

	private static Odometer odometer;
	private static MotorControl motorcontrol;
	private LightPoller left_sensor;
	private LightPoller right_sensor;
	private int threshold = 25;

	public Angle_Localization(LightPoller L_sens, LightPoller R_sens) throws OdometerExceptions {
		this.odometer = Odometer.getOdometer();
		this.motorcontrol = MotorControl.getMotor();
		this.left_sensor = L_sens;
		this.right_sensor = R_sens;
	}

	public void fix_angle() {
		while (true) {
			if (right_sensor.lessThan(threshold)) {
				motorcontrol.rightStop();
				do {
					if (left_sensor.lessThan(threshold)) {
						motorcontrol.leftStop();
						break;
					}
				} while (true);
				break;
			} else if (left_sensor.lessThan(threshold)) {
				motorcontrol.leftStop();
				do {
					if (right_sensor.lessThan(threshold)) {
						motorcontrol.rightStop();
						break;
					}
				} while (true);
				break;
			}
		}
		double heading = odometer.getXYT()[2];
		if (heading > 315 && heading < 45) {
			odometer.setTheta(0);
		} else if (heading > 45 && heading < 135) {
			odometer.setTheta(90);
		} else if (heading > 135 && heading < 225) {
			odometer.setTheta(180);
		} else if (heading > 225 && heading < 315) {
			odometer.setTheta(270);
		}
	}

}
