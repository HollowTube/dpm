package ca.mcgill.ecse211.Localization;

import ca.mcgill.ecse211.main_package.LightPoller;
import ca.mcgill.ecse211.main_package.MotorControl;
import ca.mcgill.ecse211.odometer.*;

/**
 * This class is in charge to make the robot's orientation exactly pointing in
 * the right direction. It uses two light sensors to keep the orientation of the
 * robot always straight which is necessary since the robot will always move in
 * 90 degree angles.
 * 
 * @author Tritin and Alexandre
 *
 */
public class Angle_Localization {

	private static Odometer odometer;
	private static MotorControl motorcontrol;
	private LightPoller left_sensor;
	private LightPoller right_sensor;
	private float thresholdRight = 21f;
	private float thresholdLeft = 17f;

	double initial_position[];
	double current_position[];
	private boolean recovery = false;
	long startTime, endTime;

	/**
	 * Class constructor.
	 * 
	 * @author Tritin
	 * @param L_sens
	 *            Left Light Sensor
	 * @param R_sens
	 *            Right Light Sensor
	 * @throws OdometerExceptions
	 */
	public Angle_Localization(LightPoller L_sens, LightPoller R_sens, UltrasonicLocalizer USL)
			throws OdometerExceptions {
		Angle_Localization.odometer = Odometer.getOdometer();
		Angle_Localization.motorcontrol = MotorControl.getMotor();
		this.left_sensor = L_sens;
		this.right_sensor = R_sens;
	}

	/**
	 * Method to stop the robot so that the wheel track is parallel to grid line.
	 * The robot must be put in forward motion before calling the method.
	 * <p>
	 * The light sensor will stop its motor when it detects the line. The other
	 * motor will continue to approach the line until it detects it and stops. It
	 * will then correct the odometer angle heading.
	 * 
	 * This method will block
	 * 
	 * @author Tritin Truong
	 */
	public void fix_angle() {
		while (true) {
			if (right_sensor.lessThan(thresholdRight)) {
				motorcontrol.rightStop();
				do {
					if (left_sensor.lessThan(thresholdLeft)) {
						motorcontrol.leftStop();
						break;
					}
				} while (true);
				break;
			} else if (left_sensor.lessThan(thresholdLeft)) {
				motorcontrol.leftStop();
				do {
					if (right_sensor.lessThan(thresholdRight)) {
						motorcontrol.rightStop();
						break;
					}
				} while (true);
				break;
			}
		}
		angle_correction();
	}

	/**
	 * Method to stop the robot while on path with a wheel track parallel to the
	 * grid line. The robot must be put in forward motion before calling the method.
	 * <p>
	 * The light sensor will stop its motor when it detects the line. The other
	 * motor will continue to approach the line until it detects it and stops. It
	 * will then correct the odometer angle heading.
	 * <p>
	 * This method will not block.
	 * 
	 * @author Tri-tin Truong
	 */
	public void fix_angle_on_path() {
		if (right_sensor.lessThan(thresholdRight) && !recovery) {
			motorcontrol.rightStop();
			while (!left_sensor.lessThan(thresholdLeft)) {
				try {
					Thread.sleep(10);
				} catch (InterruptedException e) {

					e.printStackTrace();
				}
			}
			motorcontrol.leftStop();
			angle_correction();
			startTime = System.currentTimeMillis();
			initial_position = odometer.getXYT();
			recovery = true;

		} else if (left_sensor.lessThan(thresholdLeft) && !recovery) {
			motorcontrol.leftStop();
			while (!right_sensor.lessThan(thresholdRight)) {
				try {
					Thread.sleep(10);
				} catch (InterruptedException e) {

					e.printStackTrace();
				}
			}
			motorcontrol.rightStop();
			angle_correction();
			startTime = System.currentTimeMillis();
			initial_position = odometer.getXYT();
			recovery = true;

		} else if (recovery) {
			endTime = System.currentTimeMillis();
			if(endTime-startTime > 1000){
				recovery = false;
			}
		}
	}

	/**
	 * Method to correct the odometer angle. Takes the previous heading to know in
	 * which direction the robot is facing. Sets the angle to the heading it is
	 * supposed to be, since the robot is parallel with the grid line.
	 * 
	 * @author Alexandre Coulombe
	 */
	private void angle_correction() {

		double heading = odometer.getXYT()[2];
		if (heading > 315 || heading < 45) {
			odometer.setTheta(0);
		} else if (heading > 45 && heading < 135) {
			odometer.setTheta(90);
		} else if (heading > 135 && heading < 225) {
			odometer.setTheta(180);
		} else if (heading > 225 && heading < 315) {
			odometer.setTheta(270);
		}
	}

	/**
	 * Method to calculate the distance error between two points using the change in
	 * x and in y directions.
	 * 
	 * @param dx
	 *            Change in x direction
	 * @param dy
	 *            Change in y direction
	 * @return Distance error
	 */
	private static double euclidian_error(double dx, double dy) {
		double error = Math.sqrt(dx * dx + dy * dy);
		return error;
	}
}
